#!/usr/bin/env python3
import rclpy,math
import numpy as np
from rclpy.node import Node
from rclpy.clock import Clock
from geometry_msgs.msg import Twist,PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from ekf_localization.ekf_localization import EKF_LOCALIZATION
from tf_transformations import euler_from_quaternion,quaternion_from_euler


def normalize_angle(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi 

class EKF_SLAM(Node):

    def __init__(self):
        super().__init__('ekf_slam')

        #Publishers
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped,"/pose_with_cov_stamped",10)

        #Subscribers
        self.odom_sub = self.create_subscription(Odometry,'/odom',self.odomCb,10)
        self.laser_sub = self.create_subscription(LaserScan,'/scan',self.laserCb,10)

        #Variable_Initializations
        self.x_prev,self.y_prev,self.theta_prev =0,0,0
        self.mu_bar = np.array([[0],[0],[0]])
        self.G_t = np.eye(3)
        self.covariance_bar = np.zeros((3, 3))
        self.final_covariance = np.zeros((3, 3))
        self.R_t = np.zeros((3, 3))
        self.previous_cylinders = np.array([[]])
        self.sigma_r = 0.1 # This is to define standard deviation in the distance measurement
        self.sigma_alpha = 0.01  # This is to define the standard deviation in the angle measurement
        self.dim = 3

    def laserCb(self,msg:LaserScan):
        """This Function filters the lidar scan data and then stores it in a class variable."""
        while(msg.ranges==None):
            self.get_logger().warn("The lidar data is not being subscribed")
        self.laser_points = [0]*len(msg.ranges)
        for i, value in enumerate(msg.ranges[:-1]):
            if not math.isinf(value) and not math.isnan(value):
                self.laser_points[i] = value
            else:
                self.laser_points[i] = msg.range_max

    def odomCb(self,msg:Odometry):
        self.x=msg.pose.pose.position.x
        self.y=msg.pose.pose.position.y

        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, self.yaw) = euler_from_quaternion (orientation_list)
        self.theta = normalize_angle(self.yaw)
        self.delta_trans = math.sqrt((self.x-self.x_prev)**2+(self.y-self.y_prev)**2)
        self.delta_rot1 = normalize_angle(math.atan2(self.y - self.y_prev, self.x - self.x_prev) - self.theta_prev)
        self.delta_rot2 = normalize_angle(self.theta - self.theta_prev - self.delta_rot1)

    def pose_predictor(self):
        """This step is to predict the pose of the robot using the odometry motion model."""
        self.x_predicted = self.x_prev + self.delta_trans*math.cos(self.theta + self.delta_rot1)
        self.y_predicted = self.y_prev + self.delta_trans*math.sin(self.theta + self.delta_rot1)
        self.theta_predicted = normalize_angle(self.theta_prev + self.delta_rot1 + self.delta_rot2)


        self.mu_bar[:3] = np.array([
            [self.x_predicted],
            [self.y_predicted],
            [self.theta_predicted]
        ])

    def state_cov_pred(self):
        """This function serves as the calculation of state_covariance."""
        self.G_t[:3,:3] = np.array([
            [1 , 0  , -self.delta_trans*math.sin(self.theta_prev+self.delta_rot1)],
            [0 , 1 , self.delta_trans*math.cos(self.theta_prev+self.delta_rot1)],
            [0 , 0, 1]
        ]) #W.R.T. STATES(POSITION,ORIENTATION)

    def cont_cov_pred(self):
        """This function is used to obtain the covariance in the control signals given to the robot."""
        self.V = np.array([
            [-self.delta_trans*(math.sin(self.theta_prev+self.delta_rot1)) , math.cos(self.theta_prev + self.delta_rot1) , 0],
            [self.delta_trans*math.cos(self.theta_prev + self.delta_rot1) , math.sin(self.theta_prev + self.delta_rot1) , 0],
            [1 , 0 , 1]
        ]) #W.R.T. CONTROLS U=[DEL_R1,DEL_T,DEL_R2]  # 3+2N X 3

    def prediction_covariance_calc(self):
        """This function is used to get the exact prediction covariance."""
        alpha1 = 0.05
        alpha2 = 0.01
        alpha3 = 0.05
        alpha4 = 0.01
        self.rot1_variance = alpha1 * pow((self.delta_rot1),2) + alpha2 * pow((self.delta_trans),2)
        self.trans_variance = alpha3 * pow((self.delta_trans),2) + alpha4 * (pow((self.delta_rot1),2) + pow((self.delta_rot2),2))
        self.rot2_variance = alpha1 * pow((self.delta_rot2),2) + alpha2 * pow((self.delta_trans),2)
        control_covariance = np.diag([self.rot1_variance, self.trans_variance, self.rot2_variance]) #M_t matrix
        self.R_t[:3,:3] = np.dot(self.V, np.dot(control_covariance, self.V.T))

        self.covariance_bar[:3,:3] = np.dot(self.G_t, np.dot(self.final_covariance, self.G_t.T)) + self.R_t

    def feature_detection(self):
        """
        Process the lidar scan (self.laser_points) to detect jumps.
        Each jump is assumed to correspond to a cylinder edge.
        The average ray index and depth for each detected cylinder region
        are stored in self.approx_linear_distance and self.approx_angular_position.
        """
        # Compute jump derivatives for each laser ray
        jumps = [0.0] * len(self.laser_points)
        for i in range(1, len(self.laser_points) - 1):
            prev_point = self.laser_points[i - 1]
            next_point = self.laser_points[i + 1]
            if prev_point > 0.2 and next_point > 0.2:
                derivative = (next_point - prev_point) / 2.0
            else:
                derivative = 0.0
            if abs(derivative) > 0.1:
                jumps[i] = derivative
            # Else, jumps[i] remains 0.0
        
        # self.get_logger().info(f"Detected jumps: {jumps}")

        # Process the jumps to group rays into cylinder detections.
        self.approx_linear_distance = []
        self.approx_angular_position = []
        cylin_active = False
        no_of_rays = 0
        sum_ray_indices = 0
        sum_depth = 0.0

        i = 0
        while i < len(jumps):
            # Start of a cylinder detection: falling edge (derivative < 0)
            if jumps[i] < 0 and not cylin_active:
                cylin_active = True
                no_of_rays = 1
                sum_ray_indices = i
                sum_depth = self.laser_points[i]
            # If we are in a cylinder region and the derivative is near zero
            elif cylin_active and abs(jumps[i]) < 1e-6:
                no_of_rays += 1
                sum_ray_indices += i
                sum_depth += self.laser_points[i]
            # End of the cylinder region: rising edge (derivative > 0)
            elif jumps[i] > 0 and cylin_active:
                # Compute average index and depth
                avg_index = sum_ray_indices / no_of_rays
                avg_depth = sum_depth / no_of_rays
                # Convert ray index to angle (assuming a fixed angular resolution, e.g., 1° = 0.01745 rad)
                approx_ang = normalize_angle(avg_index * 0.01745)
                self.approx_angular_position.append(approx_ang)
                # Optionally add an offset (here +0.25 as in your code)
                self.approx_linear_distance.append(avg_depth + 0.25)
                # Reset for the next cylinder detection
                cylin_active = False
                no_of_rays = 0
                sum_ray_indices = 0
                sum_depth = 0.0
            i += 1

    def z_meas_current(self):
        """
        Build the measurement matrix z_meas with shape (2, N), where
        first row contains measured distances and the second row measured angles.
        """
        self.z_curr = np.vstack((self.approx_linear_distance, self.approx_angular_position))
        
    def z_mat_for_prev_pose(self):
        """
        For each reference cylinder (predefined in self.reference_cylin),
        compute the estimated distance and relative angle from the predicted pose.
        Also store the differences in x and y.
        """
        dist_estim = []
        angle_estim = []
        self.x_estim_diff = []
        self.y_estim_diff = []
        for ref in self.previous_cylinders:
            x_ref, y_ref = ref
            diff_x = self.x_predicted - x_ref
            diff_y = self.y_predicted - y_ref
            self.x_estim_diff.append(diff_x)
            self.y_estim_diff.append(diff_y)
            dist_estim.append(math.sqrt(diff_x**2 + diff_y**2))
            angle = math.atan2(y_ref - self.y_predicted, x_ref - self.x_predicted)
            angle_estim.append(normalize_angle(angle - self.theta_predicted))
        self.z_estim = np.vstack((dist_estim, angle_estim))
        self.diff_estim = np.vstack((self.x_estim_diff, self.y_estim_diff))

    def feature_pairing(self):
        """
        Pair the estimated measurements (z_estim) with the observed measurements (z_meas).
        Note: z_meas has shape (2, N_meas) and z_estim has shape (2, N_estim).
        The number of detected cylinders is given by z_meas.shape[1].
        """
        tolerance = 0.15  # Use a reasonable tolerance for matching
        paired_meas_dist = []
        paired_meas_angle = []
        paired_estim_dist = []
        paired_estim_angle = []
        paired_estim_diff_x = []
        paired_estim_diff_y = []

        num_estim = self.z_estim.shape[1]
        num_meas = self.z_curr.shape[1]
        matched_flags = [False] * num_meas

        for i in range(num_estim):
            for j in range(num_meas):
                if np.allclose(self.z_estim[:, i], self.z_curr[:, j], atol=tolerance):
                    matched_flags[j] = True
                    paired_meas_dist.append(self.z_curr[0, j])
                    paired_meas_angle.append(self.z_curr[1, j])
                    paired_estim_dist.append(self.z_estim[0, i])
                    paired_estim_angle.append(self.z_estim[1, i])
                    paired_estim_diff_x.append(self.diff_estim[0, i])
                    paired_estim_diff_y.append(self.diff_estim[1, i])
                    break
        unmatched_indices = [j for j, flag in enumerate(matched_flags) if not flag]

        if unmatched_indices:
            self.unmatched_measurements = np.array(self.z_curr[:, unmatched_indices])
        else:
            self.unmatched_measurements = np.array([])

        if paired_meas_dist and paired_meas_angle:
            self.paired_measurements = np.vstack((paired_meas_dist, paired_meas_angle))
        else:
            self.paired_measurements = np.array([])
        
        if paired_estim_dist and paired_estim_angle:
            self.paired_estimations = np.vstack((paired_estim_dist, paired_estim_angle))
        else:
            self.paired_estimations = np.array([])
        
        if paired_estim_diff_x and paired_estim_diff_y:
            self.paired_estim_diff = np.vstack((paired_estim_diff_x, paired_estim_diff_y))
        else:
            self.paired_estim_diff = np.array([])


    def correction(self):
        H_t_mat = np.zeros((3,3))
        """
        Update the state estimate using the EKF measurement update.
        If no measurement pairs were found, the prediction is kept.
        (Note: The Jacobian H_t_mat is set as a dummy 2x3 matrix here;
         adjust as needed for your observation model.)
        """
        if self.paired_measurements.size == 0:
            self.mu = self.mu_bar
            self.final_covariance = self.covariance_bar
        else:
            num_pairs = self.paired_measurements.shape[1]
            for i in range(num_pairs):
                meas_dist = self.paired_measurements[0, i]
                meas_ang = self.paired_measurements[1, i]
                z_matrix = np.array([[meas_dist], [meas_ang]])
                h_matrix = np.array([[self.paired_estimations[0, i]], [self.paired_estimations[1, i]]])
    
                # Here we use a dummy observation Jacobian for illustration.
                H_t_mat[:2,:3] = np.array([
                    [(-self.paired_estim_diff[0, i] / self.paired_estimations[0, i]), (-self.paired_estim_diff[1, i] / self.paired_estimations[0, i]), 0],
                    [(self.paired_estim_diff[1, i] / ((self.paired_estimations[0, i]) ** 2)), (-self.paired_estim_diff[0, i] / ((self.paired_estimations[0, i]) ** 2)), -1]
                ])

                H_obs = np.array(-H_t_mat[:2,:2])

                H_t_mat = np.hstack(H_t_mat,H_obs)
                
    
                q_matrix = np.array([
                    [self.sigma_r**2, 0],
                    [0, self.sigma_alpha**2]
                ])
    
                S = np.dot(H_t_mat, np.dot(self.covariance_bar, H_t_mat.T)) + q_matrix
                k_gain = np.dot(self.covariance_bar, np.dot(H_t_mat.T, np.linalg.inv(S)))
    
                Innovation_matrix = np.array([
                    [z_matrix[0, 0] - h_matrix[0, 0]],
                    [normalize_angle(z_matrix[1, 0] - h_matrix[1, 0])]
                ])
    
                self.mu = self.mu_bar + np.dot(k_gain, Innovation_matrix)
                Identity = np.eye(3)
                self.final_covariance = np.dot((Identity - np.dot(k_gain, H_t_mat)), self.covariance_bar)
    
        self.obs_bot_position = np.array([
            [self.x],
            [self.y],
            [self.theta]
        ])

    def add_new_cylin(self):
        state_new_landmarks = []
        for ref in self.unmatched_measurements:
            r_ref, theta_ref = ref
            x_new = self.x_predicted + r_ref*math.cos(normalize_angle(self.theta_predicted+theta_ref))
            y_new = self.y_predicted + r_ref*math.sin(normalize_angle(self.theta_predicted+theta_ref))
            state_new_landmarks.append([x_new, y_new])
            self.dim +=2

        state_new_landmarks = np.array(state_new_landmarks)
        state_new_landmarks_column = state_new_landmarks.reshape(-1, 1)
        self.state_new_landmarks = state_new_landmarks_column
        final_cov_with_new_landmarks = np.eye(self.dim)
        


    def publish_pose_with_covariance(self):
        self.mu = self.mu_bar
        self.final_covariance = self.covariance_bar
        clock = Clock()
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = clock.now().to_msg()

        pose_msg.header.frame_id = "odom"  # Use the appropriate frame of reference

        # Setting the pose based on the mean (mu)
        pose_msg.pose.pose.position.x = float(self.mu[0, 0])
        pose_msg.pose.pose.position.y = float(self.mu[1, 0])

        pose_msg.pose.pose.position.z = 0.0  # Assume planar navigation

        # Convert orientation from Euler to quaternion
        quat = quaternion_from_euler(0, 0, self.mu[2, 0])
        pose_msg.pose.pose.orientation.x = quat[0]
        pose_msg.pose.pose.orientation.y = quat[1]
        pose_msg.pose.pose.orientation.z = quat[2]
        pose_msg.pose.pose.orientation.w = quat[3]

        # Fill in the covariance (flattened row-major order)
        covariance_flat = self.final_covariance.flatten()
        pose_msg.pose.covariance = [float(covariance_flat[i]) if i < len(covariance_flat) else 0.0 for i in range(36)]


        # Publish the message
        self.pose_pub.publish(pose_msg)

        self.theta_prev = self.theta
        self.x_prev = self.x
        self.y_prev =self.y
        

    def run(self):
        self.pose_predictor()
        self.state_cov_pred()
        self.cont_cov_pred()
        self.prediction_covariance_calc()
        self.publish_pose_with_covariance()

def main(args=None):
    import rclpy
    rclpy.init(args=args)
    node = EKF_SLAM()
    timer_period = .5  # seconds
    node.create_timer(timer_period, node.run)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()