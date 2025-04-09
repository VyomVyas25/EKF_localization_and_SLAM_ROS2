#!/usr/bin/env python3
import rclpy, math
import numpy as np
from rclpy.node import Node
from rclpy.clock import Clock
from geometry_msgs.msg import PoseWithCovarianceStamped,PoseStamped
from nav_msgs.msg import Odometry,Path
from sensor_msgs.msg import LaserScan
from tf_transformations import euler_from_quaternion, quaternion_from_euler

def normalize_angle(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi

time_step = 5

class Ekf_Slam_Vw(Node):
    def __init__(self):
        super().__init__("ekf_slam_3rd_approach")

        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped,'/ekf_pose',10)
        self.laser_sub = self.create_subscription(LaserScan,'/scan',self.laserCB,10)
        self.control_sub = self.create_subscription(Odometry,'/odom',self.control_CB,10)
        self.count =0

        self.sigma_r = 0.1
        self.sigma_phi = 0.01

        self.initial_run = True

        self.mu = np.zeros((3,1))
        self.cov = np.zeros((3,3))
        self.landmark_count = 0
        self.time_step = time_step
    def laserCB(self,msg:LaserScan):
        while(msg.ranges==None):
            self.get_logger().warn("The lidar data is not being subscribed")
        self.laser_points = [0]*len(msg.ranges)
        for i, value in enumerate(msg.ranges[:-1]):
            if not math.isinf(value) and not math.isnan(value):
                self.laser_points[i] = value
            else:
                self.laser_points[i] = msg.range_max
    
    def control_CB(self,msg:Odometry):
        
        if self.initial_run == True:
            self.x_dash = msg.pose.pose.position.x
            self.y_dash = msg.pose.pose.position.y
            orientation_q = msg.pose.pose.orientation
            orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
            (roll, pitch, self.theta_dash) = euler_from_quaternion(orientation_list)
            self.mu[:3,0:1] = np.array([[self.x_dash, self.y_dash, self.theta_dash]]).T
            self.initial_run = False

        self.v = msg.twist.twist.linear.x
        self.w = msg.twist.twist.angular.z

        self.x_dash = msg.pose.pose.position.x
        self.y_dash = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, self.theta_dash) = euler_from_quaternion(orientation_list)
        #print(self.x_dash)
        self.count+=1

    def mu_prediction(self):

        self.x_delta = -(self.v/self.w)*math.sin(self.mu[2,0]) + (self.v/self.w)*math.sin(normalize_angle(self.mu[2,0]+self.w * self.time_step))
        self.y_delta = (self.v/self.w)*math.cos(self.mu[2,0]) - (self.v/self.w)*math.cos(normalize_angle(self.mu[2,0]+self.w * self.time_step))
        self.theta_delta = self.w * self.time_step
        self.F_x = np.zeros((3,3+2*self.landmark_count))
        self.F_x[:3,:3] = np.eye(3)

        self.mu_bar = self.mu + np.dot(self.F_x.T,(np.array([[self.x_delta,self.y_delta,self.theta_delta]]).T))
        self.mu_bar[2,0] = normalize_angle(self.mu_bar[2,0])

        self.x_bar = self.mu_bar[0,0]
        self.y_bar = self.mu_bar[1,0]
        self.theta_bar = self.mu_bar[2,0]

    def dg_dstate(self):

        G_T_x = np.array([
            [0 , 0 , -self.y_delta],
            [0 , 0 , self.x_delta ],
            [0    ,     0,       0]
        ])

        self.G_t = np.array(np.eye(3+2*self.landmark_count) + np.dot(self.F_x.T,np.dot(G_T_x,self.F_x)))

    def dg_dcontrol(self):

        dg0_dv = self.x_delta/self.v

        dg1_dv = self.y_delta/self.v

        dg2_dv = 0

        dg0_dw = ((self.v/(self.w*self.w))*math.sin(self.mu[2,0])) + \
            ((self.v * self.time_step/(self.w))*math.cos(normalize_angle(self.mu[2,0]+self.w*self.time_step))) - \
            ((self.v/(self.w*self.w))*math.sin(normalize_angle(self.mu[2,0]+self.w*self.time_step)))
        
        dg1_dw = -((self.v/(self.w*self.w))*math.cos(self.mu[2,0])) + \
            ((self.v * self.time_step/(self.w))*math.sin(normalize_angle(self.mu[2,0]+self.w*self.time_step))) + \
            ((self.v/(self.w*self.w))*math.cos(normalize_angle(self.mu[2,0]+self.w*self.time_step)))
        
        dg2_dw = self.time_step

        self.V_t_x = np.array([
            [dg0_dv,dg0_dw],
            [dg1_dv,dg1_dw],
            [dg2_dv,dg2_dw]
        ])

    def pred_meas_noise(self):

        self.alpha_1 = 0.0001
        self.alpha_2 = 0.0001

        self.noise = np.diag([self.alpha_1*(self.v*self.v),self.alpha_2*(self.w*self.w)])

        self.R_x_t = np.dot(self.V_t_x,np.dot(self.noise,self.V_t_x.T))

    def cov_pred(self):

        R_t = np.dot(self.F_x.T,np.dot(self.R_x_t,self.F_x))

        self.cov_bar = np.dot(self.G_t,np.dot(self.cov,self.G_t.T)) + R_t

    def observed_features(self):
        """
        Process the lidar scan (self.laser_points) to detect jumps.
        Each jump is assumed to correspond to a cylinder edge.
        The average ray index and depth for each detected cylinder region
        are stored in self.approx_linear_distance and self.approx_angular_position.
        """
        self.obs_curr = [[]]
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

        # For debugging, print number of detected measurements:
        # num_meas = len(self.approx_linear_distance)
        # self.get_logger().info(f"Number of cylinder measurements: {num_meas}")
        self.obs_curr = np.vstack((self.approx_linear_distance,self.approx_angular_position))

    def h_mat(self):
        # If there are no landmarks, return an empty measurement vector.
        if self.landmark_count == 0:
            self.h = np.empty((2, 0))
            return

        # Initialize arrays with shape (1, landmark_count)
        self.delta_x_cylin = np.zeros((1, self.landmark_count))
        self.delta_y_cylin = np.zeros((1, self.landmark_count))
        self.h_theta = np.zeros((1, self.landmark_count))

        # Compute the differences and the bearing for each landmark.
        for i in range(self.landmark_count):
            self.delta_x_cylin[0, i] = self.mu_bar[3 + 2 * i, 0] - self.mu_bar[0, 0]
            self.delta_y_cylin[0, i] = self.mu_bar[4 + 2 * i, 0] - self.mu_bar[1, 0]
            self.h_theta[0, i] = normalize_angle(
                math.atan2(self.delta_y_cylin[0, i], self.delta_x_cylin[0, i]) - self.mu_bar[2, 0]
            )

        # Stack delta values to form a 2 x landmark_count matrix.
        delta = np.vstack((self.delta_x_cylin, self.delta_y_cylin))
        # Compute the Euclidean norm for each landmark (column-wise).
        self.q_root = np.linalg.norm(delta, axis=0, keepdims=True)  # shape: (1, landmark_count)
        # Stack range (q_root) and bearing (h_theta) to form the measurement vector h.
        self.h = np.vstack((self.q_root, self.h_theta))


    def z_pairing_check(self):

        self.z_curr = np.zeros_like(self.h)
        self.curr_match = [False]*np.shape(self.obs_curr)[1]
        self.prev_match = [False]*(self.landmark_count)

        
        for i in range(self.landmark_count):
            for j in range(self.obs_curr.shape[1]):
                if(abs(self.h[0,i] - self.obs_curr[0,j])<0.1):
                    self.curr_match[j] = True
                    self.prev_match[i] = True
                    self.z_curr[:,i] = self.obs_curr[:,j]
                    break


    def correction_step(self):

        Q_t = np.array([
            [self.sigma_r**2, 0],
            [0, self.sigma_phi**2]
        ])

        # Save copies of current state and covariance
        self.mu_bar_copy = self.mu_bar.copy()
        self.cov_bar_copy = self.cov_bar.copy()

        state_dim = 3 + 2 * self.landmark_count

        for i in range(self.landmark_count):
            if self.prev_match[i]:
                # Create F_dash_x for landmark i with shape (5, state_dim)
                F_dash_x = np.zeros((5, state_dim))
                # First 3 rows pick the robot state
                F_dash_x[:3, :3] = np.eye(3)
                # Next 2 rows select the i-th landmark's state
                F_dash_x[3:5, 3 + 2 * i : 3 + 2 * i + 2] = np.eye(2)

                # Use proper scalar indexing (use [0,i] for 2D arrays) in the Jacobian calculation
                low_H_00 = math.sqrt(self.h[0, i]) * self.delta_x_cylin[0, i]
                low_H_10 = -self.delta_y_cylin[0, i]
                low_H_01 = math.sqrt(self.h[0, i]) * self.delta_y_cylin[0, i]
                low_H_11 = self.delta_x_cylin[0, i]
                low_H_02 = 0
                low_H_12 = -self.h[0, i]
                low_H_03 = -low_H_00
                low_H_13 = -low_H_10
                low_H_04 = -low_H_01
                low_H_14 = -low_H_11

                low_H_t = (1 / self.h[0, i]) * np.array([
                    [low_H_00, low_H_01, low_H_02, low_H_03, low_H_04],
                    [low_H_10, low_H_11, low_H_12, low_H_13, low_H_14]
                ])

                H_t = np.dot(low_H_t, F_dash_x)
                S_t = np.dot(H_t, np.dot(self.cov_bar, H_t.T)) + Q_t
                K_gain = np.dot(self.cov_bar, np.dot(H_t.T, np.linalg.inv(S_t)))
                Innovation_mat = np.array([
                    [self.z_curr[0, i] - self.h[0, i]],
                    [normalize_angle(self.z_curr[1, i] - self.h[1, i])]
                ])
                self.mu_bar = self.mu_bar + np.dot(K_gain, Innovation_mat)
                Identity = np.eye(state_dim)
                self.cov_bar = np.dot((Identity - np.dot(K_gain, H_t)), self.cov_bar)

        # For landmarks not matched, restore previous state.
        for i in range(self.landmark_count):
            if not self.prev_match[i]:
                self.mu_bar[3+2*i:3+2*i+2, 0] = self.mu_bar_copy[3+2*i:3+2*i+2, 0]
                self.cov_bar[3+2*i:3+2*i+2, 3+2*i:3+2*i+2] = \
                    self.cov_bar_copy[3+2*i:3+2*i+2, 3+2*i:3+2*i+2]


    def add_new_feature(self):

        # Increase landmark count for each new feature
        for i in range(len(self.curr_match)):
            if not self.curr_match[i]:
                self.landmark_count += 1

        first_new_cylin = True
        self.cov_dash = np.eye(3 + 2 * self.landmark_count)
        cov_bar_shape, _ = np.shape(self.cov_bar)
        self.cov_dash[:cov_bar_shape, :cov_bar_shape] = self.cov_bar
        self.cov_bar = self.cov_dash

        # Initialize extra_landmarks to None
        extra_landmarks = None

        for i in range(len(self.curr_match)):
            if not self.curr_match[i]:
                r_new_cylin = self.obs_curr[0, i]
                theta_new_cylin = self.obs_curr[1, i]
                x_new_cylin = self.x_bar + r_new_cylin * math.cos(normalize_angle(theta_new_cylin + self.theta_bar))
                y_new_cylin = self.y_bar + r_new_cylin * math.sin(normalize_angle(theta_new_cylin + self.theta_bar))
                
                if extra_landmarks is None:
                    extra_landmarks = np.array([[x_new_cylin],
                                                [y_new_cylin]])
                else:
                    # Stack new landmarks along the first axis
                    extra_landmarks = np.vstack((extra_landmarks, [[x_new_cylin], [y_new_cylin]]))
        
        # Only update self.mu_bar if there are new features
        if extra_landmarks is not None:
            self.mu_bar = np.vstack((self.mu_bar, extra_landmarks))


    
    def run(self):
        self.mu_prediction()
        self.dg_dstate()
        self.dg_dcontrol()
        self.pred_meas_noise()
        self.cov_pred()
        self.observed_features()
        self.h_mat()
        self.z_pairing_check()
        self.correction_step()
        self.add_new_feature()
        self.mu = self.mu_bar
        self.cov = self.cov_bar
        error = self.mu[0:3] - np.array([[self.x_dash, self.y_dash, self.theta_dash]]).T
        print(f"error : {error}")
        print(f"covariance : {self.mu}")

        #print(self.count)

def main(args=None):
    rclpy.init(args=args)
    node = Ekf_Slam_Vw()
    # Create a timer to call the run() method periodically (e.g., every 0.1 seconds)
    timer_period = time_step # seconds
    node.create_timer(timer_period, node.run)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()