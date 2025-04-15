#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np
import math as m

def scale(angle):
    return (angle + m.pi) % (2 * m.pi) - m.pi

class EKF_SLAM(Node):
    def __init__(self):
        super().__init__('ekf_slam')
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.pub = self.create_publisher(PoseWithCovarianceStamped, '/ekf_pose', 10)
        self.timer = self.create_timer(0.25, self.timer_cb)

        # Initialize robot state (pose only)
        self.x_prev, self.y_prev, self.theta_prev = 0.0, 0.0, 0.0
        self.mu_t_1 = np.array([[self.x_prev], [self.y_prev], [self.theta_prev]])
        self.MuBar = np.zeros((3, 1))
        self.Mu = self.mu_t_1.copy()  # state vector initially holds only robot pose

        # Motion model noise parameters
        self.alpha1 = 0.005
        self.alpha2 = 0.05
        self.alpha3 = 0.005
        self.alpha4 = 0.05

        # Measurement model noise parameters
        self.sigma_r = 0.1    # range measurement noise
        self.sigma_phi = 0.01  # bearing measurement noise
        self.Qt = np.array([[self.sigma_r**2, 0],
                            [0, self.sigma_phi**2]])  # measurement noise covariance

        self.first_itn = True
        self.Sigma = None   # covariance matrix; will be initialized later
        self.SigmaBar = None

        # For landmark tracking (from laser scan)
        self.global_coord = []  # global coordinates computed from scan
        self.lidar_dist = []
        self.lidar_angle = []

        # Odometry values
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.Dt = 0.0
        self.Dr1 = 0.0
        self.Dr2 = 0.0

    def odom_callback(self, msg):
        # Extract robot pose from odometry.
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        thetaa = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([thetaa.x, thetaa.y, thetaa.z, thetaa.w])
        self.theta = yaw

        # Compute odometry-based motion.
        self.Dt = m.sqrt((self.x - self.x_prev)**2 + (self.y - self.y_prev)**2)
        self.Dr1 = scale(m.atan2(self.y - self.y_prev, self.x - self.x_prev) - self.theta_prev)
        self.Dr2 = scale(self.theta - self.theta_prev - self.Dr1)

    def scan_callback(self, scan):
        # Process laser scan to extract landmark candidates.
        self.x_scan = [0.0] * len(scan.ranges)
        self.diff = [0.0] * len(scan.ranges)

        for i in range(len(scan.ranges)):
            if m.isinf(scan.ranges[i]) or m.isnan(scan.ranges[i]):
                self.x_scan[i] = scan.range_max
            else:
                self.x_scan[i] = scan.ranges[i]
            if i > 0 and self.x_scan[i] > 0.2 and self.x_scan[i-1] > 0.2:
                diff = self.x_scan[i] - self.x_scan[i-1]
            else:
                diff = 0.0
            self.diff[i] = diff if abs(diff) > 0.15 else 0.0

        # Reset arrays for landmark detection.
        self.lidar_dist = []
        self.lidar_angle = []
        indices, angle, ranges, j, landmarks_det = 0, 0, 0, -1, 0
        while j < len(self.diff) - 1:
            j += 1
            if self.diff[j] < 0 and landmarks_det == 0:
                landmarks_det = 1
                indices += 1
                angle += j
                ranges += self.x_scan[j]
            elif self.diff[j] < 0 and landmarks_det == 1:
                landmarks_det = 0
                indices = 0
                angle = 0
                ranges = 0
                j -= 1
            elif self.diff[j] == 0 and landmarks_det == 1:
                indices += 1
                angle += j
                ranges += self.x_scan[j]
            elif self.diff[j] > 0 and landmarks_det == 1:
                landmarks_det = 0
                if indices != 0:
                    avg_angle = (angle * m.pi / 180) / indices
                    self.lidar_angle.append(scale(avg_angle))
                    self.lidar_dist.append(ranges / indices)
                indices = 0
                angle = 0
                ranges = 0
            else:
                continue

    def global_coord_conv(self):
        # Convert detected landmarks (from laser) into global coordinates using the previous pose.
        self.global_coord = []
        for i in range(len(self.lidar_dist)):
            x = self.x_prev + self.lidar_dist[i] * np.cos(scale(self.theta_prev + self.lidar_angle[i]))
            y = self.y_prev + self.lidar_dist[i] * np.sin(scale(self.theta_prev + self.lidar_angle[i]))
            self.global_coord.append([x, y])

    def update_landmarks(self):
        """
        Check each detected landmark in global_coord. If a landmark is not yet in the state vector,
        augment the state (Mu) and update the covariance (Sigma) if already initialized.
        """
        n_stored = (self.Mu.shape[0] - 3) // 2  # landmarks already in state
        for landmark in self.global_coord:
            if not self.is_landmark_known(landmark, n_stored):
                new_landmark = np.array([[landmark[0]], [landmark[1]]])
                self.Mu = np.vstack((self.Mu, new_landmark))
                self.get_logger().info("New landmark added: " + str(landmark))
                if not self.first_itn:
                    old_size = self.Sigma.shape[0]
                    new_size = old_size + 2
                    Sigma_new = np.eye(new_size) * 1e5  # high uncertainty for new landmark
                    Sigma_new[:old_size, :old_size] = self.Sigma
                    self.Sigma = Sigma_new
                n_stored += 1

    def is_landmark_known(self, landmark, n_stored):
        """Return True if the landmark [x,y] is already in the state (within a threshold)."""
        threshold = 0.5  # adjust as needed
        for i in range(n_stored):
            stored_x = self.Mu[3 + 2*i, 0]
            stored_y = self.Mu[4 + 2*i, 0]
            if m.sqrt((landmark[0] - stored_x)**2 + (landmark[1] - stored_y)**2) < threshold:
                return True
        return False

    def state_matrix(self):
        # Update the robot state using odometry.
        dx = self.x_prev + self.Dt * np.cos(self.theta_prev + self.Dr1)
        dy = self.y_prev + self.Dt * np.sin(self.theta_prev + self.Dr1)
        dtheta = scale(self.theta_prev + self.Dr1 + self.Dr2)
        motion_update = np.array([[dx - self.x_prev],
                                  [dy - self.y_prev],
                                  [scale(dtheta - self.theta_prev)]])
        if self.first_itn:
            if len(self.global_coord) > 0:
                landmark_states = np.zeros((2 * len(self.global_coord), 1))
                for i in range(len(self.global_coord)):
                    landmark_states[2*i, 0] = self.global_coord[i][0]
                    landmark_states[2*i+1, 0] = self.global_coord[i][1]
                self.Mu = np.vstack((self.mu_t_1, landmark_states))
                self.MuBar = np.vstack((self.mu_t_1 + motion_update, landmark_states))
            else:
                self.MuBar = self.mu_t_1 + motion_update
                self.Mu = self.MuBar.copy()
        else:
            # Update robot state only and keep stored landmarks.
            robot_state = self.Mu[0:3] + motion_update
            landmark_states = self.Mu[3:]
            self.MuBar = np.vstack((robot_state, landmark_states))
        self.MuBar[2, 0] = scale(self.MuBar[2, 0])

    def state_covariance(self):
        # Build the motion model Jacobian for the robot.
        self.lowGt = np.array([
            [1, 0, -self.Dt * m.sin(self.theta_prev + self.Dr1)],
            [0, 1, self.Dt * m.cos(self.theta_prev + self.Dr1)],
            [0, 0, 1]
        ])
        n_landmarks = (self.Mu.shape[0] - 3) // 2
        top_right = np.zeros((3, 2 * n_landmarks))
        bottom_left = np.zeros((2 * n_landmarks, 3))
        bottom_right = np.eye(2 * n_landmarks)
        self.Gt = np.block([
            [self.lowGt, top_right],
            [bottom_left, bottom_right]
        ])

    def control_covariance(self):
        # Build the control Jacobian (v) and append zeros for landmarks.
        v = np.array([
            [-self.Dt * m.sin(self.theta_prev + self.Dr1), m.cos(self.theta_prev + self.Dr1), 0],
            [self.Dt * m.cos(self.theta_prev + self.Dr1), m.sin(self.theta_prev + self.Dr1), 0],
            [1, 0, 1]
        ])
        n_landmarks = (self.Mu.shape[0] - 3) // 2
        lm_control = np.zeros((2 * n_landmarks, 3))
        self.Vt = np.block([[v], [lm_control]])

    def process_noise(self):
        # Compute process noise covariance.
        min_noise = 1e-6
        self.Mt = np.array([
            [max((self.alpha1 * self.Dr1**2) + (self.alpha2 * self.Dt**2), min_noise), 0, 0],
            [0, max((self.alpha3 * self.Dt**2) + (self.alpha4 * (self.Dr2**2 + self.Dr1**2)), min_noise), 0],
            [0, 0, max((self.alpha1 * self.Dr1**2) + (self.alpha2 * self.Dt**2), min_noise)]
        ])
        self.Rt = self.Vt @ self.Mt @ self.Vt.T

    def covariance(self):
        if self.first_itn:
            n_landmarks = (self.Mu.shape[0] - 3) // 2
            sigmaTl = np.eye(3) * 0.01
            sigmaTr = np.zeros((3, 2 * n_landmarks))
            sigmaBl = np.zeros((2 * n_landmarks, 3))
            sigmaBr = np.eye(2 * n_landmarks) * (10**5)
            self.Sigma = np.block([
                [sigmaTl, sigmaTr],
                [sigmaBl, sigmaBr]
            ])
            self.first_itn = False
        else:
            expected_dim = self.Mu.shape[0]
            if self.Sigma.shape[0] != expected_dim:
                old_size = self.Sigma.shape[0]
                new_size = expected_dim
                Sigma_new = np.eye(new_size) * 1e5
                Sigma_new[:old_size, :old_size] = self.Sigma
                self.Sigma = Sigma_new
        self.SigmaBar = self.Gt @ self.Sigma @ self.Gt.T + self.Rt
        self.Sigma = self.SigmaBar

    def correction(self):
        # If no landmarks are detected in the current scan, prune any stored landmarks.
        if len(self.global_coord) == 0:
            self.get_logger().info("No landmarks detected, reverting to robot-only state.")
            # Remove landmarks: take only the robot state (first 3 rows)
            self.Mu = self.MuBar[0:3]
            if self.Sigma is not None and self.Sigma.shape[0] > 3:
                self.Sigma = self.SigmaBar[0:3, 0:3]
            self.x_prev = self.x
            self.y_prev = self.y
            self.theta_prev = self.theta
            self.mu_t_1 = self.Mu[0:3]
            return

        current_mu = self.MuBar.copy()
        current_sigma = self.SigmaBar.copy()
        n_landmarks = (current_mu.shape[0] - 3) // 2
        n_meas = len(self.lidar_dist)
        self.get_logger().info("n_landmarks: " + str(n_landmarks) + ", n_meas: " + str(n_meas))

        for i in range(min(n_landmarks, n_meas)):
            x_robot = current_mu[0, 0]
            y_robot = current_mu[1, 0]
            theta_robot = current_mu[2, 0]

            x_landmark = current_mu[3+2*i, 0]
            y_landmark = current_mu[4+2*i, 0]

            dx = x_landmark - x_robot
            dy = y_landmark - y_robot
            q = dx**2 + dy**2
            if q == 0:
                continue  # Avoid division by zero

            hx = np.array([[m.sqrt(q)],
                           [scale(m.atan2(dy, dx) - theta_robot)]])
            z = np.array([[self.lidar_dist[i]],
                          [self.lidar_angle[i]]])
            lowHt = np.array([
                [-m.sqrt(q)*dx, -m.sqrt(q)*dy, 0, m.sqrt(q)*dx, m.sqrt(q)*dy],
                [dy, -dx, -q, -dy, dx]
            ])/q
            Ht = np.zeros((2, 3+2*n_landmarks))
            Ht[0:2, 0:3] = lowHt[:, 0:3]
            Ht[0:2, 3+2*i:5+2*i] = lowHt[:, 3:5]
            self.get_logger().info(f"Ht for landmark {i}: {Ht}")
            S = Ht @ current_sigma @ Ht.T + self.Qt
            K = current_sigma @ Ht.T @ np.linalg.inv(S)
            current_mu = current_mu + K @ (z - hx)
            current_sigma = (np.eye(current_mu.shape[0]) - K @ Ht) @ current_sigma
            current_mu[2, 0] = scale(current_mu[2, 0])
        self.Mu = current_mu
        self.Sigma = current_sigma
        self.x_prev = self.x
        self.y_prev = self.y
        self.theta_prev = self.theta
        self.mu_t_1 = self.Mu[0:3]

    def publish_pose(self):
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "map"
        pose_msg.pose.pose.position.x = self.Mu[0, 0]
        pose_msg.pose.pose.position.y = self.Mu[1, 0]
        pose_msg.pose.pose.position.z = 0.0
        q = quaternion_from_euler(0, 0, self.Mu[2, 0])
        pose_msg.pose.pose.orientation.x = q[0]
        pose_msg.pose.pose.orientation.y = q[1]
        pose_msg.pose.pose.orientation.z = q[2]
        pose_msg.pose.pose.orientation.w = q[3]
        if self.Sigma is not None and self.Sigma.shape[0] >= 3:
            pose_cov = np.zeros((6, 6))
            pose_cov[0:2, 0:2] = self.Sigma[0:2, 0:2]
            pose_cov[5, 5] = self.Sigma[2, 2]
            pose_msg.pose.covariance = pose_cov.flatten().tolist()
        self.pub.publish(pose_msg)

    def timer_cb(self):
        self.global_coord_conv()
        self.update_landmarks()    # Update state with any new landmarks.
        self.state_matrix()
        self.state_covariance()
        self.control_covariance()
        self.process_noise()
        self.covariance()
        self.correction()
        self.publish_pose()

def main(args=None):
    rclpy.init(args=args)
    ekf_slam = EKF_SLAM()
    rclpy.spin(ekf_slam)
    rclpy.shutdown()

if __name__ == '__main__':
    main()