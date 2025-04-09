import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField
from cv_bridge import CvBridge
import numpy as np
import random
import struct
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point

class GroundPlaneDetector(Node):
    def __init__(self):
        super().__init__('ground_plane_detector')
        self.subscription = self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self.image_callback,
            10)
        self.bridge = CvBridge()
        
        # Intrinsic parameters (adjust if necessary)
        self.fx = 525.0
        self.fy = 525.0
        self.cx = 320.0
        self.cy = 240.0

        # Camera mounting parameters:
        self.camera_height = 0.45  # in meters
        self.camera_tilt_deg = 15  # in degrees
        self.camera_tilt_rad = np.deg2rad(self.camera_tilt_deg)
        
        # Tolerance for how close a point's y should be to the expected ground y.
        self.ground_tolerance = 0.15  # in meters

        # Adjusted RANSAC distance threshold.
        self.ransac_distance_threshold = 0.02

        # Store the latest depth image
        self.latest_depth_image = None

        # Publisher for markers
        self.marker_pub = self.create_publisher(Marker, 'ground_markers', 10)
        self.marker_array_pub = self.create_publisher(MarkerArray, 'ground_marker_array', 10)
        
        # Publishers for PointCloud2 messages
        self.ground_cloud_pub = self.create_publisher(PointCloud2, 'ground_pointcloud', 10)
        self.object_cloud_pub = self.create_publisher(PointCloud2, 'object_pointcloud', 10)
        self.combined_cloud_pub = self.create_publisher(PointCloud2, 'colored_pointcloud', 10)

        # Create a timer callback to process the image every 0.5 seconds
        self.timer = self.create_timer(0.5, self.timer_callback)

    def image_callback(self, msg):
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
            self.latest_depth_image = depth_image
            self.get_logger().info(f"Stored new depth image with shape: {depth_image.shape}")
        except Exception as e:
            self.get_logger().error("Could not convert depth image: " + str(e))

    def timer_callback(self):
        if self.latest_depth_image is None:
            self.get_logger().warn("No depth image available yet.")
            return
        
        depth_image = self.latest_depth_image
        self.get_logger().info(f"Processing depth image with shape: {depth_image.shape}")

        # Generate the point cloud.
        points = self.create_point_cloud(depth_image)
        if points.size == 0:
            self.get_logger().warn("No valid points in depth image.")
            return

        # Remove NaN values.
        valid_indices = ~np.isnan(points).any(axis=1)
        points = points[valid_indices]
        
        if points.shape[0] == 0:
            self.get_logger().warn("No valid points after NaN filtering.")
            return
        
        # Filter points based on vertical alignment with expected ground.
        expected_y = (np.sin(self.camera_tilt_rad) * points[:, 2] - self.camera_height) / np.cos(self.camera_tilt_rad)
        ground_diff = np.abs(points[:, 1] - expected_y)
        points_filtered = points[ground_diff < self.ground_tolerance]
        self.get_logger().info(f"Points before filtering: {points.shape[0]}, after ground filter: {points_filtered.shape[0]}")

        if points_filtered.shape[0] < 3:
            self.get_logger().warn("Not enough points after ground filtering.")
            return

        # Run RANSAC to fit a plane on the filtered points.
        plane, inlier_indices = self.ransac_plane(points_filtered, iterations=100, distance_threshold=self.ransac_distance_threshold)
        if plane is None:
            self.get_logger().info("No plane found in the current frame.")
            return

        # Create a boolean mask for ground points.
        ground_mask = np.zeros(points_filtered.shape[0], dtype=bool)
        ground_mask[inlier_indices] = True
        
        # Split points into ground and non-ground.
        ground_points = points_filtered[ground_mask]
        non_ground_points = points_filtered[~ground_mask]
        
        self.get_logger().info(f"Ground points: {ground_points.shape[0]}, Other objects: {non_ground_points.shape[0]}")

        # Publish visualizations (try all methods for better chances of success)
        self.publish_markers(ground_points, non_ground_points)
        self.publish_marker_array(ground_points, non_ground_points)
        self.publish_separate_point_clouds(ground_points, non_ground_points)
        self.publish_combined_point_cloud(ground_points, non_ground_points)

    def create_point_cloud(self, depth_image):
        height, width = depth_image.shape
        v, u = np.indices((height, width))
        z = depth_image.astype(np.float32)
        x = (u - self.cx) * z / self.fx
        y = (v - self.cy) * z / self.fy  # In the optical frame, y is vertical (downward)
        x = x.flatten()
        y = y.flatten()
        z = z.flatten()
        points = np.vstack((x, y, z)).T
        valid = z > 0.0
        return points[valid]

    def ransac_plane(self, points, iterations=100, distance_threshold=0.02):
        best_inliers = []
        best_plane = None
        num_points = points.shape[0]
        if num_points < 3:
            return None, None

        for i in range(iterations):
            idx = random.sample(range(num_points), 3)
            sample = points[idx, :]
            p1, p2, p3 = sample

            v1 = p2 - p1
            v2 = p3 - p1
            normal = np.cross(v1, v2)
            norm_val = np.linalg.norm(normal)
            if norm_val == 0:
                continue
            normal = normal / norm_val
            d = -np.dot(normal, p1)

            distances = np.abs(np.dot(points, normal) + d)
            inlier_indices = np.where(distances < distance_threshold)[0]

            if len(inlier_indices) > len(best_inliers):
                best_inliers = inlier_indices
                best_plane = (normal, d)

        return best_plane, best_inliers

    def publish_markers(self, ground_points, non_ground_points):
        """Publish separate markers for ground and non-ground points"""
        # Sample points if there are too many
        max_points = 1000
        if len(ground_points) > max_points:
            indices = np.random.choice(len(ground_points), max_points, replace=False)
            ground_points_sample = ground_points[indices]
        else:
            ground_points_sample = ground_points
            
        if len(non_ground_points) > max_points:
            indices = np.random.choice(len(non_ground_points), max_points, replace=False)
            non_ground_points_sample = non_ground_points[indices]
        else:
            non_ground_points_sample = non_ground_points
        
        # Ground points marker (green)
        ground_marker = Marker()
        ground_marker.header = Header()
        ground_marker.header.stamp = self.get_clock().now().to_msg()
        ground_marker.header.frame_id = "camera_depth_optical_frame"  # Adjust to match your frame
        ground_marker.ns = "ground_points"
        ground_marker.id = 0
        ground_marker.type = Marker.POINTS
        ground_marker.action = Marker.ADD
        ground_marker.pose.orientation.w = 1.0
        ground_marker.scale.x = 0.03  # Increased point size for visibility
        ground_marker.scale.y = 0.03
        ground_marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)  # Green
        
        for pt in ground_points_sample:
            point = Point()
            point.x = float(pt[0])
            point.y = float(pt[1])
            point.z = float(pt[2])
            ground_marker.points.append(point)
        
        # Non-ground points marker (red)
        object_marker = Marker()
        object_marker.header = Header()
        object_marker.header.stamp = self.get_clock().now().to_msg()
        object_marker.header.frame_id = "camera_depth_optical_frame"  # Adjust to match your frame
        object_marker.ns = "object_points"
        object_marker.id = 1
        object_marker.type = Marker.POINTS
        object_marker.action = Marker.ADD
        object_marker.pose.orientation.w = 1.0
        object_marker.scale.x = 0.03  # Increased point size for visibility
        object_marker.scale.y = 0.03
        object_marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)  # Red
        
        for pt in non_ground_points_sample:
            point = Point()
            point.x = float(pt[0])
            point.y = float(pt[1])
            point.z = float(pt[2])
            object_marker.points.append(point)
        
        # Publish both markers
        self.marker_pub.publish(ground_marker)
        self.marker_pub.publish(object_marker)
        self.get_logger().info(f"Published ground marker with {len(ground_marker.points)} points")
        self.get_logger().info(f"Published object marker with {len(object_marker.points)} points")

    def publish_marker_array(self, ground_points, non_ground_points):
        """Publish ground and non-ground points as a MarkerArray"""
        # Sample points if there are too many
        max_points = 1000
        if len(ground_points) > max_points:
            indices = np.random.choice(len(ground_points), max_points, replace=False)
            ground_points_sample = ground_points[indices]
        else:
            ground_points_sample = ground_points
            
        if len(non_ground_points) > max_points:
            indices = np.random.choice(len(non_ground_points), max_points, replace=False)
            non_ground_points_sample = non_ground_points[indices]
        else:
            non_ground_points_sample = non_ground_points
        
        # Ground points marker (green)
        ground_marker = Marker()
        ground_marker.header = Header()
        ground_marker.header.stamp = self.get_clock().now().to_msg()
        ground_marker.header.frame_id = "camera_depth_optical_frame"  # Adjust to match your frame
        ground_marker.ns = "ground_points"
        ground_marker.id = 0
        ground_marker.type = Marker.POINTS
        ground_marker.action = Marker.ADD
        ground_marker.pose.orientation.w = 1.0
        ground_marker.scale.x = 0.03  # Increased point size for visibility
        ground_marker.scale.y = 0.03
        ground_marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)  # Green
        
        for pt in ground_points_sample:
            point = Point()
            point.x = float(pt[0])
            point.y = float(pt[1])
            point.z = float(pt[2])
            ground_marker.points.append(point)
        
        # Non-ground points marker (red)
        object_marker = Marker()
        object_marker.header = Header()
        object_marker.header.stamp = self.get_clock().now().to_msg()
        object_marker.header.frame_id = "camera_depth_optical_frame"  # Adjust to match your frame
        object_marker.ns = "object_points"
        object_marker.id = 1
        object_marker.type = Marker.POINTS
        object_marker.action = Marker.ADD
        object_marker.pose.orientation.w = 1.0
        object_marker.scale.x = 0.03  # Increased point size for visibility
        object_marker.scale.y = 0.03
        object_marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)  # Red
        
        for pt in non_ground_points_sample:
            point = Point()
            point.x = float(pt[0])
            point.y = float(pt[1])
            point.z = float(pt[2])
            object_marker.points.append(point)
        
        # Create marker array and publish
        marker_array = MarkerArray()
        marker_array.markers = [ground_marker, object_marker]
        self.marker_array_pub.publish(marker_array)
        self.get_logger().info(f"Published marker array with {len(ground_marker.points)} ground points and {len(object_marker.points)} object points")

    def publish_separate_point_clouds(self, ground_points, non_ground_points):
        """Publish ground and non-ground points as separate PointCloud2 messages"""
        # Sample points if there are too many
        max_points = 5000
        if len(ground_points) > max_points:
            indices = np.random.choice(len(ground_points), max_points, replace=False)
            ground_points_sample = ground_points[indices]
        else:
            ground_points_sample = ground_points
            
        if len(non_ground_points) > max_points:
            indices = np.random.choice(len(non_ground_points), max_points, replace=False)
            non_ground_points_sample = non_ground_points[indices]
        else:
            non_ground_points_sample = non_ground_points
        
        # Create and publish ground point cloud (if there are points)
        if len(ground_points_sample) > 0:
            # Create point cloud fields
            fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            ]
            
            # Prepare cloud data as a numpy array
            cloud_array = ground_points_sample.astype(np.float32)
            
            # Create the PointCloud2 message
            cloud_msg = PointCloud2()
            cloud_msg.header = Header()
            cloud_msg.header.stamp = self.get_clock().now().to_msg()
            cloud_msg.header.frame_id = "camera_depth_optical_frame"  # Adjust to match your frame
            cloud_msg.height = 1
            cloud_msg.width = len(ground_points_sample)
            cloud_msg.fields = fields
            cloud_msg.is_bigendian = False
            cloud_msg.point_step = 12  # 4 bytes per float * 3 fields
            cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width
            cloud_msg.is_dense = True
            
            # Convert numpy array to bytes
            cloud_msg.data = cloud_array.tobytes()
            
            # Publish the cloud
            self.ground_cloud_pub.publish(cloud_msg)
            self.get_logger().info(f"Published ground point cloud with {len(ground_points_sample)} points")
        
        # Create and publish object point cloud (if there are points)
        if len(non_ground_points_sample) > 0:
            # Create point cloud fields
            fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            ]
            
            # Prepare cloud data as a numpy array
            cloud_array = non_ground_points_sample.astype(np.float32)
            
            # Create the PointCloud2 message
            cloud_msg = PointCloud2()
            cloud_msg.header = Header()
            cloud_msg.header.stamp = self.get_clock().now().to_msg()
            cloud_msg.header.frame_id = "camera_depth_optical_frame"  # Adjust to match your frame
            cloud_msg.height = 1
            cloud_msg.width = len(non_ground_points_sample)
            cloud_msg.fields = fields
            cloud_msg.is_bigendian = False
            cloud_msg.point_step = 12  # 4 bytes per float * 3 fields
            cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width
            cloud_msg.is_dense = True
            
            # Convert numpy array to bytes
            cloud_msg.data = cloud_array.tobytes()
            
            # Publish the cloud
            self.object_cloud_pub.publish(cloud_msg)
            self.get_logger().info(f"Published object point cloud with {len(non_ground_points_sample)} points")

    def publish_combined_point_cloud(self, ground_points, non_ground_points):
    # Sample points if there are too many
        max_points = 5000
        if len(ground_points) > max_points:
            indices = np.random.choice(len(ground_points), max_points, replace=False)
            ground_points_sample = ground_points[indices]
        else:
            ground_points_sample = ground_points

        if len(non_ground_points) > max_points:
            indices = np.random.choice(len(non_ground_points), max_points, replace=False)
            non_ground_points_sample = non_ground_points[indices]
        else:
            non_ground_points_sample = non_ground_points

        # Combine points
        all_points = np.vstack((ground_points_sample, non_ground_points_sample))
        
        # Define colors:
        # Here, for example, we set ground points to green and non-ground to red.
        green_uint32 = 0xFF00FF00  # ARGB: A=FF, R=00, G=FF, B=00
        red_uint32   = 0xFFFF0000  # ARGB: A=FF, R=FF, G=00, B=00
        ground_color = np.full((len(ground_points_sample), 1), green_uint32, dtype=np.uint32)
        object_color = np.full((len(non_ground_points_sample), 1), red_uint32, dtype=np.uint32)
        colors_uint32 = np.vstack((ground_color, object_color))
        
        # Convert each uint32 color into a float32 with the same bit pattern.
        colors_float = np.array(
            [struct.unpack('f', struct.pack('I', int(c)))[0] for c in colors_uint32.flatten()],
            dtype=np.float32
        ).reshape(-1, 1)
        
        # Create combined cloud data with x, y, z, rgb
        cloud_data = np.hstack((all_points, colors_float)).astype(np.float32)
        
        # Define the fields for the PointCloud2
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "camera_depth_optical_frame"  # Ensure this frame exists in your TF tree

        cloud_msg = PointCloud2()
        cloud_msg.header = header
        cloud_msg.height = 1
        cloud_msg.width = cloud_data.shape[0]
        cloud_msg.fields = fields
        cloud_msg.is_bigendian = False
        cloud_msg.point_step = 16  # 4 fields * 4 bytes each
        cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width
        cloud_msg.is_dense = True
        cloud_msg.data = cloud_data.tobytes()
        
        self.combined_cloud_pub.publish(cloud_msg)
        self.get_logger().info(f"Published combined point cloud with {cloud_data.shape[0]} points")

def main(args=None):
    rclpy.init(args=args)
    node = GroundPlaneDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("GroundPlaneDetector node stopped cleanly.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()