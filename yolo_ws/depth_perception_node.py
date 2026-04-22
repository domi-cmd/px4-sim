import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from cv_bridge import CvBridge
import message_filters
import open3d as o3d
import numpy as np
import cv2

class DepthPerceptionNode(Node):
    def __init__(self):
        super().__init__('depth_perception_node')
        self.bridge = CvBridge()
        self.pcd_publisher = self.create_publisher(PointCloud2, '/filtered_cloud', 10)
        
        # Camera Intrinsics (Initialized via CameraInfo)
        self.intrinsic = None

        # Subscribers
        self.info_sub = self.create_subscription(CameraInfo, 
            '/world/baylands/model/x500_depth_0/link/camera_link/sensor/IMX214/camera_info', 
            self.info_callback, 10)
            
        self.rgb_sub = message_filters.Subscriber(self, Image, 
            '/world/baylands/model/x500_depth_0/link/camera_link/sensor/IMX214/image')
            
        self.depth_sub = message_filters.Subscriber(self, Image, 
            '/world/baylands/model/x500_depth_0/link/camera_link/sensor/StereoOV7251/depth_image')

        # Synchronize RGB and Depth (10-frame queue, 0.1s max slop)
        self.ts = message_filters.ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub], 10, 0.1)
        self.ts.registerCallback(self.sync_callback)

    def info_callback(self, msg):
        # Extract intrinsic parameters from CameraInfo
        self.intrinsic = o3d.camera.PinholeCameraIntrinsic(
            msg.width, msg.height, msg.k[0], msg.k[4], msg.k[2], msg.k[5]
        )

    def sync_callback(self, rgb_msg, depth_msg):
        if self.intrinsic is None:
            return

        # 1. Convert ROS images to Open3D format
        color_data = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
        depth_data = self.bridge.imgmsg_to_cv2(depth_msg, "32FC1")

        # Convert meters to millimeters (Open3D standard)
        depth_mm = (depth_data * 1000).astype(np.uint16)
        
        color_o3d = o3d.geometry.Image(cv2.cvtColor(color_data, cv2.COLOR_BGR2RGB))
        depth_o3d = o3d.geometry.Image(depth_mm)

        # 2. Create RGBD Image and PointCloud
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
            color_o3d, depth_o3d, depth_scale=1000.0, depth_trunc=10.0, convert_rgb_to_intensity=False
        )
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, self.intrinsic)

        # 3. RANSAC Ground Segmentation
        if len(pcd.points) > 100:
            plane_model, inliers = pcd.segment_plane(distance_threshold=0.2, ransac_n=3, num_iterations=1000)
            
            # Filter: Keep only the outliers (everything that ISN'T the ground)
            obstacle_pcd = pcd.select_by_index(inliers, invert=True)
            
            # 4. Convert back to ROS PointCloud2
            self.publish_pc2(obstacle_pcd, rgb_msg.header)

    def publish_pc2(self, pcd, header):
        points = np.asarray(pcd.points)
        colors = np.asarray(pcd.colors) * 255
        
        # Combine XYZ and RGB (packed into float)
        cloud_data = []
        for i in range(len(points)):
            r, g, b = colors[i].astype(int)
            rgb_packed = (r << 16) | (g << 8) | b
            cloud_data.append([points[i][0], points[i][1], points[i][2], rgb_packed])

        fields = [
            pc2.PointField(name='x', offset=0, datatype=pc2.PointField.FLOAT32, count=1),
            pc2.PointField(name='y', offset=4, datatype=pc2.PointField.FLOAT32, count=1),
            pc2.PointField(name='z', offset=8, datatype=pc2.PointField.FLOAT32, count=1),
            pc2.PointField(name='rgb', offset=12, datatype=pc2.PointField.UINT32, count=1),
        ]

        pc2_msg = pc2.create_cloud(header, fields, cloud_data)
        self.pcd_publisher.publish(pc2_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DepthPerceptionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()