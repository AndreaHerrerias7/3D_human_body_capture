import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import pcl
import numpy as np

class PointCloudFilter(Node):
    def __init__(self):
        super().__init__('pointcloud_filter')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/camera/depth/color/points',
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(PointCloud2, '/filtered/pointcloud', 10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        # Convert ROS PointCloud2 message to PCL PointCloud
        cloud = pcl.PointCloud.PointXYZ()
        points_list = list(pc2.read_points(msg, skip_nans=True, field_names=("x", "y", "z")))
        np_points = np.array(points_list, dtype=np.float32)
        cloud.from_array(np_points)

        # Perform plane segmentation using RANSAC
        seg = pcl.segmentation.SACSegmentationFromNormals.PointXYZ_Normal()
        seg.setOptimizeCoefficients(True)
        seg.setModelType(pcl.SACMODEL_PLANE)
        seg.setMethodType(pcl.SAC_RANSAC)
        seg.setDistanceThreshold(0.01)

        # Fit the model
        inliers = pcl.PointIndices()
        coefficients = pcl.ModelCoefficients()
        seg.setInputCloud(cloud)
        seg.segment(inliers, coefficients)

        if len(inliers.indices) == 0:
            self.get_logger().info('Could not estimate a planar model for the given dataset.')
            return

        # Extract inliers
        extract = pcl.filters.ExtractIndices.PointXYZ()
        extract.setInputCloud(cloud)
        extract.setIndices(inliers)
        extract.setNegative(False)
        cloud_filtered = pcl.PointCloud.PointXYZ()
        extract.filter(cloud_filtered)

        # Convert PCL PointCloud to ROS PointCloud2 message
        header = msg.header
        filtered_points = np.asarray(cloud_filtered.xyz, dtype=np.float32)
        filtered_msg = pc2.create_cloud_xyz32(header, filtered_points.tolist())

        # Publish the filtered cloud
        self.publisher.publish(filtered_msg)

def main(args=None):
    rclpy.init(args=args)
    pointcloud_filter = PointCloudFilter()
    rclpy.spin(pointcloud_filter)
    pointcloud_filter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
