#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <limits>

class PointCloudFilter : public rclcpp::Node
{
public:
  PointCloudFilter()
  : Node("pointcloud_filter")
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/camera/depth/points", 10, std::bind(&PointCloudFilter::filter_callback, this, std::placeholders::_1));
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/filtered/pointcloud", 10);
  }

private:
  void filter_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // Mostrar por la pantalla los puntos 
    RCLCPP_INFO(this->get_logger(), "Received point cloud with %zu points", cloud->points.size());
    
    pcl::fromROSMsg(*msg, *cloud);

    if (cloud->points.empty())
    {
      RCLCPP_WARN(this->get_logger(), "Input cloud is empty.");
      return;
    }

    // Encontrar la z más pequeña
    float min_z = std::numeric_limits<float>::max();
    for (const auto& point : cloud->points)
    {
      if (point.z < min_z)
      {
        min_z = point.z;
      }
    }

    // Filtrar los puntos que están dentro del rango de +-0.1 de min_z
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto& point : cloud->points)
    {
      if (point.z >= min_z - 0.1 && point.z <= min_z + 0.1)
      // Mostrar por pantalla el punto
      {
        RCLCPP_INFO(this->get_logger(), "Point: (%f, %f, %f)", point.x, point.y, point.z);
        cloud_filtered->points.push_back(point);
      }
    }

    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(*cloud_filtered, output);
    output.header = msg->header;

    publisher_->publish(output);
    // Resetear el punto
    cloud->points.clear(); 
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudFilter>());
  rclcpp::shutdown();
  return 0;
}
