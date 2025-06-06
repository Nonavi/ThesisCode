#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

class AccumulatingCloudNode : public rclcpp::Node
{
public:
  AccumulatingCloudNode()
  : Node("accumulating_cloud_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/camera/camera/depth/color/points", 10,
      std::bind(&AccumulatingCloudNode::pointCloudCallback, this, std::placeholders::_1));

    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/accumulated_cloud", 10);

    accumulated_cloud_ = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());

    RCLCPP_INFO(this->get_logger(), "Nodo acumulador iniciado.");
  }

private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    sensor_msgs::msg::PointCloud2 transformed_cloud;

    // Intentar transformar la nube al frame "world"
    try
    {
      geometry_msgs::msg::TransformStamped transform =
        tf_buffer_.lookupTransform("world", msg->header.frame_id, tf2::TimePointZero);

      tf2::doTransform(*msg, transformed_cloud, transform);
    }
    catch (const tf2::TransformException &ex)
    {
      RCLCPP_WARN(this->get_logger(), "Transformación fallida: %s", ex.what());
      return;
    }

    // Convertir a formato PCL
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::fromROSMsg(transformed_cloud, *pcl_cloud);

    // ------------------------------------
    // Aplicar recorte espacial en x, y, z
    // ------------------------------------
    pcl::PassThrough<pcl::PointXYZRGB> pass;

    // Filtro Z
    pass.setInputCloud(pcl_cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.07, 0.5);
    pass.filter(*pcl_cloud);

    // Filtro X
    pass.setInputCloud(pcl_cloud);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(-0.45, 0.45);
    pass.filter(*pcl_cloud);

    // Filtro Y
    pass.setInputCloud(pcl_cloud);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-0.277, 0.277);
    pass.filter(*pcl_cloud);

    if (pcl_cloud->empty())
    {
      RCLCPP_WARN(this->get_logger(), "La nube está vacía luego del recorte.");
      return;
    }

    // ------------------------------------
    // Acumular la nube transformada y recortada
    // ------------------------------------
    *accumulated_cloud_ += *pcl_cloud;

    // ------------------------------------
    // Aplicar filtro voxel (1 cm resolución)
    // ------------------------------------
    pcl::VoxelGrid<pcl::PointXYZRGB> voxel_filter;
    voxel_filter.setInputCloud(accumulated_cloud_);
    voxel_filter.setLeafSize(0.01f, 0.01f, 0.01f);
    voxel_filter.filter(*accumulated_cloud_);

    // Publicar la nube acumulada
    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(*accumulated_cloud_, output);
    output.header.frame_id = "world";
    output.header.stamp = this->now();
    publisher_->publish(output);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr accumulated_cloud_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AccumulatingCloudNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
