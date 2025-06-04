#include <rclcpp/rclcpp.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <octomap/OcTree.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

using std::placeholders::_1;

class OctomapVoxelsWithSize : public rclcpp::Node {
public:
  OctomapVoxelsWithSize()
  : Node("octomap_voxels_with_size")
  {
    sub_ = this->create_subscription<octomap_msgs::msg::Octomap>(
      "/octomap_binary",  10,
      std::bind(&OctomapVoxelsWithSize::onOctomap, this, _1));
    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/octomap_voxels_info", 1);
  }

private:
  void onOctomap(const octomap_msgs::msg::Octomap::SharedPtr msg) {
    auto abs_tree = octomap_msgs::binaryMsgToMap(*msg);
    auto tree = std::unique_ptr<octomap::OcTree>(
      dynamic_cast<octomap::OcTree*>(abs_tree));
    if (!tree) {
      RCLCPP_WARN(get_logger(), "No se pudo reconstruir OcTree");
      return;
    }

    // Contar hojas ocupadas
    size_t count = 0;
    for (auto it = tree->begin_leafs(), end = tree->end_leafs(); it != end; ++it) {
      if (tree->isNodeOccupied(*it)) ++count;
    }

    // Preparar PointCloud2 con campos x,y,z,intensity
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header = msg->header;
    cloud.height = 1;
    cloud.width  = count;
    cloud.is_bigendian = false;
    cloud.is_dense = false;  // o true si sabes que no hay NaNs

    sensor_msgs::PointCloud2Modifier mod(cloud);
    // Aquí definimos 4 campos: X,Y,Z e INTENSITY (que usaremos para el tamaño)
    mod.setPointCloud2Fields(
      /*n_fields=*/4,
      "x",         sensor_msgs::msg::PointField::FLOAT32, 1,
      "y",         sensor_msgs::msg::PointField::FLOAT32, 1,
      "z",         sensor_msgs::msg::PointField::FLOAT32, 1,
      "intensity", sensor_msgs::msg::PointField::FLOAT32, 1
    );
    mod.resize(count);

    // Rellenar datos
    sensor_msgs::PointCloud2Iterator<float> it_x(cloud, "x"),
                                        it_y(cloud, "y"),
                                        it_z(cloud, "z"),
                                        it_i(cloud, "intensity");
    for (auto leaf = tree->begin_leafs(), end = tree->end_leafs(); leaf != end; ++leaf) {
      if (!tree->isNodeOccupied(*leaf)) {
        continue;
      }
      *it_x = leaf.getX();
      *it_y = leaf.getY();
      *it_z = leaf.getZ();
      *it_i = static_cast<float>(leaf.getSize());
      ++it_x; ++it_y; ++it_z; ++it_i;
    }

    pub_->publish(cloud);
    RCLCPP_INFO(get_logger(), "Publicado %zu voxels con size en intensity", count);
  }

  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OctomapVoxelsWithSize>());
  rclcpp::shutdown();
  return 0;
}
