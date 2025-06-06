// Mas o menos detecta las formas circulares y las esta segmentando del ruido.
// Código actualizado: detectar círculos solitarios estables con memoria por confianza

#include <rclcpp/rclcpp.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <octomap/OcTree.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>
#include <numeric>
#include <vector>
#include <cmath>
#include <algorithm>
#include <map>

using std::placeholders::_1;

struct CircleCandidate {
  float z;
  float cx, cy;
  float radius;
  int count;
  float confianza;  // valor entre 0.0 y 1.0
};

class OctomapVoxelsWithSize : public rclcpp::Node {
public:
  OctomapVoxelsWithSize()
  : Node("octomap_voxels_with_size") {
    sub_ = this->create_subscription<octomap_msgs::msg::Octomap>(
      "/octomap_binary", 10,
      std::bind(&OctomapVoxelsWithSize::onOctomap, this, _1));
    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/octomap_voxels_info", 10);
  }

private:
  std::vector<CircleCandidate> memoria_;  // persistencia

  void onOctomap(const octomap_msgs::msg::Octomap::SharedPtr msg) {
    auto abs_tree = octomap_msgs::binaryMsgToMap(*msg);
    auto octree = std::unique_ptr<octomap::OcTree>(
      dynamic_cast<octomap::OcTree*>(abs_tree));
    if (!octree) {
      RCLCPP_WARN(get_logger(), "No se pudo reconstruir OcTree");
      return;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_all(new pcl::PointCloud<pcl::PointXYZI>);
    for (auto it = octree->begin_leafs(), end = octree->end_leafs(); it != end; ++it) {
      if (!octree->isNodeOccupied(*it)) continue;
      pcl::PointXYZI pt;
      pt.x = it.getX(); pt.y = it.getY(); pt.z = it.getZ();
      pt.intensity = 0.0;
      cloud_all->points.push_back(pt);
    }
    cloud_all->width = cloud_all->points.size();
    cloud_all->height = 1;

    float min_z = std::numeric_limits<float>::max();
    float max_z = std::numeric_limits<float>::lowest();
    for (const auto& pt : cloud_all->points) {
      min_z = std::min(min_z, pt.z);
      max_z = std::max(max_z, pt.z);
    }

    float delta_z = 0.01;
    std::vector<CircleCandidate> nuevos;

    for (float z = max_z; z >= min_z; z -= delta_z) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr slice_cloud(new pcl::PointCloud<pcl::PointXYZ>);
      for (const auto& pt : cloud_all->points) {
        if (std::abs(pt.z - z) < delta_z / 2.0) {
          slice_cloud->push_back(pcl::PointXYZ(pt.x, pt.y, 0));
        }
      }
      if (slice_cloud->size() < 5) continue;

      pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
      tree->setInputCloud(slice_cloud);

      std::vector<pcl::PointIndices> cluster_indices;
      pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
      ec.setClusterTolerance(0.03);
      ec.setMinClusterSize(5);
      ec.setMaxClusterSize(1000);
      ec.setSearchMethod(tree);
      ec.setInputCloud(slice_cloud);
      ec.extract(cluster_indices);

      for (const auto& indices : cluster_indices) {
        float cx = 0.0f, cy = 0.0f;
        for (int idx : indices.indices) {
          cx += slice_cloud->points[idx].x;
          cy += slice_cloud->points[idx].y;
        }
        cx /= indices.indices.size();
        cy /= indices.indices.size();

        int count_base = 0;
        for (int idx : indices.indices) {
          const auto& p = slice_cloud->points[idx];
          if (std::hypot(p.x - cx, p.y - cy) <= 0.02) ++count_base;
        }
        if (count_base < 5) continue;

        bool estable = true;
        for (float r = 0.025; r <= 0.05; r += 0.005) {
          int count_next = 0;
          for (int idx : indices.indices) {
            const auto& p = slice_cloud->points[idx];
            if (std::hypot(p.x - cx, p.y - cy) <= r) ++count_next;
          }
          if (count_next > count_base + 2) {
            estable = false;
            break;
          }
        }
        if (!estable) continue;

        CircleCandidate cand;
        cand.z = z;
        cand.cx = cx;
        cand.cy = cy;
        cand.radius = 0.02f;
        cand.count = count_base;
        cand.confianza = 1.0f;  // nuevo círculo
        nuevos.push_back(cand);
      }
    }

    // Actualizar memoria: si un círculo se vuelve a ver, aumentar confianza; si no, disminuir
    for (auto& mem : memoria_) {
      bool encontrado = false;
      for (const auto& nuevo : nuevos) {
        float dz = std::abs(mem.z - nuevo.z);
        float dist = std::hypot(mem.cx - nuevo.cx, mem.cy - nuevo.cy);
        if (dz < 0.02 && dist < 0.03) {
          mem.confianza = std::min(1.0f, mem.confianza + 0.1f);
          encontrado = true;
          break;
        }
      }
      if (!encontrado) mem.confianza -= 0.05f;
    }
    memoria_.erase(std::remove_if(memoria_.begin(), memoria_.end(), [](const CircleCandidate& c) {
      return c.confianza <= 0.0f;
    }), memoria_.end());

    // Agregar nuevos círculos que no estaban
    for (const auto& nuevo : nuevos) {
      bool duplicado = false;
      for (const auto& mem : memoria_) {
        if (std::abs(mem.z - nuevo.z) < 0.02 && std::hypot(mem.cx - nuevo.cx, mem.cy - nuevo.cy) < 0.03) {
          duplicado = true;
          break;
        }
      }
      if (!duplicado) memoria_.push_back(nuevo);
    }

    int intensidad = 20;
    for (const auto& c : memoria_) {
      for (auto& pt : cloud_all->points) {
        float dz = std::abs(pt.z - c.z);
        if (dz > 0.05) continue;
        float dist = std::hypot(pt.x - c.cx, pt.y - c.cy);
        if (dist <= c.radius + 0.01) {
          pt.intensity = static_cast<float>(intensidad);
        }
      }
      intensidad += 20;
    }

    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(*cloud_all, output);
    output.header.frame_id = "world";
    output.header.stamp = this->get_clock()->now();
    pub_->publish(output);
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