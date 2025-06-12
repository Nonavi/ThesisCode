// Nodo ROS2: Detección de cilindros mediante agrupación de círculos y publicación para MoveIt
#include <rclcpp/rclcpp.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <octomap/OcTree.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <std_msgs/msg/bool.hpp>

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
  float confianza;
};

struct Cilindro {
  float cx, cy;
  float radio;
  float z_min, z_max;
  int total_count;
  int num_circulos;
};

class OctomapVoxelsWithSize : public rclcpp::Node {
public:
  OctomapVoxelsWithSize()
  : Node("octomap_voxels_with_size"), activar_deteccion_(false), ultimo_numero_cilindros_(0) {
    sub_ = this->create_subscription<octomap_msgs::msg::Octomap>(
      "/octomap_binary", 10,
      std::bind(&OctomapVoxelsWithSize::onOctomap, this, _1));
    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/octomap_voxels_info", 10);
    pub_objects_ = this->create_publisher<moveit_msgs::msg::CollisionObject>(
      "detected_objects", 10);

    sub_activar_ = this->create_subscription<std_msgs::msg::Bool>(
      "/activar_deteccion", 10,
      std::bind(&OctomapVoxelsWithSize::callbackActivacion, this, _1));
  }

private:
  std::vector<CircleCandidate> memoria_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::Publisher<moveit_msgs::msg::CollisionObject>::SharedPtr pub_objects_;
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_activar_;
  bool activar_deteccion_;
  int ultimo_numero_cilindros_;

  bool dentroZonaRobot(const Cilindro& c) {
    const float radio_seguridad = 0.10f;
    float distancia = std::hypot(c.cx, c.cy);
    return (distancia - c.radio < radio_seguridad);
  }

  void callbackActivacion(const std_msgs::msg::Bool::SharedPtr msg) {
    activar_deteccion_ = msg->data;
    RCLCPP_INFO(this->get_logger(), "Detección de cilindros %s",
                activar_deteccion_ ? "ACTIVADA" : "DESACTIVADA");
  }

  void onOctomap(const octomap_msgs::msg::Octomap::SharedPtr msg);
};

void OctomapVoxelsWithSize::onOctomap(const octomap_msgs::msg::Octomap::SharedPtr msg) {
  if (!activar_deteccion_) return;

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZI>);
  auto abs_tree = octomap_msgs::binaryMsgToMap(*msg);
  auto octree = std::unique_ptr<octomap::OcTree>(dynamic_cast<octomap::OcTree*>(abs_tree));
  if (!octree) return;

  for (auto it = octree->begin_leafs(), end = octree->end_leafs(); it != end; ++it) {
    if (!octree->isNodeOccupied(*it)) continue;
    pcl::PointXYZI pt;
    pt.x = it.getX(); pt.y = it.getY(); pt.z = it.getZ();
    pt.intensity = 0.0;
    cloud_temp->points.push_back(pt);
  }
  cloud_temp->width = cloud_temp->points.size();
  cloud_temp->height = 1;

  pcl::SACSegmentation<pcl::PointXYZI> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.02);
  seg.setInputCloud(cloud_temp);

  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  seg.segment(*inliers, *coefficients);
  if (coefficients->values.size() < 4) return;

  float a = coefficients->values[0];
  float b = coefficients->values[1];
  float c = coefficients->values[2];
  float d = coefficients->values[3];

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_all(new pcl::PointCloud<pcl::PointXYZI>);
  for (const auto& pt : cloud_temp->points) {
    float dist = a * pt.x + b * pt.y + c * pt.z + d;
    if (dist > 0.01) cloud_all->points.push_back(pt);
  }
  cloud_all->width = cloud_all->points.size();
  cloud_all->height = 1;

  float min_z = std::numeric_limits<float>::max(), max_z = std::numeric_limits<float>::lowest();
  for (const auto& pt : cloud_all->points) {
    min_z = std::min(min_z, pt.z);
    max_z = std::max(max_z, pt.z);
  }

  float delta_z = 0.01;
  std::vector<CircleCandidate> nuevos;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtrado(new pcl::PointCloud<pcl::PointXYZI>(*cloud_all));

  for (float z = max_z; z >= min_z; z -= delta_z) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr slice_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto& pt : cloud_filtrado->points) {
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

      std::map<float, int> growth;
      for (float r = 0.02f; r <= 0.04f; r += 0.005f) {
        int count = 0;
        for (int idx : indices.indices) {
          const auto& p = slice_cloud->points[idx];
          if (std::hypot(p.x - cx, p.y - cy) <= r) ++count;
        }
        growth[r] = count;
      }

      bool estable = false;
      float radio_aceptado = 0.02f;
      float r_prev = 0.02f;
      int count_prev = growth[r_prev];

      for (auto& [r, count] : growth) {
        if (r == r_prev) continue;
        float delta_r = r - r_prev;
        float delta_count = count - count_prev;
        float pendiente = delta_count / delta_r;
        if (pendiente < 100.0f) {
          estable = true;
          radio_aceptado = r;
          break;
        }
        if (delta_count > 30) {
          estable = false;
          break;
        }
        r_prev = r;
        count_prev = count;
      }

      if (!estable) continue;

      pcl::PointCloud<pcl::PointXYZI>::Ptr nuevo_filtrado(new pcl::PointCloud<pcl::PointXYZI>);
      for (const auto& pt : cloud_filtrado->points) {
        if (std::abs(pt.z - z) < delta_z / 2.0 && std::hypot(pt.x - cx, pt.y - cy) < radio_aceptado + 0.01f) continue;
        nuevo_filtrado->push_back(pt);
      }
      cloud_filtrado = nuevo_filtrado;

      nuevos.push_back(CircleCandidate{z, cx, cy, radio_aceptado, growth[radio_aceptado], 1.0f});
    }
  }

  std::vector<Cilindro> cilindros;
  float dz_margen = 0.02f;
  for (const auto& cand : nuevos) {
    bool unido = false;
    for (auto& cil : cilindros) {
      if (cand.z >= cil.z_min - dz_margen && cand.z <= cil.z_max + dz_margen) {
        float dist = std::hypot(cil.cx - cand.cx, cil.cy - cand.cy);
        float margen = 0.01f;
        if (dist < cil.radio + cand.radius + margen) {
          cil.cx = (cil.cx * cil.total_count + cand.cx * cand.count) / (cil.total_count + cand.count);
          cil.cy = (cil.cy * cil.total_count + cand.cy * cand.count) / (cil.total_count + cand.count);
          cil.radio = std::max(cil.radio, cand.radius);
          cil.z_min = std::min(cil.z_min, cand.z);
          cil.z_max = std::max(cil.z_max, cand.z);
          cil.total_count += cand.count;
          cil.num_circulos++;
          unido = true;
          break;
        }
      }
    }
    if (!unido) {
      cilindros.push_back(Cilindro{cand.cx, cand.cy, cand.radius, cand.z, cand.z, cand.count, 1});
    }
  }

  int intensidad = 20;
  for (const auto& c : cilindros) {
    for (auto& pt : cloud_all->points) {
      if (pt.z < c.z_min - 0.05 || pt.z > c.z_max + 0.05) continue;
      float dist = std::hypot(pt.x - c.cx, pt.y - c.cy);
      if (dist <= c.radio + 0.01f) {
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

  std::vector<Cilindro> cilindros_filtrados;
  for (const auto& cil : cilindros) {
    if (cil.num_circulos >= 4 && !dentroZonaRobot(cil)) {
      cilindros_filtrados.push_back(cil);
    }
  }

  for (size_t i = 0; i < cilindros_filtrados.size(); ++i) {
    const auto& c = cilindros_filtrados[i];

    moveit_msgs::msg::CollisionObject obj;
    obj.id = "cylinder_" + std::to_string(i);
    obj.header.frame_id = "world";

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    primitive.dimensions.resize(2);
    primitive.dimensions[0] = c.z_max - c.z_min;
    primitive.dimensions[1] = c.radio;

    geometry_msgs::msg::Pose pose;
    pose.position.x = c.cx;
    pose.position.y = c.cy;
    pose.position.z = (c.z_min + c.z_max) / 2.0;
    pose.orientation.w = 1.0;

    obj.primitives.push_back(primitive);
    obj.primitive_poses.push_back(pose);
    obj.operation = moveit_msgs::msg::CollisionObject::ADD;

    pub_objects_->publish(obj);
  }
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OctomapVoxelsWithSize>());
  rclcpp::shutdown();
  return 0;
}