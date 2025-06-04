#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <cmath>
#include <limits>
#include <string>

class DetectorNode : public rclcpp::Node
{
public:
  DetectorNode() : Node("detector")
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/camera/camera/depth/color/points", 1,
      std::bind(&DetectorNode::cloudCB, this, std::placeholders::_1));
      
    publisher_ = this->create_publisher<moveit_msgs::msg::CollisionObject>("detected_objects", 10);
    RCLCPP_INFO(this->get_logger(), "Nodo detector iniciado");
  }

private:
  void cloudCB(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // Convertir el mensaje ROS a una nube PCL
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::fromROSMsg(*msg, *cloud);
    
    // Filtrar la nube de puntos en el eje z (región de interés)
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.07, 0.5); //Camera Specs
    pass.filter(*cloud);

    // Filtro en x
    pass.setFilterFieldName("x");
    pass.setFilterLimits(-0.45, 0.45);  // Max Depth * Tan (Horizontal FOV)
    pass.filter(*cloud);

    // Filtro en y
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-0.277, 0.277); //Max Depth * Tan (Vertical FOV)
    pass.filter(*cloud);

    if (cloud->points.empty())
    {
      RCLCPP_WARN(this->get_logger(), "La nube de puntos está vacía tras el filtro");
      return;
    }
    
    // Llamar a cada función de detección
    detectCylinder(cloud);
    detectSphere(cloud);
    detectBox(cloud);
  }

  void detectCylinder(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud)
  {
    // Calcular normales
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    ne.setSearchMethod(tree);
    ne.setInputCloud(cloud);
    ne.setKSearch(50);
    ne.compute(*normals);
    
    // Segmentación del cilindro
    pcl::SACSegmentationFromNormals<pcl::PointXYZRGB, pcl::Normal> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_CYLINDER);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight(0.1);
    seg.setMaxIterations(10000);
    seg.setDistanceThreshold(0.05);
    seg.setRadiusLimits(0, 1);
    seg.setInputCloud(cloud);
    seg.setInputNormals(normals);
    
    pcl::PointIndices::Ptr inliers_cylinder(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients_cylinder(new pcl::ModelCoefficients);
    seg.segment(*inliers_cylinder, *coefficients_cylinder);
    
    if (inliers_cylinder->indices.size() < 50)  // Umbral arbitrario
    {
      RCLCPP_INFO(this->get_logger(), "No se detectó un cilindro.");
      return;
    }
    
    // Los coeficientes son:
    // [point_on_axis.x, point_on_axis.y, point_on_axis.z, axis.x, axis.y, axis.z, radius]
    double radius = coefficients_cylinder->values[6];
    double ax = coefficients_cylinder->values[3];
    double ay = coefficients_cylinder->values[4];
    double az = coefficients_cylinder->values[5];
    
    // Calcular el centro promediando los puntos inliers
    double sum_x = 0, sum_y = 0, sum_z = 0;
    for (auto idx : inliers_cylinder->indices)
    {
      sum_x += cloud->points[idx].x;
      sum_y += cloud->points[idx].y;
      sum_z += cloud->points[idx].z;
    }
    size_t count = inliers_cylinder->indices.size();
    double center_x = sum_x / count;
    double center_y = sum_y / count;
    double center_z = sum_z / count;
    
    // Calcular la altura estimada proyectando los puntos sobre el eje del cilindro
    double min_proj = std::numeric_limits<double>::max();
    double max_proj = -std::numeric_limits<double>::max();
    Eigen::Vector3d axis_vec(ax, ay, az);
    if (axis_vec.norm() != 0)
      axis_vec.normalize();
    for (auto idx : inliers_cylinder->indices)
    {
      Eigen::Vector3d pt(cloud->points[idx].x, cloud->points[idx].y, cloud->points[idx].z);
      double proj = pt.dot(axis_vec);
      if (proj < min_proj) min_proj = proj;
      if (proj > max_proj) max_proj = proj;
    }
    double height = max_proj - min_proj;
    
    RCLCPP_INFO(this->get_logger(), "Cilindro detectado: Centro: [%.2f, %.2f, %.2f] Radio: %.2f Altura: %.2f",
                center_x, center_y, center_z, radius, height);
    
    // Crear el mensaje CollisionObject para el cilindro
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = "link_base";  // Asegúrate que coincide con tu frame de planificación
    collision_object.id = "detected_cylinder";
    
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.CYLINDER;
    primitive.dimensions.resize(2);
    // Para cilindro: dimensions[0] = altura, dimensions[1] = radio
    primitive.dimensions[primitive.CYLINDER_HEIGHT] = height;
    primitive.dimensions[primitive.CYLINDER_RADIUS] = radius;
    
    // Calcular la orientación: la primitiva por defecto está alineada con el eje Z,
    // se rota para alinear el eje Z con el eje detectado
    Eigen::Vector3d z_axis(0.0, 0.0, 1.0);
    Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(z_axis, axis_vec);
    
    geometry_msgs::msg::Pose cylinder_pose;
    cylinder_pose.position.x = center_x;
    cylinder_pose.position.y = center_y;
    cylinder_pose.position.z = center_z;
    cylinder_pose.orientation.x = q.x();
    cylinder_pose.orientation.y = q.y();
    cylinder_pose.orientation.z = q.z();
    cylinder_pose.orientation.w = q.w();
    
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(cylinder_pose);
    collision_object.operation = collision_object.ADD;
    
    publisher_->publish(collision_object);
    RCLCPP_INFO(this->get_logger(), "Objeto cilindro publicado: %s", collision_object.id.c_str());
  }
  
  void detectSphere(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud)
  {
    // Segmentación para esfera
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_SPHERE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(10000);
    seg.setDistanceThreshold(0.05);
    seg.setInputCloud(cloud);
    
    pcl::PointIndices::Ptr inliers_sphere(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients_sphere(new pcl::ModelCoefficients);
    seg.segment(*inliers_sphere, *coefficients_sphere);
    
    if (inliers_sphere->indices.size() < 50)
    {
      RCLCPP_INFO(this->get_logger(), "No se detectó una esfera.");
      return;
    }
    
    // Los coeficientes de la esfera son: [center_x, center_y, center_z, radius]
    double center_x = coefficients_sphere->values[0];
    double center_y = coefficients_sphere->values[1];
    double center_z = coefficients_sphere->values[2];
    double radius = coefficients_sphere->values[3];
    
    RCLCPP_INFO(this->get_logger(), "Esfera detectada: Centro: [%.2f, %.2f, %.2f] Radio: %.2f",
                center_x, center_y, center_z, radius);
    
    // Crear el mensaje CollisionObject para la esfera
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = "link_base";
    collision_object.id = "detected_sphere";
    
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.SPHERE;
    primitive.dimensions.resize(1);
    // Para esfera: dimensions[0] = radio
    primitive.dimensions[primitive.SPHERE_RADIUS] = radius;
    
    geometry_msgs::msg::Pose sphere_pose;
    sphere_pose.position.x = center_x;
    sphere_pose.position.y = center_y;
    sphere_pose.position.z = center_z;
    sphere_pose.orientation.w = 1.0;  // Identidad
    
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(sphere_pose);
    collision_object.operation = collision_object.ADD;
    
    publisher_->publish(collision_object);
    RCLCPP_INFO(this->get_logger(), "Objeto esfera publicado: %s", collision_object.id.c_str());
  }
  
  void detectBox(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud)
  {
    if (cloud->points.empty())
    {
      RCLCPP_INFO(this->get_logger(), "Nube vacía para detección de caja.");
      return;
    }
    
    // Calcular el Oriented Bounding Box (OBB)
    pcl::MomentOfInertiaEstimation<pcl::PointXYZRGB> feature_extractor;
    feature_extractor.setInputCloud(cloud);
    feature_extractor.compute();
    
    pcl::PointXYZRGB min_point_OBB, max_point_OBB, position_OBB;
    Eigen::Matrix3f rotational_matrix_OBB;
    feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
    
    double size_x = max_point_OBB.x - min_point_OBB.x;
    double size_y = max_point_OBB.y - min_point_OBB.y;
    double size_z = max_point_OBB.z - min_point_OBB.z;
    
    Eigen::Vector3f center_OBB(position_OBB.x, position_OBB.y, position_OBB.z);
    Eigen::Quaternionf quat(rotational_matrix_OBB);
    
    RCLCPP_INFO(this->get_logger(), "Caja detectada: Centro: [%.2f, %.2f, %.2f] Dimensiones: [%.2f, %.2f, %.2f]",
                center_OBB.x(), center_OBB.y(), center_OBB.z(), size_x, size_y, size_z);
    
    // Crear el mensaje CollisionObject para la caja
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = "link_base";
    collision_object.id = "detected_box";
    
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = size_x;
    primitive.dimensions[primitive.BOX_Y] = size_y;
    primitive.dimensions[primitive.BOX_Z] = size_z;
    
    geometry_msgs::msg::Pose box_pose;
    box_pose.position.x = center_OBB.x();
    box_pose.position.y = center_OBB.y();
    box_pose.position.z = center_OBB.z();
    box_pose.orientation.x = quat.x();
    box_pose.orientation.y = quat.y();
    box_pose.orientation.z = quat.z();
    box_pose.orientation.w = quat.w();
    
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;
    
    publisher_->publish(collision_object);
    RCLCPP_INFO(this->get_logger(), "Objeto caja publicado: %s", collision_object.id.c_str());
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<moveit_msgs::msg::CollisionObject>::SharedPtr publisher_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DetectorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
