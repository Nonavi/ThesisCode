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
#include <pcl/features/moment_of_inertia_estimation.h>  // NUEVO: para OBB

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cmath>
#include <limits>

class ShapeDetectorNode : public rclcpp::Node
{
public:
  ShapeDetectorNode()
  : Node("shape_detector")
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      //"/camera/depth_registered/points", 1,
      "/camera/camera/depth/color/points", 1,
      std::bind(&ShapeDetectorNode::cloudCB, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Nodo shape_detector iniciado");
  }

private:
  // Callback: se invoca cada vez que llega una nube de puntos
  void cloudCB(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // Convertir el mensaje ROS2 a una nube de puntos PCL
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::fromROSMsg(*msg, *cloud);

    // Aplicar filtro Passthrough para extraer la región de interés (por ejemplo, en z)
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.07, 0.7);
    pass.filter(*cloud);

    if(cloud->points.empty())
    {
      RCLCPP_WARN(this->get_logger(), "La nube de puntos está vacía tras el filtro");
      return;
    }

    // Realizar detección de formas. Para cada forma, se utiliza la nube original
    detectCylinder(cloud);
    detectSphere(cloud);
    detectBox(cloud);
  }

  // Función para detectar cilindros usando segmentación con normales
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

    // Configurar segmentación para cilindro
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

    // Los coeficientes del cilindro son:
    // [point_on_axis.x, point_on_axis.y, point_on_axis.z, axis.x, axis.y, axis.z, radius]
    double radius = coefficients_cylinder->values[6];
    double ax = coefficients_cylinder->values[3];
    double ay = coefficients_cylinder->values[4];
    double az = coefficients_cylinder->values[5];

    // Calcular el centro aproximado promediando los puntos inliers
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

    // Calcular la altura estimada: proyección de los puntos inliers sobre el eje del cilindro
    double min_proj = std::numeric_limits<double>::max();
    double max_proj = -std::numeric_limits<double>::max();
    Eigen::Vector3d axis_vec(ax, ay, az);
    if(axis_vec.norm() != 0)
      axis_vec.normalize();
    for (auto idx : inliers_cylinder->indices)
    {
      Eigen::Vector3d pt(cloud->points[idx].x, cloud->points[idx].y, cloud->points[idx].z);
      double proj = pt.dot(axis_vec);
      if (proj < min_proj) min_proj = proj;
      if (proj > max_proj) max_proj = proj;
    }
    double height = max_proj - min_proj;

    // Imprimir las características del cilindro
    RCLCPP_INFO(this->get_logger(), "Cilindro detectado:");
    RCLCPP_INFO(this->get_logger(), "  Centro: [%.2f, %.2f, %.2f]", center_x, center_y, center_z);
    RCLCPP_INFO(this->get_logger(), "  Eje (orientación): [%.2f, %.2f, %.2f]", ax, ay, az);
    RCLCPP_INFO(this->get_logger(), "  Radio: %.2f", radius);
    RCLCPP_INFO(this->get_logger(), "  Altura estimada: %.2f", height);
  }

  // Función para detectar esferas usando segmentación (no requiere normales)
  void detectSphere(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud)
  {
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

    RCLCPP_INFO(this->get_logger(), "Esfera detectada:");
    RCLCPP_INFO(this->get_logger(), "  Centro: [%.2f, %.2f, %.2f]", center_x, center_y, center_z);
    RCLCPP_INFO(this->get_logger(), "  Radio: %.2f", radius);
  }

  // Función para detectar una caja calculando su Oriented Bounding Box (OBB)
  void detectBox(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud)
  {
    if (cloud->points.empty())
    {
      RCLCPP_INFO(this->get_logger(), "Nube vacía para detección de caja.");
      return;
    }

    // Calcular los momentos de inercia y extraer el OBB
    pcl::MomentOfInertiaEstimation<pcl::PointXYZRGB> feature_extractor;
    feature_extractor.setInputCloud(cloud);
    feature_extractor.compute();

    pcl::PointXYZRGB min_point_OBB, max_point_OBB, position_OBB;
    Eigen::Matrix3f rotational_matrix_OBB;
    feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);

    // Las dimensiones del OBB se obtienen como la diferencia entre los puntos mínimos y máximos en el marco OBB
    double size_x = max_point_OBB.x - min_point_OBB.x;
    double size_y = max_point_OBB.y - min_point_OBB.y;
    double size_z = max_point_OBB.z - min_point_OBB.z;

    // El centro del OBB se encuentra en 'position_OBB'
    Eigen::Vector3f center_OBB(position_OBB.x, position_OBB.y, position_OBB.z);

    // Convertir la matriz de rotación a ángulos de Euler (roll, pitch, yaw)
    Eigen::Vector3f euler_angles = rotational_matrix_OBB.eulerAngles(0, 1, 2);

    RCLCPP_INFO(this->get_logger(), "Caja detectada (Oriented Bounding Box):");
    RCLCPP_INFO(this->get_logger(), "  Centro: [%.2f, %.2f, %.2f]", center_OBB.x(), center_OBB.y(), center_OBB.z());
    RCLCPP_INFO(this->get_logger(), "  Dimensiones (x, y, z): [%.2f, %.2f, %.2f]", size_x, size_y, size_z);
    RCLCPP_INFO(this->get_logger(), "  Orientación (roll, pitch, yaw): [%.2f, %.2f, %.2f]",
                euler_angles[0], euler_angles[1], euler_angles[2]);
  }

  // Variable miembro: suscripción a la nube de puntos
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ShapeDetectorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
