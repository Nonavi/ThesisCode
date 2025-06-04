#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include <fstream>
#include <unordered_map>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <string>
#include <limits>
#include <cmath>

class PoseLogger : public rclcpp::Node
{
public:
  PoseLogger() : Node("pose_logger_node"), grabando_(false)
  {
    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/rviz/moveit/move_marker/goal_link_tcp", 10,
      std::bind(&PoseLogger::pose_callback, this, std::placeholders::_1));

    toggle_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/activar_grabacion", 10,
      std::bind(&PoseLogger::toggle_callback, this, std::placeholders::_1));

    save_cylinder_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/guardar_cilindro_mas_cercano", 10,
      std::bind(&PoseLogger::save_cylinder_callback, this, std::placeholders::_1));

    cylinder_sub_ = this->create_subscription<moveit_msgs::msg::CollisionObject>(
      "/detected_objects", 50,
      std::bind(&PoseLogger::cylinder_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Nodo pose_logger_node iniciado.");
  }

  ~PoseLogger() {
    cerrar_archivos();
  }

private:
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr toggle_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr save_cylinder_sub_;
  rclcpp::Subscription<moveit_msgs::msg::CollisionObject>::SharedPtr cylinder_sub_;

  std::ofstream log_pose_;
  std::ofstream log_cylinder_;
  bool grabando_;
  geometry_msgs::msg::Pose last_pose_;
  std::unordered_map<std::string, moveit_msgs::msg::CollisionObject> cylinder_cache_;

  std::string get_timestamp()
  {
    auto now = std::chrono::system_clock::now();
    auto t = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      now.time_since_epoch()) % 1000;
    std::ostringstream ss;
    ss << std::put_time(std::localtime(&t), "%F %T")
       << "." << std::setw(3) << std::setfill('0') << ms.count();
    return ss.str();
  }

  std::string get_file_timestamp()
  {
    auto now = std::chrono::system_clock::now();
    auto t = std::chrono::system_clock::to_time_t(now);
    std::ostringstream ss;
    ss << std::put_time(std::localtime(&t), "%F_%H-%M-%S");
    return ss.str();
  }

  void abrir_nuevos_archivos()
  {
    std::string base = "/media/data/Desktop/pycode/AcerUbu/thesis_data_exp/";
    std::string timestamp = get_file_timestamp();

    std::string traj_path = base + "pose_logger_" + timestamp + "_trajectory.csv";
    std::string cyl_path  = base + "pose_logger_" + timestamp + "_poses.csv";

    log_pose_.open(traj_path, std::ios::out | std::ios::trunc);
    log_cylinder_.open(cyl_path, std::ios::out | std::ios::trunc);

    if (!log_pose_.is_open() || !log_cylinder_.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Error al abrir archivos.");
      grabando_ = false;
      return;
    }

    log_pose_ << "timestamp,x,y,z,qx,qy,qz,qw\n";
    log_cylinder_ << "timestamp,id,cx,cy,cz,radio,altura\n";
    grabando_ = true;

    RCLCPP_INFO(this->get_logger(), "Archivos abiertos y grabación iniciada.");
  }

  void cerrar_archivos()
  {
    if (log_pose_.is_open()) log_pose_.close();
    if (log_cylinder_.is_open()) log_cylinder_.close();
    grabando_ = false;
    RCLCPP_INFO(this->get_logger(), "Archivos cerrados y grabación detenida.");
  }

  void toggle_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data && !grabando_) {
      abrir_nuevos_archivos();
    } else if (!msg->data && grabando_) {
      cerrar_archivos();
    }
  }

  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    last_pose_ = msg->pose;

    if (!grabando_ || !log_pose_.is_open()) return;

    const auto& p = msg->pose.position;
    const auto& o = msg->pose.orientation;

    log_pose_ << get_timestamp() << ","
              << p.x << "," << p.y << "," << p.z << ","
              << o.x << "," << o.y << "," << o.z << "," << o.w << "\n";
  }

  void cylinder_callback(const moveit_msgs::msg::CollisionObject::SharedPtr msg)
  {
    // Solo guardamos cilindros válidos
    if (msg->primitives.empty() || msg->primitive_poses.empty()) return;
    if (msg->primitives[0].type != shape_msgs::msg::SolidPrimitive::CYLINDER) return;

    cylinder_cache_[msg->id] = *msg;
  }

  void save_cylinder_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (!msg->data || !grabando_ || !log_cylinder_.is_open()) return;

    std::string closest_id;
    double min_dist = std::numeric_limits<double>::max();
    geometry_msgs::msg::Point cyl_pos;
    double radio = 0.0, altura = 0.0;

    for (const auto& [id, obj] : cylinder_cache_)
    {
      const auto& p = obj.primitive_poses[0].position;
      double dx = last_pose_.position.x - p.x;
      double dy = last_pose_.position.y - p.y;
      double dz = last_pose_.position.z - p.z;
      double dist = std::sqrt(dx*dx + dy*dy + dz*dz);

      if (dist < min_dist) {
        min_dist = dist;
        closest_id = id;
        cyl_pos = p;
        altura = obj.primitives[0].dimensions[0];
        radio = obj.primitives[0].dimensions[1];
      }
    }

    if (!closest_id.empty()) {
      log_cylinder_ << get_timestamp() << ","
                    << closest_id << ","
                    << cyl_pos.x << "," << cyl_pos.y << "," << cyl_pos.z << ","
                    << radio << "," << altura << "\n";

      RCLCPP_INFO(this->get_logger(),
                  "Cilindro guardado: %s en (%.3f, %.3f, %.3f) radio=%.3f altura=%.3f",
                  closest_id.c_str(), cyl_pos.x, cyl_pos.y, cyl_pos.z, radio, altura);
    } else {
      RCLCPP_WARN(this->get_logger(), "No se encontró cilindro válido en caché.");
    }
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseLogger>());
  rclcpp::shutdown();
  return 0;
}
