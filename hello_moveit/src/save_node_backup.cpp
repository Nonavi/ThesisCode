#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <fstream>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <string>

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

    RCLCPP_INFO(this->get_logger(), "Nodo listo. Esperando /activar_grabacion...");
  }

  ~PoseLogger() {
    cerrar_archivo();
  }

private:
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr toggle_sub_;
  std::ofstream log_file_;
  bool grabando_;

  std::string get_timestamp()
  {
    auto now = std::chrono::system_clock::now();
    auto now_t = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      now.time_since_epoch()) % 1000;

    std::ostringstream ss;
    ss << std::put_time(std::localtime(&now_t), "%F %T")
       << "." << std::setw(3) << std::setfill('0') << ms.count();
    return ss.str();
  }

  std::string get_file_timestamp()
  {
    auto now = std::chrono::system_clock::now();
    auto now_t = std::chrono::system_clock::to_time_t(now);
    std::ostringstream ss;
    ss << std::put_time(std::localtime(&now_t), "%F_%H-%M-%S");
    return ss.str();
  }

  void abrir_nuevo_archivo()
  {
    std::string base_path = "/media/data/Desktop/pycode/AcerUbu/thesis_data_exp/";
    std::string filename = "pose_log_" + get_file_timestamp() + ".csv";
    std::string full_path = base_path + filename;

    log_file_.open(full_path, std::ios::out | std::ios::trunc);
    if (!log_file_.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "No se pudo abrir el archivo: %s", full_path.c_str());
    } else {
      RCLCPP_INFO(this->get_logger(), "Grabando en archivo: %s", full_path.c_str());
      log_file_ << "timestamp,x,y,z,qx,qy,qz,qw\n";
      grabando_ = true;
    }
  }

  void cerrar_archivo()
  {
    if (log_file_.is_open()) {
      log_file_.close();
      RCLCPP_INFO(this->get_logger(), "Grabación detenida.");
    }
    grabando_ = false;
  }

  void toggle_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data && !grabando_) {
      abrir_nuevo_archivo();
    } else if (!msg->data && grabando_) {
      cerrar_archivo();
    }
  }

  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (!grabando_ || !log_file_.is_open()) return;

    const auto& p = msg->pose.position;
    const auto& o = msg->pose.orientation;

    log_file_ << get_timestamp() << ","
              << p.x << "," << p.y << "," << p.z << ","
              << o.x << "," << o.y << "," << o.z << "," << o.w << "\n";
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseLogger>());
  rclcpp::shutdown();
  return 0;
}
