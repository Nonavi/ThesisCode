#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>  // NUEVO
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Transform.h>
#include <memory>
#include <thread>
#include <mutex>
#include <unordered_map>  // NUEVO

class DualSenseTeleop : public rclcpp::Node
{
public:
  DualSenseTeleop() : Node("dual_sense_teleop"),
                      absolute_mode_(true),
                      toggle_mode_pressed_(false),
                      plan_button_pressed_(false),
                      execute_button_pressed_(false),
                      deteccion_activada_(false),
                      deteccion_toggle_pressed_(false),
                      plan_available_(false),
                      vel_dx_(0.0), vel_dy_(0.0), vel_dz_(0.0),
                      vel_droll_(0.0), vel_dpitch_(0.0), vel_dyaw_(0.0)
  {
    joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&DualSenseTeleop::joy_callback, this, std::placeholders::_1));

    goal_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/rviz/moveit/move_marker/goal_link_tcp", 10);

    deteccion_publisher_ = this->create_publisher<std_msgs::msg::Bool>(
      "/activar_deteccion", 10);

    // NUEVO subscriber para comando de pose:
    command_subscriber_ = this->create_subscription<std_msgs::msg::String>(
      "/go_to_predefined_pose", 10,
      std::bind(&DualSenseTeleop::command_callback, this, std::placeholders::_1));

    scale_translation_ = 0.01;
    scale_rotation_ = 0.005;

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(20),
      std::bind(&DualSenseTeleop::update_pose_timer, this));
  }

  void init_move_group()
  {
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      this->shared_from_this(), "xarm7");

    tf2::Quaternion orientation;
    orientation.setRPY(-M_PI, 0.0, 0.0);
    orientation.normalize();

    tf2::Vector3 position(0.206, 0.0, 0.121);

    current_tf_.setOrigin(position);
    current_tf_.setRotation(orientation);

    tf2::toMsg(current_tf_, target_pose_);

    move_group_->setPoseTarget(target_pose_);

    RCLCPP_INFO(this->get_logger(), "Pose inicial del EEF establecida manualmente.");
  }

private:

  // NUEVA función: command_callback
  void command_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    const std::string & command = msg->data;
    RCLCPP_INFO(this->get_logger(), "Recibido comando: '%s'", command.c_str());

    // Comando "plan:"
    if (command.rfind("plan:", 0) == 0) {
      std::string sub_command = command.substr(std::string("plan:").size());
      RCLCPP_INFO(this->get_logger(), "PLAN → preparando pose + planificando: '%s'", sub_command.c_str());

      // Simular recepción de sub-comando → llamar a command_callback de nuevo
      std_msgs::msg::String fake_msg;
      fake_msg.data = sub_command;

      // Mover el marker
      this->command_callback(std::make_shared<std_msgs::msg::String>(fake_msg));

      // Planificar en un hilo
      std::thread planning_thread([this]() {
        moveit::planning_interface::MoveGroupInterface::Plan local_plan;
        {
          std::lock_guard<std::mutex> lock(pose_mutex_);
          move_group_->setPlannerId("RRTstar");
          move_group_->setPlanningTime(1.0);
          move_group_->setPoseTarget(target_pose_);
          if (move_group_->plan(local_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "Planificación exitosa. Listo para ejecutar.");
            std::lock_guard<std::mutex> lock(plan_mutex_);
            last_plan_ = local_plan;
            plan_available_ = true;
          } else {
            RCLCPP_WARN(this->get_logger(), "La planificación falló.");
            plan_available_ = false;
          }
        }
      });
      planning_thread.detach();

      return;
    }

    // Comando "execute"
    if (command == "execute") {
      RCLCPP_INFO(this->get_logger(), "EXECUTE → intentando ejecutar el último plan.");
      if (plan_available_) {
        std::thread exec_thread([this]() {
          RCLCPP_INFO(this->get_logger(), "Ejecutando trayectoria planificada...");
          std::lock_guard<std::mutex> lock(plan_mutex_);
          move_group_->execute(last_plan_);
          RCLCPP_INFO(this->get_logger(), "Ejecución terminada.");
        });
        exec_thread.detach();
      } else {
        RCLCPP_WARN(this->get_logger(), "No hay un plan disponible. Manda primero 'plan:' para planificar.");
      }
      return;
    }

    // Comando "degrees:" → solo mueve el marker (sin plan ni ejecución)
    if (command.rfind("degrees:", 0) == 0) {
      std::istringstream iss(command);
      std::string mode;
      double x_in, y_in, z_in;
      double roll_in, pitch_in, yaw_in;
      iss >> mode >> x_in >> y_in >> z_in >> roll_in >> pitch_in >> yaw_in;

      double roll  = roll_in  * M_PI / 180.0;
      double pitch = pitch_in * M_PI / 180.0;
      double yaw   = yaw_in   * M_PI / 180.0;

      RCLCPP_INFO(this->get_logger(), "Setting Pose → x: %.3f, y: %.3f, z: %.3f | Roll: %.3f deg, Pitch: %.3f deg, Yaw: %.3f deg",
        x_in, y_in, z_in, roll_in, pitch_in, yaw_in);

      tf2::Quaternion ori;
      ori.setRPY(roll, pitch, yaw);
      ori.normalize();

      tf2::Vector3 pos(x_in, y_in, z_in);

      {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        current_tf_.setOrigin(pos);
        current_tf_.setRotation(ori);

        tf2::toMsg(current_tf_, target_pose_);
        move_group_->setPoseTarget(target_pose_);
      }

      // PUBLICAR en RViz:
      geometry_msgs::msg::PoseStamped goal_pose;
      goal_pose.header.stamp = this->now();
      goal_pose.header.frame_id = "world";
      goal_pose.pose = target_pose_;
      goal_pose_publisher_->publish(goal_pose);

      RCLCPP_INFO(this->get_logger(), "Applied Quaternion → x: %.4f, y: %.4f, z: %.4f, w: %.4f",
        ori.x(), ori.y(), ori.z(), ori.w());

      return;
    }

    // Si no es un comando válido:
    RCLCPP_WARN(this->get_logger(), "Comando no reconocido: '%s'", command.c_str());
  }

  
  // === EL RESTO de tus funciones originales ===

  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    if (!move_group_) {
      RCLCPP_WARN(this->get_logger(), "MoveGroupInterface no inicializado.");
      return;
    }

    {
      std::lock_guard<std::mutex> lock(pose_mutex_);

      if (msg->buttons[1] == 1 && !toggle_mode_pressed_) {
        absolute_mode_ = !absolute_mode_;
        toggle_mode_pressed_ = true;
        RCLCPP_INFO(this->get_logger(), "Modo cambiado a: %s", absolute_mode_ ? "ABSOLUTO" : "RELATIVO");
      }
      if (msg->buttons[1] == 0) toggle_mode_pressed_ = false;

      vel_dx_ = msg->axes[0] * scale_translation_;
      if (absolute_mode_) {
        vel_dx_ = msg->axes[0] * scale_translation_;
        vel_dy_ = -msg->axes[1] * scale_translation_;
        vel_dz_ = 0.0;
        if (msg->buttons[9] == 1) vel_dz_ += scale_translation_;
        if (msg->buttons[10] == 1) vel_dz_ -= scale_translation_;
      
        vel_droll_  = msg->axes[2] * scale_rotation_;
        vel_dpitch_ = msg->axes[3] * scale_rotation_;
        vel_dyaw_   = ((msg->axes[4] - msg->axes[5]) / 2.0) * scale_rotation_;
      } else {
        vel_dz_ = msg->axes[1] * scale_translation_;
        vel_dy_ = -msg->axes[0] * scale_translation_;
        vel_dx_ = 0.0;
        if (msg->buttons[9] == 1) vel_dx_ += scale_translation_;
        if (msg->buttons[10] == 1) vel_dx_ -= scale_translation_;
      
        vel_dyaw_   = msg->axes[2] * scale_rotation_;
        vel_dpitch_ = msg->axes[3] * scale_rotation_;
        vel_droll_  = ((msg->axes[4] - msg->axes[5]) / 2.0) * scale_rotation_;
      }
    }

    if (msg->buttons[0] == 1 && !plan_button_pressed_) {
      plan_button_pressed_ = true;
    
      // === PUBLICAR guardar_cilindro_mas_cercano ===
      auto guardar_cilindro_pub = this->create_publisher<std_msgs::msg::Bool>(
        "/guardar_cilindro_mas_cercano", 10);
    
      std_msgs::msg::Bool msg_guardar;
      msg_guardar.data = true;
    
      // Publicamos UNA VEZ (luego destruimos el publisher temporal)
      guardar_cilindro_pub->publish(msg_guardar);
    
      RCLCPP_INFO(this->get_logger(), "Solicitud enviada → Guardar cilindro más cercano.");
    
      // === PLANEAR ===
      std::thread planning_thread([this]() {
        moveit::planning_interface::MoveGroupInterface::Plan local_plan;
        {
          std::lock_guard<std::mutex> lock(pose_mutex_);
          move_group_->setPlannerId("RRTstar");
          move_group_->setPlanningTime(1.0);
          move_group_->setPoseTarget(target_pose_);
          if (move_group_->plan(local_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "Planificación exitosa. Puedes ejecutar el plan.");
            std::lock_guard<std::mutex> lock(plan_mutex_);
            last_plan_ = local_plan;
            plan_available_ = true;
          } else {
            RCLCPP_WARN(this->get_logger(), "La planificación falló.");
            plan_available_ = false;
          }
        }
      });
      planning_thread.detach();
    }
    
    if (msg->buttons[0] == 0)
      plan_button_pressed_ = false;

    if (msg->buttons[3] == 1 && !execute_button_pressed_) {
      execute_button_pressed_ = true;
      if (plan_available_) {
        std::thread exec_thread([this]() {
          RCLCPP_INFO(this->get_logger(), "Ejecutando trayectoria planificada...");
          std::lock_guard<std::mutex> lock(plan_mutex_);
          move_group_->execute(last_plan_);
        });
        exec_thread.detach();
      } else {
        RCLCPP_WARN(this->get_logger(), "No hay un plan disponible. Presiona primero el botón de planear.");
      }
    }
    if (msg->buttons[3] == 0)
      execute_button_pressed_ = false;

    if (msg->buttons[2] == 1 && !deteccion_toggle_pressed_) {
      deteccion_activada_ = !deteccion_activada_;
      deteccion_toggle_pressed_ = true;

      std_msgs::msg::Bool msg_out;
      msg_out.data = deteccion_activada_;
      deteccion_publisher_->publish(msg_out);

      RCLCPP_INFO(this->get_logger(), "Detección %s", deteccion_activada_ ? "ACTIVADA" : "DESACTIVADA");
    }
    if (msg->buttons[2] == 0)
      deteccion_toggle_pressed_ = false;
  }

  void update_pose_timer()
  {
    std::lock_guard<std::mutex> lock(pose_mutex_);

    if (vel_dx_ == 0.0 && vel_dy_ == 0.0 && vel_dz_ == 0.0 &&
        vel_droll_ == 0.0 && vel_dpitch_ == 0.0 && vel_dyaw_ == 0.0) {
      return;
    }

    tf2::Quaternion dq;
    dq.setRPY(vel_droll_, vel_dpitch_, vel_dyaw_);
    dq.normalize();

    tf2::Transform delta_tf(dq, tf2::Vector3(vel_dx_, vel_dy_, vel_dz_));

    if (absolute_mode_) {
      current_tf_ = delta_tf * current_tf_;
    } else {
      current_tf_ = current_tf_ * delta_tf;
    }

    tf2::toMsg(current_tf_, target_pose_);
    move_group_->setPoseTarget(target_pose_);

    geometry_msgs::msg::PoseStamped goal_pose;
    goal_pose.header.stamp = this->now();
    goal_pose.header.frame_id = "world";
    goal_pose.pose = target_pose_;
    goal_pose_publisher_->publish(goal_pose);
  }

  // ROS2
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_subscriber_;  // NUEVO
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr deteccion_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

  tf2::Transform current_tf_;
  geometry_msgs::msg::Pose target_pose_;
  double scale_translation_;
  double scale_rotation_;
  bool absolute_mode_;
  bool toggle_mode_pressed_;
  bool plan_button_pressed_;
  bool execute_button_pressed_;

  bool deteccion_activada_;
  bool deteccion_toggle_pressed_;

  double vel_dx_, vel_dy_, vel_dz_;
  double vel_droll_, vel_dpitch_, vel_dyaw_;

  moveit::planning_interface::MoveGroupInterface::Plan last_plan_;
  bool plan_available_;
  std::mutex pose_mutex_;
  std::mutex plan_mutex_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DualSenseTeleop>();
  node->init_move_group();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
