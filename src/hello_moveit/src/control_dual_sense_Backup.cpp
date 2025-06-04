// Este era el codigo utilizado. Tiene muchos puntos a mejorarse, pero era funcional

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Transform.h>
#include <memory>
#include <thread>
#include <mutex>

class DualSenseTeleop : public rclcpp::Node
{
public:
  DualSenseTeleop() : Node("dual_sense_teleop"),
                      absolute_mode_(true),
                      toggle_mode_pressed_(false),
                      plan_button_pressed_(false)
  {
    joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&DualSenseTeleop::joy_callback, this, std::placeholders::_1));

    goal_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/rviz/moveit/move_marker/goal_link_tcp", 10);

    scale_translation_ = 0.01;
    scale_rotation_ = 0.005;
  }

  void init_move_group()
  {
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      this->shared_from_this(), "xarm7");

    geometry_msgs::msg::PoseStamped current_pose_msg = move_group_->getCurrentPose();
    tf2::fromMsg(current_pose_msg.pose, current_tf_);
  }

private:
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    if (!move_group_) {
      RCLCPP_WARN(this->get_logger(), "MoveGroupInterface no inicializado.");
      return;
    }

    {
      std::lock_guard<std::mutex> lock(pose_mutex_);

      // --- Toggle Modo Absoluto/Relativo ---
      if (msg->buttons[1] == 1 && !toggle_mode_pressed_) {
        absolute_mode_ = !absolute_mode_;
        toggle_mode_pressed_ = true;
        RCLCPP_INFO(this->get_logger(), "Modo cambiado a: %s", absolute_mode_ ? "ABSOLUTO" : "RELATIVO");
      }
      if (msg->buttons[1] == 0) toggle_mode_pressed_ = false;

      // --- Entrada del usuario ---
      double dx = msg->axes[0] * scale_translation_;
      double dy = -msg->axes[1] * scale_translation_;
      double dz = 0.0;
      if (msg->buttons[9] == 1) dz += scale_translation_;
      if (msg->buttons[10] == 1) dz -= scale_translation_;

      double droll  = msg->axes[2] * scale_rotation_;
      double dpitch = msg->axes[3] * scale_rotation_;
      double dyaw   = ((msg->axes[4] - msg->axes[5]) / 2.0) * scale_rotation_;

      // Crear delta como transformación homogénea
      tf2::Quaternion dq;
      dq.setRPY(droll, dpitch, dyaw);
      dq.normalize();

      tf2::Transform delta_tf(dq, tf2::Vector3(dx, dy, dz));

      // --- Aplicar según modo ---
      if (absolute_mode_) {
        current_tf_ = delta_tf * current_tf_; // En marco global
      } else {
        current_tf_ = current_tf_ * delta_tf; // En marco local
      }

      // Convertir a msg y publicar
      tf2::toMsg(current_tf_, target_pose_);
      move_group_->setPoseTarget(target_pose_);
    }

    // --- Publicar a RViz ---
    geometry_msgs::msg::PoseStamped goal_pose;
    goal_pose.header.stamp = this->now();
    goal_pose.header.frame_id = "world";
    {
      std::lock_guard<std::mutex> lock(pose_mutex_);
      goal_pose.pose = target_pose_;
    }
    goal_pose_publisher_->publish(goal_pose);

    // --- Planificación/Ejecución ---
    if (msg->buttons[0] == 1 && !plan_button_pressed_) {
      plan_button_pressed_ = true;
      std::thread planning_thread([this]() {
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        {
          std::lock_guard<std::mutex> lock(pose_mutex_);
          if (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "Planificación exitosa. Ejecutando trayectoria...");
            move_group_->execute(plan);
          } else {
            RCLCPP_WARN(this->get_logger(), "La planificación falló.");
          }
        }
      });
      planning_thread.detach();
    }
    if (msg->buttons[0] == 0)
      plan_button_pressed_ = false;
  }

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_publisher_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

  tf2::Transform current_tf_;  // MATRIZ de transformación actual
  geometry_msgs::msg::Pose target_pose_;  // Solo para MoveIt

  double scale_translation_;
  double scale_rotation_;

  bool absolute_mode_;
  bool toggle_mode_pressed_;
  bool plan_button_pressed_;

  std::mutex pose_mutex_;
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
