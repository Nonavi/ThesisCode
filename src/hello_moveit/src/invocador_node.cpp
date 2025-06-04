#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <rclcpp/rclcpp.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <thread>
#include <unordered_map>
#include <vector>
#include <string>

class ObjectSpawner : public rclcpp::Node
{
public:
  ObjectSpawner() : Node("object_spawner")
  {
    subscription_ = this->create_subscription<moveit_msgs::msg::CollisionObject>(
      "detected_objects", 10,
      std::bind(&ObjectSpawner::objectCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Nodo object_spawner iniciado");
  }

private:
  void objectCallback(const moveit_msgs::msg::CollisionObject::SharedPtr msg)
  {
    const std::string& id = msg->id;
    received_count_++;

    // Aplicar el objeto
    planning_scene_interface_.applyCollisionObject(*msg);
    RCLCPP_INFO(this->get_logger(), "Objeto de colision aplicado: %s", id.c_str());

    // Reiniciar contador para este id
    id_counters_[id] = received_count_;

    // Verificar qué ids están viejos
    std::vector<std::string> ids_to_remove;
    for (const auto& pair : id_counters_)
    {
      if ((received_count_ - pair.second) > 50)
        ids_to_remove.push_back(pair.first);
    }

    if (!ids_to_remove.empty())
    {
      planning_scene_interface_.removeCollisionObjects(ids_to_remove);
      for (const std::string& old_id : ids_to_remove)
      {
        RCLCPP_INFO(this->get_logger(), "Removiendo objeto obsoleto: %s", old_id.c_str());
        id_counters_.erase(old_id);
      }
    }
  }

  rclcpp::Subscription<moveit_msgs::msg::CollisionObject>::SharedPtr subscription_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
  std::unordered_map<std::string, size_t> id_counters_;
  size_t received_count_ = 0;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ObjectSpawner>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
