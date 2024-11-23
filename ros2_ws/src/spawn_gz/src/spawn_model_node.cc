#include <vector>
#include <memory>
#include <chrono>

#include <ignition/plugin/Register.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/World.hh>
#include <ignition/gazebo/EntityComponentManager.hh>
#include "ignition/gazebo/Entity.hh"
#include <ignition/gazebo/gui/GuiEvents.hh>
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Entity.hh>
#include "ignition/msgs.hh"
#include "ignition/math.hh"
#include "ignition/transport.hh"
#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "ras_interfaces/msg/spawn_request.hpp" // Include the custom message

int object_number = 0;
// Declaring ROS 2 and ignition Nodes
ignition::transport::Node gz_node;
std::shared_ptr<rclcpp::Node> ros_node;

// Function to spawn models inside Gazebo
void SpawnModelgz(const std::string &model_name, const geometry_msgs::msg::Pose &pose, int i)
{
  // Using the /create service to spawn models
  bool result;
  ignition::msgs::EntityFactory req;
  ignition::msgs::Boolean res;
  req.set_name(model_name + std::to_string(i));

  RCLCPP_INFO(ros_node->get_logger(), ("Spawning model: " + model_name).c_str());

  // Storing the location of gpt_gz package
  std::string gpt_gz_pkg_dir = ament_index_cpp::get_package_share_directory("ras_sim");

  // Passing the .sdf file of the model we need to spawn
  req.set_sdf_filename(gpt_gz_pkg_dir + "/models/" + model_name + "/model.sdf");

  // Setting the pose of the model
  ignition::msgs::Set(req.mutable_pose(), ignition::math::Pose3d(
                                              pose.position.x, pose.position.y, pose.position.z,
                                              pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z));

  int timeout = 300;

  // Executed is a boolean whose value tells us if the service is available or not
  bool executed = gz_node.Request("/world/empty/create", req, timeout, res, result);

  if (executed)
  {
    RCLCPP_INFO(ros_node->get_logger(), "Service call made successfully.");
  }
  else
  {
    RCLCPP_WARN(ros_node->get_logger(), ("Service call failed. Unable to spawn model: " + model_name).c_str());
  }

  return;
}

// Callback function for spawn_model topic
void SpawnModelCb(const ras_interfaces::msg::SpawnRequest::SharedPtr msg)
{
  // Use the SpawnModelgz function to spawn the model at the received pose
  SpawnModelgz(msg->model_name, msg->pose, object_number);
  object_number++;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  ros_node = rclcpp::Node::make_shared("spawn_model_node");

  RCLCPP_INFO(ros_node->get_logger(), "spawn_model_node active");

  // Subscriber to /spawn_model topic
  auto spawn_model_subscriber = ros_node->create_subscription<ras_interfaces::msg::SpawnRequest>(
      "spawn_model", 10, &SpawnModelCb);

  rclcpp::spin(ros_node);

  rclcpp::shutdown();
}
