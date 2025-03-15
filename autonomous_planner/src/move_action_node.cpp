// Copyright 2019 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <math.h>
#include <cmath>


#include <memory>
#include <string>
#include <map>
#include <algorithm>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

#include "plansys2_executor/ActionExecutorClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "autonomous_planner_interfaces/srv/get_last_marker.hpp"
#include "std_msgs/msg/string.hpp"

using GetLastMarker = autonomous_planner_interfaces::srv::GetLastMarker;
using namespace std::chrono_literals;
double tolerance=0.5;
class MoveAction : public plansys2::ActionExecutorClient
{
public:
  MoveAction():
  plansys2::ActionExecutorClient("move", 1s)
  { 
    geometry_msgs::msg::PoseStamped wp;
    wp.header.frame_id = "map";
    wp.header.stamp = now();
    wp.pose.position.x = 5.2;
    wp.pose.position.y = 2.3;
    waypoints_["wp1"] = wp;

    wp.pose.position.x = 6.97;
    wp.pose.position.y = -5.12;
    waypoints_["wp2"] = wp;

    wp.header.stamp = now();
    wp.pose.position.x = -3.3;
    wp.pose.position.y = -7.8;
    waypoints_["wp3"] = wp;

    wp.pose.position.x = -7.03;
    wp.pose.position.y = 1.28;
    waypoints_["wp4"] = wp;

    wp.pose.position.x = 2.0;
    wp.pose.position.y = 2.0;
    waypoints_["wp_init"] = wp;

    using namespace std::placeholders;
    pos_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/amcl_pose",
      10,
      std::bind(&MoveAction::current_pos_callback, this, _1));
    
    navigation_action_client_ =
      rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
      this,
      "navigate_to_pose");

    smallest_wp_subscriber_ = this->create_subscription<std_msgs::msg::String>(
      "smallest_wp_topic", 10, std::bind(&MoveAction::topic_callback, this, std::placeholders::_1));

  }

  void current_pos_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    current_pos_ = msg->pose.pose;
  }

  void marker_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    smallest_wp = msg->data;
    RCLCPP_INFO(get_logger(), "Received waypoint: %s", smallest_wp.c_str());
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & previous_state)
  {

    RCLCPP_INFO(get_logger(), "on_activate");

    send_feedback(0.0, "Move starting");

    bool is_action_server_ready = false;
    do {
      RCLCPP_INFO(get_logger(), "Waiting for navigation action server...");

      is_action_server_ready =
        navigation_action_client_->wait_for_action_server(std::chrono::seconds(5));
    } while (!is_action_server_ready);

    RCLCPP_INFO(get_logger(), "Navigation action server ready");

    std::string wp_to_navigate = get_arguments()[2] ;  // The goal is in the 3rd argument of the action
   
    RCLCPP_INFO(get_logger(), "Start navigation to [%s]", wp_to_navigate.c_str());

    if (wp_to_navigate == "wpf") {
      RCLCPP_INFO(get_logger(), "inside wpf if statement");
      wp_to_navigate = smallest_wp;
      RCLCPP_INFO(get_logger(), "Received waypoint from topic: %s", wp_to_navigate.c_str());
    }

    goal_pos_ = waypoints_[wp_to_navigate];
    navigation_goal_.pose = goal_pos_;

    dist_to_move = getDistance(goal_pos_.pose, current_pos_);

    auto send_goal_options =
      rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
    RCLCPP_INFO(get_logger(), "t7t send goal options");

    send_goal_options.feedback_callback = [this](
      NavigationGoalHandle::SharedPtr,
      NavigationFeedback feedback) {
        send_feedback(
          std::min(1.0, std::max(0.0, 1.0 - (feedback->distance_remaining / dist_to_move))),
          "Move running");

        progress= std::min(1.0, std::max(0.0, 1.0 - (feedback->distance_remaining / dist_to_move)));
        progress_f=1-progress;
      };

    send_goal_options.result_callback = [this](auto) {
        finish(true, 1.0, "Move completed");
      };

    future_navigation_goal_handle_ =
      navigation_action_client_->async_send_goal(navigation_goal_, send_goal_options);

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

private:

  std::string smallest_wp;
  void topic_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    smallest_wp = msg->data.c_str();
    // Log the received waypoint message
    RCLCPP_INFO(this->get_logger(), "Received waypoint: '%s'", smallest_wp);
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr smallest_wp_subscriber_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr marker_sub_;


  double getDistance(const geometry_msgs::msg::Pose & pos1, const geometry_msgs::msg::Pose & pos2)
  {
    return sqrt(
      (pos1.position.x - pos2.position.x) * (pos1.position.x - pos2.position.x) +
      (pos1.position.y - pos2.position.y) * (pos1.position.y - pos2.position.y));
  }

  double progress;
  double progress_f;
  bool done=((progress_f)<tolerance);
   
  void do_work()
  {
        
  }

  std::map<std::string, geometry_msgs::msg::PoseStamped> waypoints_;

  using NavigationGoalHandle =
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>;
  using NavigationFeedback =
    const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback>;

  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr navigation_action_client_;
  std::shared_future<NavigationGoalHandle::SharedPtr> future_navigation_goal_handle_;
  NavigationGoalHandle::SharedPtr navigation_goal_handle_;

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pos_sub_;
  geometry_msgs::msg::Pose current_pos_;
  geometry_msgs::msg::PoseStamped goal_pos_;
  nav2_msgs::action::NavigateToPose::Goal navigation_goal_;

  double dist_to_move;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MoveAction>();

  node->set_parameter(rclcpp::Parameter("action_name", "move"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}
