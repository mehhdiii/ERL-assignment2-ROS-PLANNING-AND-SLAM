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

#include <memory>
#include <iostream>
#include <list>
#include <map>
#include <algorithm>
#include "geometry_msgs/msg/twist.hpp"
#include "ros2_aruco_interfaces/msg/aruco_markers.hpp"

#include "plansys2_executor/ActionExecutorClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "lifecycle_msgs/msg/state.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;


class ScanMarker : public plansys2::ActionExecutorClient
{
public:
  rclcpp::Subscription<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr aruco_pose_sub_;
  
  ScanMarker()
      : plansys2::ActionExecutorClient("scan_marker", 1s)
  {
  }


  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state)
  {
    RCLCPP_INFO(rclcpp::get_logger("On activate triggered"), "On activate triggered");

    progress_ = 0.0;

      RCLCPP_INFO(rclcpp::get_logger("On 51"), "On 51");


    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    aruco_pose_sub_ = this->create_subscription<ros2_aruco_interfaces::msg::ArucoMarkers>("aruco_poses", 10, std::bind(&ScanMarker::aruco_pose_callback, this, _1));
    RCLCPP_INFO(rclcpp::get_logger("On 57"), "On 57");


    return ActionExecutorClient::on_activate(previous_state);
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state)
  {
    return ActionExecutorClient::on_deactivate(previous_state);
  }

private:
  std::map<int, std::tuple<float, float>> arucos_map;
  int last_id;
  std::tuple<float, float> last_coord; 

  void aruco_pose_callback(const ros2_aruco_interfaces::msg::ArucoMarkers & msg)
  {
    RCLCPP_INFO(rclcpp::get_logger("Received aruco pose"), "Received aruco pose");

    if (msg.poses.size() > 0) {
      // ASSUMING THERE IS ONLY ONE MARKER DETECTED AT ALL TIMES!
      for (long unsigned int i = 0; i < msg.poses.size(); i++) {
        last_id = msg.marker_ids[i];
        last_coord = std::make_tuple(msg.poses[i].position.x, msg.poses[i].position.y);
      }
    }
  }
  void do_work()
  {
      RCLCPP_INFO(rclcpp::get_logger("On 90"), "On 90");

    if (arucos_map.find(last_id) == arucos_map.end()) {
      arucos_map[last_id] = last_coord;
      finish(true, 1.0, "MARKER FOUND! Scanning marker completed");

    }
    send_feedback(progress_, "Scan Marker running");
  }

  float progress_ = 0.0; // 0 MEANS NO MARKER FOUND; 1 MEANS MARKER FOUND

};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ScanMarker>();

  node->set_parameter(rclcpp::Parameter("action_name", "scan_marker"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}
