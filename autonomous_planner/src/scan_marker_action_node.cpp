#include <memory>
#include <iostream>
#include <list>
#include <map>
#include <algorithm>
#include "geometry_msgs/msg/twist.hpp"
#include "ros2_aruco_interfaces/msg/aruco_markers.hpp"
#include "plansys2_executor/ActionExecutorClient.hpp"

#include "autonomous_planner_interfaces/srv/get_last_marker.hpp"

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

    rclcpp::Service<autonomous_planner_interfaces::srv::GetLastMarker>::SharedPtr service =
        this->create_service<autonomous_planner_interfaces::srv::GetLastMarker>("get_smallest_aruco", std::bind(&ScanMarker::get_smallest_aruco_callback, this, std::placeholders::_1, std::placeholders::_2) );


    aruco_pose_sub_ = this->create_subscription<ros2_aruco_interfaces::msg::ArucoMarkers>("aruco_markers", 10, std::bind(&ScanMarker::aruco_pose_callback, this, _1));
    

    return ActionExecutorClient::on_activate(previous_state);
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state)
  {
    return ActionExecutorClient::on_deactivate(previous_state);
  }


private:
  // SUBSCRIBER: ARUCO POSE
  std::map<int, std::tuple<float, float, float>> arucos_map;
  int last_id = 0;
  std::tuple<float, float, float> last_coord; 
  // float last_theta; 


  void aruco_pose_callback(const ros2_aruco_interfaces::msg::ArucoMarkers & msg)
  {
    RCLCPP_INFO(rclcpp::get_logger("Received aruco pose"), "Received aruco pose");

    if (msg.poses.size() > 0) {
      // ASSUMING THERE IS ONLY ONE MARKER DETECTED AT ALL TIMES!
      for (long unsigned int i = 0; i < msg.poses.size(); i++) {
        if (msg.marker_ids[i] != 0){
          last_id = msg.marker_ids[i];
          last_coord = std::make_tuple(msg.poses[i].position.x, msg.poses[i].position.y, msg.poses[i].orientation.z);
   
        }
      }
    }
  }

  // SUBSCRIBER: SYSPLAN ACTION
  void do_work()
  {
      RCLCPP_INFO(rclcpp::get_logger("On 90"), "On 90");

    if (last_id != 0 && arucos_map.find(last_id) == arucos_map.end()) {
      arucos_map[last_id] = last_coord;
      progress_ = 0;
      RCLCPP_INFO(rclcpp::get_logger("Found marker"), "Found marker. id: %d", last_id);

      finish(true, 1.0, "MARKER FOUND! Scanning marker completed");

    }
    send_feedback(progress_, "Scan Marker running");
  }

  float progress_ = 0.0; // 0 MEANS NO MARKER FOUND; 1 MEANS MARKER FOUND
    // SERVICE: GET SMALLEST ARUCO POSE

  void get_smallest_aruco_callback(const std::shared_ptr<autonomous_planner_interfaces::srv::GetLastMarker::Request> request,
                                   std::shared_ptr<autonomous_planner_interfaces::srv::GetLastMarker::Response> response)
  {
    (void)request;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request");
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response");


    int smallest_last_id = arucos_map.begin()->first;

    response->marker_id = smallest_last_id;
    response->x = std::get<0>(arucos_map[smallest_last_id]);
    response->y = std::get<1>(arucos_map[smallest_last_id]);
    response->theta = std::get<2>(arucos_map[smallest_last_id]);
  }
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
