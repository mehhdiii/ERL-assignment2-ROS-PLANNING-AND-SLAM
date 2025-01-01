#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from ros2_aruco_interfaces.action import MarkerAction, MarkerResult, MarkerGoal
from autonomous_planner.srv import GetLastMarker  

from ros2_aruco_interfaces.msg import ArucoMarkers


class MarkerActionServer(Node):
    def __init__(self):
        super().__init__('scan_action_node')

        # Create subscriber for /aruco_markers topic
        self.subscriber_ = self.create_subscription(
            ArucoMarkers, 
            '/aruco_markers',
            self.marker_callback,
            10
        )

        # Create service for get_last_marker
        self._service = self.create_service(
            GetLastMarker,
            'get_last_marker',
            self.get_last_marker_service_callback
        )

        # To store all markers and the last one detected
        self.markers = {}
        self.last_marker = None  # Initialize as None to handle no markers detected


    def marker_callback(self, msg):
        # Handle the markers received in the ArUcoMarkers message
        for marker_id in msg.marker_ids:
            self.markers[marker_id] = 1  # Storing markers, assuming '1' is just an example of marker's status
            self.last_marker = marker_id  # Update the last marker to the most recent one

        self.get_logger().info(f"Detected markers: {self.markers.keys()}, Last marker ID: {self.last_marker}")

    def get_last_marker_service_callback(self, request, response):
        # This service callback will return the last marker ID
        if self.last_marker is not None:
            response.marker_id = self.last_marker
            self.get_logger().info(f"Service: Returning last marker ID {self.last_marker}")
        else:
            response.marker_id = -1  # No markers detected
            self.get_logger().info(f"Service: No markers detected")
        return response


def main(args=None):
    rclpy.init(args=args)

    action_server = MarkerActionServer()

    rclpy.spin(action_server)

    rclpy.shutdown()

