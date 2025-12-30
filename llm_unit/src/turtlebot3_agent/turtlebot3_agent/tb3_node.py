#!/usr/bin/env python3
"""
TurtleBot3 agent node for ROS2.
This version only publishes PoseStamped goals to MPC controller
based on user commands; no direct movement is done.
"""

import math
import threading

import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from transforms3d.euler import euler2quat, quat2euler

from turtlebot3_agent.utils import normalize_angle

TOPIC_ODOMETRY = "/odom"
TOPIC_IMG = "/camera/image_raw"
TOPIC_SCAN = "/scan"
TOPIC_NMPC_GOAL = "/goal"


class TB3Agent(Node):
    def __init__(self):
        super().__init__("turtlebot3_agent")
        self.declare_parameter("interface", "cli")
        self.declare_parameter("agent_model", "gemini-2.0-flash")

        self.interface = (
            self.get_parameter("interface").get_parameter_value().string_value
        )
        self.agent_model = (
            self.get_parameter("agent_model").get_parameter_value().string_value
        )

        # Publishers and subscribers
        self.goal_pub = self.create_publisher(PoseStamped, TOPIC_NMPC_GOAL, 10)
        self.create_subscription(Odometry, TOPIC_ODOMETRY, self.odom_callback, 10)
        self.create_subscription(LaserScan, TOPIC_SCAN, self.scan_callback, 10)
        self.create_subscription(Image, TOPIC_IMG, self.image_callback, 10)

        # Sensor data
        self.scan = None
        self.odom = None
        self.camera_image = None

        # Robot state
        self.x, self.y, self.yaw = 0.0, 0.0, 0.0
        self.bridge = CvBridge()

        # Synchronization
        self.pose_ready = threading.Event()

    # -------------------- Callbacks --------------------
    def scan_callback(self, msg):
        self.scan = msg

    def odom_callback(self, msg):
        self.odom = msg
        self.x, self.y, self.yaw = self.get_pose(msg)
        self.pose_ready.set()

    def get_pose(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q_x = msg.pose.pose.orientation.x
        q_y = msg.pose.pose.orientation.y
        q_z = msg.pose.pose.orientation.z
        q_w = msg.pose.pose.orientation.w
        (_, _, yaw) = quat2euler((q_w, q_x, q_y, q_z))
        return x, y, yaw

    def wait_for_pose(self, timeout=1.0):
        return self.pose_ready.wait(timeout=timeout)

    def image_callback(self, msg):
        self.camera_image = msg

    def get_latest_image(self) -> Image:
        if self.camera_image is None:
            raise RuntimeError("No image received yet.")
        return self.camera_image

    # -------------------- Movement functions (compute pose only) --------------------
    def move_linear(self, distance_m: float):
        """
        Compute a new pose forward/backward by distance_m along current yaw,
        then publish it as PoseStamped goal to MPC.
        """
        if not self.wait_for_pose():
            self.get_logger().warning("Initial pose not received yet.")
            return None

        x_goal = self.x + distance_m * math.cos(self.yaw)
        y_goal = self.y + distance_m * math.sin(self.yaw)
        yaw_goal = self.yaw
        return self.publish_pose_goal(x_goal, y_goal, yaw_goal)

    def rotate_angle(self, angle_rad: float):
        """
        Compute a new yaw by rotating current yaw by angle_rad,
        then publish it as PoseStamped goal to MPC.
        """
        if not self.wait_for_pose():
            self.get_logger().warning("Initial pose not received yet.")
            return None

        x_goal = self.x
        y_goal = self.y
        yaw_goal = self.yaw + angle_rad
        return self.publish_pose_goal(x_goal, y_goal, yaw_goal)

    def move_non_linear(self, duration_sec, linear_velocity, angular_velocity):
        """
        Compute a new pose after moving in curve for duration_sec
        and publish it to MPC.
        """
        if not self.wait_for_pose():
            self.get_logger().warning("Initial pose not received yet.")
            return None

        x_goal = self.x + linear_velocity * duration_sec * math.cos(self.yaw)
        y_goal = self.y + linear_velocity * duration_sec * math.sin(self.yaw)
        yaw_goal = self.yaw + angular_velocity * duration_sec
        return self.publish_pose_goal(x_goal, y_goal, yaw_goal)

    # -------------------- Publish PoseStamped --------------------
    def publish_pose_goal(self, x, y, yaw):
        goal = PoseStamped()
        goal.header.frame_id = "base_link"  # simplified
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = 0.0
        quat = euler2quat(0, 0, yaw)
        goal.pose.orientation.w = quat[0]
        goal.pose.orientation.x = quat[1]
        goal.pose.orientation.y = quat[2]
        goal.pose.orientation.z = quat[3]

        self.goal_pub.publish(goal)
        self.get_logger().info(f"[MPC GOAL PUBLISHED] x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}")

        return f"My computed goal pose is x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}"

    # -------------------- Compatibility --------------------
    def navigate_to_pose(self, x, y, yaw, timeout_sec=30.0):
        return self.publish_pose_goal(x, y, yaw)

    def transform_odom_to_map(self, odom_x, odom_y, odom_yaw):
        return odom_x, odom_y, odom_yaw
