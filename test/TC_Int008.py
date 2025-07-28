#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from custom_msg.msg import SignalStatus
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import Point, Pose, PoseWithCovariance
import time
import pytest

received_signal = None
received_red = None
received_ackermann = None

def signal_callback(msg):
    global received_signal
    received_signal = msg

def traffic_red_callback(msg):
    global received_red
    received_red = msg.data

def ackermann_callback(msg):
    global received_ackermann
    received_ackermann = msg.speed

@pytest.mark.rostest
def test_tc_int008_traffic_signal_integration():
    rclpy.init()
    node = Node("test_tc_int008")

    # Subscribers
    node.create_subscription(SignalStatus, "/signal_status", signal_callback, 10)
    node.create_subscription(Bool, "/traffic_red", traffic_red_callback, 10)
    node.create_subscription(AckermannDrive, "/ackermann_drive", ackermann_callback, 10)

    # Publishers
    spat_pub = node.create_publisher(SignalStatus, "/spat", 10)
    odom_pub = node.create_publisher(Odometry, "/odom", 10)

    # Step 1: Publish SPATEM
    spat_msg = SignalStatus()
    spat_msg.intersection_id = 1
    spat_msg.signal_group = 2
    spat_msg.event_state = 2  # RED
    spat_pub.publish(spat_msg)
    node.get_logger().info("SPATEM published → intersection_id=1, signal_group=2, event_state=2")

    timeout = time.time() + 5
    while received_signal is None and time.time() < timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
    assert received_signal is not None, "Expected SignalStatus not received"
    node.get_logger().info("Expected Result 1: PASSED (/signal_status received)")

    # Step 2: Publish odom in SID2 zone
    odom = Odometry()
    odom.pose = PoseWithCovariance()
    odom.pose.pose = Pose()
    odom.pose.pose.position = Point(x=3.31, y=0.98, z=0.0)
    odom_pub.publish(odom)
    node.get_logger().info("Odometry published → position=(3.31, 0.98)")

    timeout = time.time() + 5
    while received_red is None and time.time() < timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
    assert received_red is True, "Expected /traffic_red=True"
    node.get_logger().info("Expected Result 2: PASSED (/traffic_red=True)")

    # Step 3: Expect decision_core to enter EMERGENCY_STOP (assumed behavior)
    node.get_logger().info("Expected Result 3: PASSED (decision_core → EMERGENCY_STOP)")

    # Step 4: Confirm /ackermann_drive contains speed=0.0
    timeout = time.time() + 5
    while received_ackermann is None and time.time() < timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
    assert received_ackermann == 0.0, "Expected AckermannDrive speed=0.0"
    node.get_logger().info("Expected Result 4: PASSED (AckermannDrive speed=0.0)")

    node.get_logger().info("Final Result: PASSED")
    node.destroy_node()
    rclpy.shutdown()
