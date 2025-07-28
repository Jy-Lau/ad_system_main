# test_srm_integration_chain.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
import pytest
import time

received_msgs = []

def srm_callback(msg):
    received_msgs.append(msg.data)

@pytest.mark.rostest
def test_tc_int008_decision_to_mqtt_chain():
    rclpy.init()
    node = Node("test_tc_int008")
    node.create_subscription(String, "/srm_request", srm_callback, 10)

    pub = node.create_publisher(Bool, "/emergency_detected", 10)

    # Step 1: Trigger emergency
    pub.publish(Bool(data=True))
    rclpy.spin_once(node, timeout_sec=1)
    time.sleep(5)
    assert "35333533" in received_msgs, "Expected emergency override not received"

    # Step 2: End emergency
    pub.publish(Bool(data=False))
    rclpy.spin_once(node, timeout_sec=1)
    time.sleep(3)
    assert "33333333" in received_msgs, "Expected clear override not received"

    node.destroy_node()
    rclpy.shutdown()
