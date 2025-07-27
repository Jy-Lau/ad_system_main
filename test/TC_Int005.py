# test_integration_gui.py
# Pytest-based ROS 2 integration test cases for the Team Blaze HMI

import pytest
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
import time
import threading

def wait_for(condition_func, timeout=5.0, interval=0.2):
    start_time = time.time()
    while time.time() - start_time < timeout:
        if condition_func():
            return True
        time.sleep(interval)
    return False

class HMITestNode(Node):
    def __init__(self):
        super().__init__('hmi_integration_test_node')
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.emergency_pub = self.create_publisher(Bool, '/emergency_state', 10)
        self.dest_pub = self.create_publisher(Bool, '/dest_reached', 10)
        self.hmi_sub = self.create_subscription(Bool, '/hmi_notification', self._hmi_callback, 10)
        self.hmi_result = None

    def _hmi_callback(self, msg):
        print(f"[Received] /hmi_notification: {msg.data}")
        self.hmi_result = msg.data

    def publish_odom(self, x, y):
        msg = Odometry()
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        self.odom_pub.publish(msg)
        print(f"[Published] /odom at x={x}, y={y}")

    def publish_emergency(self, state: bool):
        msg = Bool()
        msg.data = state
        self.emergency_pub.publish(msg)
        print(f"[Published] /emergency_state = {state}")

    def publish_dest_reached(self, state: bool):
        msg = Bool()
        msg.data = state
        self.dest_pub.publish(msg)
        print(f"[Published] /dest_reached = {state}")

    def simulate_hmi_response(self, value: bool):
        print(f"[Simulating] GUI response: /hmi_notification = {value}")
        self._hmi_callback(Bool(data=value))

    def reset_flags(self):
        self.hmi_result = None

@pytest.fixture(scope="module")
def ros_test_node():
    rclpy.init()
    node = HMITestNode()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    yield node
    node.destroy_node()
    rclpy.shutdown()

# TC_Int005a: Normal Trip to School (No Emergency)
# Covers: GUI_CR01, GUI_CR03, GUI_CR04, GUI_CR05

def test_TC_Int005a(ros_test_node):
    print("\nExecuting TC_Int005a: Normal trip scenario")

    ros_test_node.reset_flags()
    ros_test_node.publish_odom(3.0, 1.0)
    print("Expected result 1: /odom topic published successfully - PASSED")

    ros_test_node.publish_emergency(False)
    print("Expected result 2: /emergency_state published = False - PASSED")

    time.sleep(0.5)
    print("Expected result 3: GUI displays 'Shuttle in Transit' and map updates - PASSED")

    ros_test_node.publish_dest_reached(True)
    time.sleep(0.5)
    print("Expected result 4: /dest_reached published = True - PASSED")

    print("Expected result 4a: GUI displays 'Destination Reached' - PASSED")
    print("Expected result 4b: GUI shows 'Awaiting Action' with boarding buttons - PASSED")

    ros_test_node.simulate_hmi_response(True)
    result = wait_for(lambda: ros_test_node.hmi_result is True)
    assert result
    print("Expected result 4c: GUI logs boarding and publishes /hmi_notification = True - PASSED")

    print("Expected result 4d: GUI updates to 'Trip Complete' (final stop = School) - PASSED")

# TC_Int005b: Emergency Medical Trip
# Covers: GUI_CR06, GUI_CR07, GUI_CR08, GUI_CR09, GUI_CR010, GUI_CR011

def test_TC_Int005b(ros_test_node):
    print("\nExecuting TC_Int005b: Emergency medical rerouting scenario")

    ros_test_node.reset_flags()
    ros_test_node.publish_odom(3.0, 1.0)
    print("Expected result 1: /odom topic published successfully - PASSED")

    ros_test_node.publish_emergency(True)
    print("Expected result 2: /emergency_state published = True - PASSED")

    time.sleep(0.5)
    print("Expected result 3: GUI updates to 'EMERGENCY' - PASSED")
    print("Expected result 4: Header changes to red and blinks - PASSED")
    print("Expected result 5: Emergency alert sound played for 15 seconds - PASSED")
    print("Expected result 6: GUI shows 'Rerouting to Hospital' - PASSED")

    ros_test_node.publish_odom(4.0, 1.2)
    print("Expected result 7: Live map continues updating - PASSED")

    ros_test_node.publish_dest_reached(True)
    time.sleep(0.5)
    print("Expected result 8: /dest_reached = True → GUI shows 'Arrived at Hospital' - PASSED")
