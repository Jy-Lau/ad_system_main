import pytest
import rclpy
import time
import threading
from rclpy.node import Node
from std_msgs.msg import Bool
from custom_msg.msg import PathStage
from std_srvs.srv import SetBool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import  Point, Quaternion
from ackermann_msgs.msg import AckermannDrive
from rclpy.executors import SingleThreadedExecutor
import math

class TC_Int003(Node):
    def __init__(self):
        super().__init__('TC_Int003')
        self.emergency_state = False
        self.last_cmd = None
        self.path_stage = None
        self.ackermann_speed = 0.0
        # Subscribers for verification
        self.create_subscription(AckermannDrive, '/ackermann_drive', self.cmd_cb, 10)
        self.create_subscription(Bool, '/emergency_state', self.emergency_state_cb, 10)
        self.create_subscription(PathStage, '/path_stage', self.path_stage_cb, 10)
        self.create_subscription(AckermannDrive, '/ackermann_drive', self.ackermann_cb, 10)

        # Publisher for /odom topic
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # Create client to call /driving service
        self.drive_client = self.create_client(SetBool, '/driving')
        while not self.drive_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /driving service...")

    def emergency_state_cb(self, msg):
        self.emergency_state = msg.data

    def cmd_cb(self, msg):
        self.last_cmd = msg
        self.get_logger().info(f"Received AckermannDrive with speed={msg.speed}")

    def call_driving(self):
        req = SetBool.Request()
        req.data = True
        return self.drive_client.call_async(req)
    
    def path_stage_cb(self, msg):
        self.path_stage=msg

    def ackermann_cb(self, msg):
        self.ackermann_speed = msg.speed

    def publish_odom(self, x=2.3, y=5.0):
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'

        # Set pose (position + orientation)
        odom_msg.pose.pose.position = Point(x=x, y=y, z=0.0)
        odom_msg.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        # Publish the message
        self.odom_pub.publish(odom_msg)
        self.get_logger().info(f"Published Odometry with position x={x}, y={y}")

def wait_for(condition_fn, timeout=10.0, interval=0.1):
    start = time.time()
    while time.time() - start < timeout:
        if condition_fn():
            return True
        time.sleep(interval)
    return False

@pytest.fixture(scope='module')
def test_node():
    rclpy.init()
    node = TC_Int003()
    exe = SingleThreadedExecutor()
    exe.add_node(node)
    thread = threading.Thread(target=exe.spin, daemon=True)
    thread.start()
    yield node
    exe.shutdown()
    thread.join(timeout=1.0)
    node.destroy_node()
    rclpy.shutdown()

def test_TC_Int003(test_node):
    # Test step 1: Publish custom /odom message with x=2.3, y=5 before starting driving
    test_node.publish_odom(x=2.3, y=5.0)

    # Test step 2: Call the driving service to start the mission
    fut = test_node.call_driving()
    wait_for(lambda: fut.done(), timeout=5.0), "Driving service call did not complete"        
    # Expected result 2a
    assert not test_node.emergency_state, "/emergency_state not correctly published as False"
    print("Expected result 2a: PASSED")
    # Expected result 2b
    assert math.isclose(test_node.path_stage.poses[-1].pose.position.x, 5.8, rel_tol=1e-2) and \
        math.isclose(test_node.path_stage.poses[-1].pose.position.y, 0.48, rel_tol=1e-2) and \
        not test_node.path_stage.is_final_stage, "/path_stage not planned correctly (x, y, or is_final_stage wrong)"
    print("Expected result 2b: PASSED")
    # Expected result 2c
    assert test_node.ackermann_speed > 0.0, "/ackermann_drive speed not greater than 0.0"
    print("Expected result 2c: PASSED")
