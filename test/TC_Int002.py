import pytest
import rclpy
import time
import threading
import math
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, PoseStamped
from ackermann_msgs.msg import AckermannDrive
from custom_msg.msg import PathStage
from std_srvs.srv import SetBool
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class TC_Int002(Node):
    def __init__(self):
        super().__init__('TC_Int002')
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.ERROR)
        qos = QoSProfile(
    		depth=5,
    		reliability=ReliabilityPolicy.BEST_EFFORT,
    		history=HistoryPolicy.KEEP_LAST
		)
        
        # Publishers
        self.pub_scan = self.create_publisher(LaserScan, '/scan', qos)
        self.pub_odom = self.create_publisher(Odometry, '/odom', 10)
        self.pub_path = self.create_publisher(PathStage, '/path_stage', 10)
        
        # Internal state
        self.obstacle_detected = None
        self.last_cmd = None
        
        # Subscribers
        self.create_subscription(Bool, '/obstacle_detected', self.obstacle_callback, qos)
        self.create_subscription(AckermannDrive, '/ackermann_drive', self.cmd_callback, 10)

        # Service clients
        self.emergency_client = self.create_client(SetBool, '/emergency_stop')
        self.driving_client = self.create_client(SetBool, '/driving')

        # Wait for services
        while not self.emergency_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for emergency stop service...')
        while not self.driving_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for driving service...')

    def obstacle_callback(self, msg):
        self.obstacle_detected = msg.data
        self.get_logger().info(f"[TestNode] Obstacle detected: {msg.data}")

    def cmd_callback(self, msg):
        self.last_cmd = msg
        self.get_logger().info(f"[TestNode] Vehicle command: speed={msg.speed:.2f}, steering={msg.steering_angle:.2f}")

    def create_test_scan(self, distance=0.5):
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = "lidar_frame"
        scan.angle_min = -3.14
        scan.angle_max = 3.14
        scan.angle_increment = 0.01
        scan.range_min = 0.1
        scan.range_max = 10.0
        scan.ranges = [distance] * 628
        return scan

    def publish_obstacle(self, distance):
        scan = self.create_test_scan(distance)
        self.pub_scan.publish(scan)
        self.get_logger().info(f"[TestNode] Published obstacle at {distance}m")

    def mock_odometry(self, x=0.0, y=0.0, yaw=0.0):
        odom = Odometry()
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation.z = math.sin(yaw / 2)
        odom.pose.pose.orientation.w = math.cos(yaw / 2)
        odom.twist.twist.linear.x = 1.0
        return odom

    def mock_path_stage(self, waypoints=[(5.0, 0.0)]):
        path = PathStage()
        path.poses = [PoseStamped(pose=Pose(position=Point(x=wp[0], y=wp[1]))) for wp in waypoints]
        path.is_final_stage = False
        return path

    def call_emergency_stop(self, activate=True):
        req = SetBool.Request()
        req.data = activate
        return self.emergency_client.call_async(req)

    def call_driving_mode(self, activate=True):
        req = SetBool.Request()
        req.data = activate
        return self.driving_client.call_async(req)

def wait_for_condition(condition_fn, timeout=10.0, interval=0.1):
    start = time.time()
    while time.time() - start < timeout:
        if condition_fn():
            return True
        time.sleep(interval)
    return False

@pytest.fixture(scope='module')
def test_node():
    rclpy.init()
    node = TC_Int002()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    yield node
    executor.shutdown()
    thread.join(timeout=1.0)
    node.destroy_node()
    rclpy.shutdown()

def test_TC_Int002(test_node):
    # Test step 1: Deactivate emergency stop and activate driving mode
    test_node.call_emergency_stop(False)
    future = test_node.call_driving_mode(True)
    assert wait_for_condition(lambda: future.done(), timeout=5.0), "Failed to activate driving mode"
    print("Expected Result 1: PASSED")  # ← Expected Result 1


    # Test step 2: Simulate path and odometry by publishing path
    test_node.pub_odom.publish(test_node.mock_odometry())
    test_node.pub_path.publish(test_node.mock_path_stage([(5.0, 0.0), (10.0, 0.0)]))

    assert wait_for_condition(
        lambda: test_node.last_cmd is not None,
        timeout=3.0
    ), "Vehicle never sent any drive command"
    print("Expected Result 2: PASSED")  # ← Expected Result 2a


    # Test step 3: Simulate clear road (obstacle at 10m)
    test_node.publish_obstacle(10.0)

    assert wait_for_condition(
        lambda: test_node.obstacle_detected is False,
        timeout=5.0
    ), f"False positive obstacle detected: {test_node.obstacle_detected}"
    print("Expected Result 3a: PASSED")  # ← Expected Result 3a


    assert wait_for_condition(
        lambda: test_node.last_cmd is not None and test_node.last_cmd.speed > 0.0,
        timeout=10.0
    ), f"Vehicle did  continue moving: {test_node.last_cmd}"
    
    print("Expected Result 3b: PASSED")  # ← Expected Result 3b

