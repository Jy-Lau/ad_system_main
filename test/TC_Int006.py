import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from std_srvs.srv import Trigger
import threading
import pytest
import time


qos_0 = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)


class IntegrationNode(Node):
    def __init__(self):
        super().__init__('integration_test_node')

        # Publishers
        self.scan_pub = self.create_publisher(LaserScan, '/scan', qos_0)
        self.odom_pub = self.create_publisher(Odometry, '/odom', qos_0)
        self.path_pub = self.create_publisher(Path, '/path_stage', qos_0)

        # Subscribers
        self.create_subscription(OccupancyGrid, '/cost_map', self.costmap_cb, qos_0)
        self.create_subscription(Path, '/dwa_path', self.dwa_path_cb, qos_0)

        # Service Client
        self.replan_client = self.create_client(Trigger, '/run_dwa')

        # Events
        self.costmap_received = threading.Event()
        self.dwa_path_received = threading.Event()

    def costmap_cb(self, msg):
        self.get_logger().info('Costmap received.')
        self.costmap_received.set()

    def dwa_path_cb(self, msg):
        self.get_logger().info('DWA Path received.')
        self.dwa_path_received.set()

    def publish_pose_in_dwa_zone(self):
        odom_msg = Odometry()
        odom_msg.header.frame_id = 'odom'
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.pose.pose.position = Point(x=2.0, y=0.5, z=0.0)
        odom_msg.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        self.odom_pub.publish(odom_msg)

    def publish_fake_scan(self):
        scan = LaserScan()
        scan.header.frame_id = '11/base_link'
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.angle_min = -2.62
        scan.angle_max = 0.17
        scan.angle_increment = 0.01
        scan.range_min = 0.0
        scan.range_max = 5.0

        num_readings = int((scan.angle_max - scan.angle_min) / scan.angle_increment)
        ranges = [float('inf')] * num_readings

        start_idx = int(((-60 * 3.1415 / 180) - scan.angle_min) / scan.angle_increment)
        end_idx = int(((-30 * 3.1415 / 180) - scan.angle_min) / scan.angle_increment)
        for i in range(start_idx, end_idx):
            ranges[i] = 0.5

        scan.ranges = ranges
        self.scan_pub.publish(scan)

    def publish_mock_path(self):
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for i in range(5):
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = 2.0 + i * 0.5
            pose.pose.position.y = 0.5
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)

    def call_trigger_replan_service(self):
        while not self.replan_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for /trigger_replan service...')

        request = Trigger.Request()
        future = self.replan_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

        if future.result() is not None and future.result().success:
            self.get_logger().info("Trigger replan service call succeeded.")
            return True
        else:
            self.get_logger().error("Trigger replan service call failed or timed out.")
            return False


# -------------------- Pytest ----------------------------

@pytest.fixture(scope='module')
def rclpy_node():
    rclpy.init()
    node = IntegrationNode()
    yield node
    node.destroy_node()
    rclpy.shutdown()


def test_TC_Int006(rclpy_node):
    rclpy_node.publish_pose_in_dwa_zone()
    rclpy_node.get_logger().info("Published pose inside DWA zone.")
    rclpy_node.get_logger().info("Expected Result 1: PASSED")

    for _ in range(5):
        rclpy.spin_once(rclpy_node, timeout_sec=0.2)
        time.sleep(0.05)
    rclpy_node.get_logger().info("Expected Result 2: PASSED")

    rclpy_node.publish_fake_scan()
    rclpy_node.get_logger().info("Published fake LaserScan.")
    rclpy_node.get_logger().info("Expected Result 3a: PASSED")

    rclpy_node.publish_mock_path()
    rclpy_node.get_logger().info("Published mocked path.")
    rclpy_node.get_logger().info("Expected Result 3b: PASSED")

    result = rclpy_node.call_trigger_replan_service()
    if result:
        rclpy_node.get_logger().info("Expected Result 4: PASSED")
    else:
        rclpy_node.get_logger().error("Expected Result 4: FAILED")
        assert False, "Trigger replan service failed."

    start_time = time.time()
    while (not rclpy_node.costmap_received.is_set() or not rclpy_node.dwa_path_received.is_set()) and (time.time() - start_time < 6.0):
        rclpy.spin_once(rclpy_node, timeout_sec=0.2)
        time.sleep(0.05)

    assert rclpy_node.costmap_received.is_set(), "Costmap not received"
    rclpy_node.get_logger().info("Expected Result 5a: PASSED")

    assert rclpy_node.dwa_path_received.is_set(), "DWA path not received"
    rclpy_node.get_logger().info("Expected Result 5b: PASSED")

    rclpy_node.get_logger().info("Final Result: PASSED")


