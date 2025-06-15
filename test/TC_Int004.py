import pytest
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from custom_msg.msg import EmotionLabel, PoseLabel, PathStage
from nav_msgs.msg import Odometry
from std_srvs.srv import SetBool
from ackermann_msgs.msg import AckermannDrive
import time
import threading
from rclpy.executors import SingleThreadedExecutor
import math

class TC_Int004(Node):
    def __init__(self):
        super().__init__('TC_Int004')
        # Test verification flags
        self.emergency_detected = False
        self.emergency_state = False
        self.path_stage = None
        self.ackermann_speed = 0.0
        
        # Publishers for test inputs
        self.pub_emergency = self.create_publisher(Bool, '/emergency_detected', 10)
        self.pub_odom = self.create_publisher(Odometry, '/odom', 10)
        
        # Service client for driving state
        self.driving_client = self.create_client(SetBool, '/driving')
        
        # Subscribers for verification
        self.create_subscription(Bool, '/emergency_detected', self.emergency_detected_cb, 10)
        self.create_subscription(Bool, '/emergency_state', self.emergency_state_cb, 10)
        self.create_subscription(PathStage, '/path_stage', self.path_stage_cb, 10)
        self.create_subscription(AckermannDrive, '/ackermann_drive', self.ackermann_cb, 10)
        
        # Test sequence timer
        self.test_completed = False
        self.test_step = 1
        self.timer = self.create_timer(0.5, self.run_test_sequence)

    def emergency_detected_cb(self, msg):
        self.emergency_detected = msg.data

    def emergency_state_cb(self, msg):
        self.emergency_state = msg.data

    def path_stage_cb(self, msg):
        self.path_stage=msg

    def ackermann_cb(self, msg):
        self.ackermann_speed = msg.speed

    def run_test_sequence(self):
        if self.test_step == 1: # Test step 1
            self.set_initial_position()
            self.test_step = 2

        elif self.test_step == 2: # Test step 2
            self.set_driving_state()

            if self.path_stage is not None and not self.emergency_state and self.ackermann_speed > 0.0:
                # Expected result 2a
                assert not self.emergency_state, "/emergency_state not correctly published as False"
                print("Expected result 2a: PASSED")
                # Expected result 2b
                assert math.isclose(self.path_stage.poses[-1].pose.position.x, 5.8, rel_tol=1e-2) and \
                    math.isclose(self.path_stage.poses[-1].pose.position.y, 0.48, rel_tol=1e-2) and \
                    not self.path_stage.is_final_stage, "/path_stage not planned correctly (x, y, or is_final_stage wrong)"
                print("Expected result 2b: PASSED")
                # Expected result 2c
                assert self.ackermann_speed > 0.0, "/ackermann_drive speed not greater than 0.0"
                print("Expected result 2c: PASSED")
                self.test_step = 3

        elif self.test_step == 3: # Test step 3
            self.publish_emergency_condition()

            if self.path_stage is not None and self.emergency_state and self.ackermann_speed > 0.0:
                # Expected result 3a
                assert self.emergency_state, "/emergency_state not correctly published as True"
                print("Expected result 3a: PASSED") 
                # Expected result 3b
                assert math.isclose(self.path_stage.poses[-1].pose.position.x, 4.0, rel_tol=1e-2) and \
                    math.isclose(self.path_stage.poses[-1].pose.position.y, 7.52, rel_tol=1e-2) and \
                    self.path_stage.is_final_stage, "/path_stage not replanned correctly (x, y, or is_final_stage wrong)"
                print("Expected result 3b: PASSED")
                # Expected result 3c
                assert self.ackermann_speed > 0.0, "/ackermann_drive speed not greater than 0.0"
                print("Expected result 3c: PASSED")

                self.timer.cancel()
                self.test_completed = True

    def set_initial_position(self):
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'map'
        odom.pose.pose.position.x = 7.21
        odom.pose.pose.position.y = 2.4
        self.pub_odom.publish(odom)

    def set_driving_state(self):
        while not self.driving_client.wait_for_service(timeout_sec=2.0):
            print('Waiting for /driving service...')
            
        req = SetBool.Request()
        req.data = True
        future = self.driving_client.call_async(req)

    def publish_emergency_condition(self):
        msg = Bool(data=True)
        self.pub_emergency.publish(msg)

@pytest.fixture(scope="module")
def test_node():
    rclpy.init()
    node = TC_Int004()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    
    # Run executor in background thread
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    
    yield node
    
    # Cleanup
    executor.shutdown()
    thread.join(timeout=1.0)
    node.destroy_node()
    rclpy.shutdown()

def test_TC_Int004(test_node):
    # Wait for test to complete (timeout 10 seconds)
    start_time = time.time()
    while not test_node.test_completed and time.time() - start_time < 10:
        time.sleep(0.1)
    
    assert test_node.test_completed, "Test did not complete within timeout"