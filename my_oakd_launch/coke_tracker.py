import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from tf2_ros import TransformListener, Buffer
from cv_bridge import CvBridge
import cv2
from std_msgs.msg import String
import numpy as np
import math
import time # Importing the time module for sleep functionality
from collections import deque

# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# import cv2
# from cv_bridge import CvBridge
# import numpy as np
# from tf2_ros import TransformListener, Buffer
# from geometry_msgs.msg import PoseStamped
# from nav2_msgs.action import NavigateToPose
# from rclpy.action import ActionClient
# import time  # Importing the time module for sleep functionality

# Constants
CAMERA_OFFSET_X = -0.1  # Camera offset in meters
TARGET_POINT_TOPIC = "/target_point"

# Global variables
odom_position = None
odom_orientation = None
x_value = None
y_value = None
z_value = None

def quaternion_to_euler(x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return roll_x, pitch_y, yaw_z



class CokeTracker(Node):
    def __init__(self):
        super().__init__('coke_tracker')
        # Initialize deque to store recent measurements
        self.position_deque = deque(maxlen=10)  # Change maxlen as needed
        self.orientation_deque = deque(maxlen=10)
        
        # Timer to check for data arrival
        self.timer = self.create_timer(0.1, self.check_data_arrival)
        self.last_data_time = self.get_clock().now()
        self.create_subscription(Odometry, "/odom", self.odom_callback, 10)
        self.create_subscription(String, "/flagdata", self.flagdata_callback, 10)
        self.subscription = self.create_subscription(Point, TARGET_POINT_TOPIC, self.process_flag_data, 10)
        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.saved_pose = None
        self.position_logged = False
        self.goal_sent = False
        self.origin_pose = None  # To store the origin position

        # Initialize the origin position
        self.get_initial_pose()

        self.get_logger().info("CokeTracker node has been started.")

    def get_initial_pose(self):
        try:
            self.get_logger().info("Waiting for the 'map' frame to become available...")
            while not self.tf_buffer.can_transform('map', 'base_link', rclpy.time.Time()):
                self.get_logger().warn("Cannot transform 'map' to 'base_link' yet. Waiting...")
                rclpy.spin_once(self, timeout_sec=1.0)
            
            now = rclpy.time.Time()
            transform = self.tf_buffer.lookup_transform('map', 'base_link', now)
            self.origin_pose = PoseStamped()
            self.origin_pose.header.stamp = transform.header.stamp
            self.origin_pose.header.frame_id = 'map'
            self.origin_pose.pose.position.x = transform.transform.translation.x
            self.origin_pose.pose.position.y = transform.transform.translation.y
            self.origin_pose.pose.position.z = transform.transform.translation.z
            self.origin_pose.pose.orientation = transform.transform.rotation
            self.get_logger().info(f"Origin position saved: {self.origin_pose.pose.position}")
        except Exception as e:
            self.get_logger().error(f"Failed to get initial transform: {e}")
    
    
    def odom_callback(self, msg):
        global odom_position, odom_orientation
        odom_position = msg.pose.pose.position
        odom_orientation = msg.pose.pose.orientation

    # def flagdata_callback(self, msg):
    #     global x_value, y_value, z_value
    #     data = msg.data.split(';')
    #     for point in data:
    #         if point:
    #             x_value, y_value, z_value = map(float, point.replace("X:", "").replace("Y:", "").replace("Z:", "").split(','))
    #             # self.get_logger().info("X: "+String(x_value / 1000.0)", Y: "+String(y_value / 1000.0)", Z: "+String(z_value / 1000.0))  # Convert mm to meters
    #             self.get_logger().info("Esta")
    #     time.sleep(10)  # Wait for 10 seconds before saving position
    #     self.process_flag_data()

    def flagdata_callback(self, msg):
        global x_value, y_value, z_value
        data = msg.data.split(';')
        for point in data:
            if point:
                x_value, y_value, z_value = map(float, point.replace("X:", "").replace("Y:", "").replace("Z:", "").split(','))
                
                # Store the position in the deque
                self.position_deque.append((x_value, y_value, z_value))
                
                # Assuming yaw calculation is required, calculate yaw
                yaw = np.arctan2(y_value, x_value)  # Simplified example, replace with real yaw calculation
                self.orientation_deque.append(yaw)
                
                # Update the time of the last received data
                self.last_data_time = self.get_clock().now()

    def check_data_arrival(self):
        # Check if no data has arrived in the last second
        current_time = self.get_clock().now()
        if (current_time - self.last_data_time).nanoseconds > 1e9:
            # Calculate the moving average
            if self.position_deque and self.orientation_deque:
                mean_position = np.mean(self.position_deque, axis=0)
                mean_yaw = np.mean(self.orientation_deque)

                self.get_logger().info(f"Saved mean position: {mean_position}")
                
                # Now process this averaged data as needed
                self.process_flag_data(mean_position, mean_yaw)
                
                # Clear the deques after processing
                self.position_deque.clear()
                self.orientation_deque.clear()
    

    def process_flag_data(self, mean_position, mean_yaw):
        global x_value, y_value, z_value

        try:
            self.get_logger().info("Coke reached, saving robot position...")
            try:
                now = rclpy.time.Time()
                transform = self.tf_buffer.lookup_transform('map', 'base_link', now)

                # roll, pitch, yaw = quaternion_to_euler(
                #     odom_orientation.x, odom_orientation.y, odom_orientation.z, odom_orientation.w
                # )

                # Adjust the point from the camera frame to the robot frame
                camera_x = -mean_position[2] / 1000.0  # Camera z is robot x
                camera_y = mean_position[0] / 1000.0  # Camera x is negative robot y
                camera_x += CAMERA_OFFSET_X
                
                # * math.cos(yaw) - camera_y * math.sin(yaw)
                # * math.sin(yaw) + camera_y * math.cos(yaw)
                self.saved_pose = PoseStamped()
                self.saved_pose.header.stamp = transform.header.stamp
                self.saved_pose.header.frame_id = 'map'
                self.saved_pose.pose.position.x = transform.transform.translation.x - camera_x 
                self.saved_pose.pose.position.y = transform.transform.translation.y - camera_y 
                self.saved_pose.pose.position.z = transform.transform.translation.z
                self.saved_pose.pose.orientation = transform.transform.rotation

                # self.get_logger().info(f"Processed averaged data: {mean_position}, mean yaw: {mean_yaw:.2f}")
                
                self.get_logger().info(f"Saved robot position: {self.saved_pose.pose.position}")
                self.position_logged = True 

                # Close the OpenCV window
                cv2.destroyAllWindows()

                # Prompt user to send coordinates to Nav2
                self.prompt_user_to_send_goal()
            except Exception as e:
                self.get_logger().error(f"Failed to get transform: {e}")
                
        except Exception as e:
            self.get_logger().error(f"Failed to process image: {e}")

    def prompt_user_to_send_goal(self):
        print("Ready to send the coordinates to Nav2? (y/n)")
        user_input = input().strip().lower()
        if user_input == 'y':
            self.send_goal(self.saved_pose, self.goal_reached_callback)

    def send_goal(self, pose, callback):
        if pose and not self.goal_sent:
            self.get_logger().info("Sending goal to Nav2...")
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = pose

            self.action_client.wait_for_server()
            future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
            future.add_done_callback(callback)
            self.goal_sent = True

    def goal_reached_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal was rejected by Nav2.')
            return

        self.get_logger().info('Goal accepted by Nav2. Waiting for result...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.return_to_origin)

    def return_to_origin(self, future):
        result = future.result()
        if result.status == 3:  # SUCCEEDED
            self.get_logger().info('Goal reached, waiting before returning to origin...')
            time.sleep(5)  # Wait for 5 seconds before returning
            self.get_logger().info('Returning to origin...')
            self.send_goal(self.origin_pose, self.origin_reached_callback)

    def origin_reached_callback(self, future):
        self.get_logger().info('Robot has returned to the origin.')

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Feedback: current position x={feedback.current_pose.pose.position.x} y={feedback.current_pose.pose.position.y}')

def main(args=None):
    rclpy.init(args=args)
    node = CokeTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()