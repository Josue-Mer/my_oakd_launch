import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import Point
from tf_transformations import quaternion_from_euler
import logging

# Configuración de logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class GoalPoseFlagSender(Node):
    def __init__(self):
        super().__init__('goal_pose_flag_sender')
        
        # Acción para enviar goal pose
        self.goal_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Suscribirse al topic target_point
        self.subscription = self.create_subscription(
            Point,
            'target_point',
            self.target_point_callback,
            10
        )

        self.current_yaw = 0.0  # Placeholder for the yaw value

    def target_point_callback(self, msg):
        self.get_logger().info(f"Received target point: {msg}")
        self.send_goal_pose(msg)

    def send_goal_pose(self, target_point):
        try:
            # Convert the Euler angle to a quaternion
            quaternion = quaternion_from_euler(0, 0, self.current_yaw)

            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.pose.position.x = target_point.x
            goal_msg.pose.pose.position.y = target_point.y
            goal_msg.pose.pose.position.z = 0.0  # Assuming 2D navigation

            goal_msg.pose.pose.orientation.x = quaternion[0]
            goal_msg.pose.pose.orientation.y = quaternion[1]
            goal_msg.pose.pose.orientation.z = quaternion[2]
            goal_msg.pose.pose.orientation.w = quaternion[3]

            self.goal_client.wait_for_server()
            self.goal_client.send_goal_async(goal_msg)
            logger.info(f"Automatic goal pose sent: x={target_point.x}, y={target_point.y}, yaw={self.current_yaw:.2f}")
        except Exception as e:
            logger.error(f"An error occurred while sending goal: {e}")

    def show_command_options(self):
        while True:
            command = input("Enter command (p: save current location, s: send saved goal pose, q: quit): ").strip().lower()
            if command == 's':
                self.send_goal_pose()
            elif command == 'q':
                self.shutdown_node()
                break
    
    def shutdown_node(self):
        logger.info("Shutting down...")
        self.destroy_node()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = GoalPoseFlagSender()
    
    try:
        # node.show_command_options()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down.")
    finally:
        node.shutdown_node()

if __name__ == '__main__':
    main()
######################################################################################################################
# import rclpy
# from rclpy.node import Node
# from rclpy.action import ActionClient
# from nav2_msgs.action import NavigateToPose
# from geometry_msgs.msg import Point
# from tf_transformations import quaternion_from_euler
# import logging

# # Configuración de logging
# logging.basicConfig(level=logging.INFO)
# logger = logging.getLogger(__name__)

# class GoalPoseFlagSender(Node):
#     def __init__(self):
#         super().__init__('goal_pose_flag_sender')
        
#         # Acción para enviar goal pose
#         self.goal_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
#         # Suscribirse al topic target_point
#         self.subscription = self.create_subscription(
#             Point,
#             'target_point',
#             self.target_point_callback,
#             10
#         )

#         self.saved_x = None
#         self.saved_y = None
#         self.saved_yaw = 0.0  # Placeholder for the yaw value

#     def target_point_callback(self, msg):
#         self.get_logger().info(f"Received target point: {msg}")
#         self.saved_x = msg.x
#         self.saved_y = msg.y
#         self.saved_yaw = 0.0  # Update this with the actual yaw value if available
#         self.get_logger().info(f"Saved location: x={self.saved_x}, y={self.saved_y}, yaw={self.saved_yaw}")

#     def send_goal_pose(self):
#         if self.saved_x is not None and self.saved_y is not None:
#             try:
#                 # Convert the Euler angle to a quaternion
#                 quaternion = quaternion_from_euler(0, 0, self.saved_yaw)

#                 goal_msg = NavigateToPose.Goal()
#                 goal_msg.pose.pose.position.x = self.saved_x
#                 goal_msg.pose.pose.position.y = self.saved_y
#                 goal_msg.pose.pose.position.z = 0.0  # Assuming 2D navigation

#                 goal_msg.pose.pose.orientation.x = quaternion[0]
#                 goal_msg.pose.pose.orientation.y = quaternion[1]
#                 goal_msg.pose.pose.orientation.z = quaternion[2]
#                 goal_msg.pose.pose.orientation.w = quaternion[3]

#                 self.goal_client.wait_for_server()
#                 send_goal_future = self.goal_client.send_goal_async(goal_msg)
#                 send_goal_future.add_done_callback(self.goal_response_callback)
#                 logger.info(f"Goal pose sent: x={self.saved_x}, y={self.saved_y}, yaw={self.saved_yaw:.2f}")
#             except Exception as e:
#                 logger.error(f"An error occurred while sending goal: {e}")
#         else:
#             logger.error("No saved flag location to send goal pose.")

#     def goal_response_callback(self, future):
#         goal_handle = future.result()
#         if not goal_handle.accepted:
#             logger.error('Goal rejected')
#             return

#         logger.info('Goal accepted')
#         get_result_future = goal_handle.get_result_async()
#         get_result_future.add_done_callback(self.get_result_callback)

#     def get_result_callback(self, future):
#         result = future.result().result
#         logger.info(f'Goal result received: {result}')

#     def show_command_options(self):
#         while True:
#             command = input("Enter command (s: send saved goal pose, q: quit): ").strip().lower()
#             if command == 's':
#                 self.send_goal_pose()
#             elif command == 'q':
#                 self.shutdown_node()
#             elif command == 'p':
#                 self.save_goal_pose()
#                 break

#     def shutdown_node(self):
#         logger.info("Shutting down...")
#         self.destroy_node()
#         rclpy.shutdown()

# def main(args=None):
#     rclpy.init(args=args)
#     node = GoalPoseFlagSender()
    
#     try:
#         node.show_command_options()
#     except KeyboardInterrupt:
#         node.get_logger().info("Keyboard interrupt, shutting down.")
#     finally:
#         node.shutdown_node()

# if __name__ == '__main__':
#     main()

# import rclpy
# from rclpy.node import Node
# from rclpy.action import ActionClient
# from nav2_msgs.action import NavigateToPose
# from geometry_msgs.msg import Point
# from tf_transformations import quaternion_from_euler
# import logging

# # Configuración de logging
# logging.basicConfig(level=logging.INFO)
# logger = logging.getLogger(__name__)

# class GoalPoseFlagSender(Node):
#     def __init__(self):
#         super().__init__('goal_pose_flag_sender')
        
#         # Acción para enviar goal pose
#         self.goal_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
#         # Suscribirse al topic target_point
#         self.subscription = self.create_subscription(
#             Point,
#             'target_point',
#             self.target_point_callback,
#             10
#         )

#         self.current_x = None
#         self.current_y = None
#         self.current_yaw = 0.0  # Placeholder for the yaw value

#         self.saved_x = None
#         self.saved_y = None
#         self.saved_yaw = 0.0

#     def target_point_callback(self, msg):
#         self.get_logger().info(f"Received target point: {msg}")
#         self.current_x = msg.x
#         self.current_y = msg.y
#         self.current_yaw = 0.0  # Update this with the actual yaw value if available
#         self.get_logger().info(f"Current location: x={self.current_x}, y={self.current_y}, yaw={self.current_yaw}")

#     def save_goal_pose(self, target_point):
#         if self.current_x is not None and self.current_y is not None:
#             self.saved_x = target_point.x
#             self.saved_y = target_point.y
#             self.saved_yaw = self.current_yaw
#             logger.info(f"Saved location: x={self.saved_x}, y={self.saved_y}, yaw={self.saved_yaw:.2f}")
#         else:
#             logger.error("No current flag location to save.")

#     def send_goal_pose(self):
#         if self.saved_x is not None and self.saved_y is not None:
#             try:
#                 # Convert the Euler angle to a quaternion
#                 quaternion = quaternion_from_euler(0, 0, self.saved_yaw)

#                 goal_msg = NavigateToPose.Goal()
#                 goal_msg.pose.pose.position.x = self.saved_x
#                 goal_msg.pose.pose.position.y = self.saved_y
#                 goal_msg.pose.pose.position.z = 0.0  # Assuming 2D navigation

#                 goal_msg.pose.pose.orientation.x = quaternion[0]
#                 goal_msg.pose.pose.orientation.y = quaternion[1]
#                 goal_msg.pose.pose.orientation.z = quaternion[2]
#                 goal_msg.pose.pose.orientation.w = quaternion[3]

#                 self.goal_client.wait_for_server()
#                 send_goal_future = self.goal_client.send_goal_async(goal_msg)
#                 send_goal_future.add_done_callback(self.goal_response_callback)
#                 logger.info(f"Goal pose sent: x={self.saved_x}, y={self.saved_y}, yaw={self.saved_yaw:.2f}")
#             except Exception as e:
#                 logger.error(f"An error occurred while sending goal: {e}")
#         else:
#             logger.error("No saved flag location to send goal pose.")

#     def goal_response_callback(self, future):
#         goal_handle = future.result()
#         if not goal_handle.accepted:
#             logger.error('Goal rejected')
#             return

#         logger.info('Goal accepted')
#         get_result_future = goal_handle.get_result_async()
#         get_result_future.add_done_callback(self.get_result_callback)

#     def get_result_callback(self, future):
#         result = future.result().result
#         logger.info(f'Goal result received: {result}')

#     def show_command_options(self):
#         while True:
#             command = input("Enter command (p: save current location, s: send saved goal pose, q: quit): ").strip().lower()
#             if command == 'p':
#                 self.save_goal_pose()
#             elif command == 's':
#                 self.send_goal_pose()
#             elif command == 'q':
#                 self.shutdown_node()
#                 break

#     def shutdown_node(self):
#         logger.info("Shutting down...")
#         self.destroy_node()
#         rclpy.shutdown()

# def main(args=None):
#     rclpy.init(args=args)
#     node = GoalPoseFlagSender()
    
#     try:
#         node.show_command_options()
#     except KeyboardInterrupt:
#         node.get_logger().info("Keyboard interrupt, shutting down.")
#     finally:
#         node.shutdown_node()

# if __name__ == '__main__':
#     main()
