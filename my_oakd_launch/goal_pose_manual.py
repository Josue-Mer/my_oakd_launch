import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from tf_transformations import quaternion_from_euler
import configparser
import logging

# Configuración
config = configparser.ConfigParser()
try:
    config.read('config.ini')
    LOG_LEVEL = config['DEFAULT'].get('LogLevel', 'INFO')
except Exception as e:
    print(f"Error reading configuration: {e}")
    LOG_LEVEL = 'INFO'

# Configuración de logging
logging.basicConfig(level=LOG_LEVEL)
logger = logging.getLogger(__name__)

class ManualGoalSender(Node):

    def __init__(self):
        super().__init__('manual_goal_sender')
        
        # Acción para enviar goal pose
        self.goal_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def show_command_options(self):
        while True:
            command = input("Enter command (s: send goal pose, q: quit): ").strip().lower()
            if command == 's':
                self.manual_goal_pose()
            elif command == 'q':
                self.shutdown_node()
                break

    def manual_goal_pose(self):
        try:
            x = float(input("Enter x position: "))
            y = float(input("Enter y position: "))
            angle = float(input("Enter angle in radians: "))
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.pose.position.x = x
            goal_msg.pose.pose.position.y = y
            goal_msg.pose.pose.position.z = 0.0  # Assuming 2D navigation
            quaternion = quaternion_from_euler(0, 0, angle)
            goal_msg.pose.pose.orientation.x = quaternion[0]
            goal_msg.pose.pose.orientation.y = quaternion[1]
            goal_msg.pose.pose.orientation.z = quaternion[2]    
            goal_msg.pose.pose.orientation.w = quaternion[3]
            self.goal_client.wait_for_server()
            self.goal_client.send_goal_async(goal_msg)
            logger.info(f"Manual goal pose sent: x={x}, y={y}, angle={angle:.2f}")
        except ValueError:
            logger.error("Invalid input. Please enter numeric values for x, y, and angle.")

    def shutdown_node(self):
        logger.info("Shutting down...")
        self.destroy_node()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = ManualGoalSender()
    
    try:
        node.show_command_options()
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()