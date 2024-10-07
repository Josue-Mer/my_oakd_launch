#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import depthai as dai
import cv2

class OakDPublisher(Node):

    def __init__(self):
        super().__init__('oakd_publisher')

        # Initialize the pipeline and device
        # self.pipeline = dai.Pipeline()
        # self.device = dai.Device(self.pipeline)

        # Get camera information from Oak-D Lite
        self.rgb_camera_info = self.device.getCameraInfo(dai.CameraBoardSocket.RGB)
        self.depth_camera_info = self.device.getCameraInfo(dai.CameraBoardSocket.LEFT)

        # Initialize publishers
        self.rgb_publisher = self.create_publisher(Image, 'oakd/rgb/image_raw', 10)
        self.depth_publisher = self.create_publisher(Image, 'oakd/depth/image_raw', 10)
        self.color_info_pub = self.create_publisher(CameraInfo, '/camera/color/camera_info', 10)
        self.depth_info_pub = self.create_publisher(CameraInfo, '/camera/depth/camera_info', 10)
        
        # Use cv_bridge to convert between OpenCV images and ROS messages
        self.bridge = CvBridge()

        # Configure the DepthAI pipeline
        self.pipeline = dai.Pipeline()
        self.device.getCameraInfo(dai.ADatatype)
        # RGB camera node
        cam_rgb = self.pipeline.createColorCamera()
        cam_rgb.setBoardSocket(dai.CameraBoardSocket.RGB)
        cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        cam_rgb.setInterleaved(False)
        cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        xout_rgb = self.pipeline.createXLinkOut()
        xout_rgb.setStreamName("rgb")
        cam_rgb.video.link(xout_rgb.input)

        # Depth camera (Stereo node)
        mono_left = self.pipeline.createMonoCamera()
        mono_right = self.pipeline.createMonoCamera()
        stereo = self.pipeline.createStereoDepth()

        mono_left.setBoardSocket(dai.CameraBoardSocket.LEFT)
        mono_right.setBoardSocket(dai.CameraBoardSocket.RIGHT)
        stereo.initialConfig.setConfidenceThreshold(200)
        mono_left.out.link(stereo.left)
        mono_right.out.link(stereo.right)

        xout_depth = self.pipeline.createXLinkOut()
        xout_depth.setStreamName("depth")
        stereo.depth.link(xout_depth.input)

        # Start the device
        self.device = dai.Device(self.pipeline)

        # Start the image publishing loops
        self.timer = self.create_timer(1/30, self.publish_images)
        dai.CameraInfo

    def get_ros_camera_info(self, dai_camera_info):
        """Convert depthai.CameraInfo to ROS2 CameraInfo message."""
        camera_info_msg = CameraInfo()
        camera_info_msg.width = dai_camera_info.width
        camera_info_msg.height = dai_camera_info.height
        camera_info_msg.k = [
            dai_camera_info.intrinsicMatrix[0][0], dai_camera_info.intrinsicMatrix[0][1], dai_camera_info.intrinsicMatrix[0][2],
            dai_camera_info.intrinsicMatrix[1][0], dai_camera_info.intrinsicMatrix[1][1], dai_camera_info.intrinsicMatrix[1][2],
            dai_camera_info.intrinsicMatrix[2][0], dai_camera_info.intrinsicMatrix[2][1], dai_camera_info.intrinsicMatrix[2][2]
        ]
        camera_info_msg.d = dai_camera_info.distortionCoeff
        camera_info_msg.distortion_model = "plumb_bob"  # You can adjust this based on the actual model used
        return camera_info_msg
    

    def publish_images(self):
        # Get RGB and depth frames
        in_rgb = self.device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
        in_depth = self.device.getOutputQueue(name="depth", maxSize=4, blocking=False)

        rgb_frame = in_rgb.get().getCvFrame()
        depth_frame = in_depth.get().getCvFrame()

        # Publish RGB image
        if rgb_frame is not None:
            rgb_msg = self.bridge.cv2_to_imgmsg(rgb_frame, encoding="bgr8")
            self.rgb_publisher.publish(rgb_msg)
            # self.get_logger().info('Publishing RGB image')

        # Publish depth image
        if depth_frame is not None:
            depth_msg = self.bridge.cv2_to_imgmsg(depth_frame, encoding="mono16")
            self.depth_publisher.publish(depth_msg)
            # self.get_logger().info('Publishing depth image')

def main(args=None):
    rclpy.init(args=args)
    node = OakDPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
