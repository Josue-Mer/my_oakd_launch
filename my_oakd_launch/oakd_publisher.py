import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import depthai as dai
import cv2

class OakDPublisher(Node):
    def __init__(self):
        super().__init__('oakd_publisher')
        self.rgb_pub = self.create_publisher(Image, 'camera/color/image_raw', 10)
        self.depth_pub = self.create_publisher(Image, 'camera/depth/image_rect', 10)
        self.bridge = CvBridge()

        # Create pipeline
        pipeline = dai.Pipeline()

        # Define sources and outputs
        cam_rgb = pipeline.createColorCamera()
        cam_rgb.setBoardSocket(dai.CameraBoardSocket.RGB)
        cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        cam_rgb.setInterleaved(False)
        cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        cam_rgb.setFps(30)  # Ajusta la velocidad de fotogramas si es necesario

        xout_rgb = pipeline.createXLinkOut()
        xout_rgb.setStreamName("rgb")
        cam_rgb.video.link(xout_rgb.input)

        stereo = pipeline.createStereoDepth()
        stereo.setConfidenceThreshold(200)
        stereo.setLeftRightCheck(True)
        stereo.setSubpixel(True)
        stereo.setExtendedDisparity(False)
        stereo.setRectifyEdgeFillColor(0)  # Black, to better see the borders

        cam_left = pipeline.createMonoCamera()
        cam_left.setBoardSocket(dai.CameraBoardSocket.LEFT)
        cam_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        cam_left.setFps(30)  # Ajusta la velocidad de fotogramas si es necesario

        cam_right = pipeline.createMonoCamera()
        cam_right.setBoardSocket(dai.CameraBoardSocket.RIGHT)
        cam_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        cam_right.setFps(30)  # Ajusta la velocidad de fotogramas si es necesario

        cam_left.out.link(stereo.left)
        cam_right.out.link(stereo.right)

        xout_depth = pipeline.createXLinkOut()
        xout_depth.setStreamName("depth")
        stereo.depth.link(xout_depth.input)

        # Connect to device and start pipeline
        self.device = dai.Device(pipeline)
        self.q_rgb = self.device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
        self.q_depth = self.device.getOutputQueue(name="depth", maxSize=4, blocking=False)

        self.get_logger().info('La cámara inició bacanamente papa')

        self.timer = self.create_timer(0.1, self.publish_frames)

    def publish_frames(self):
        in_rgb = self.q_rgb.tryGet()
        in_depth = self.q_depth.tryGet()

        if in_rgb is not None:
            frame_rgb = in_rgb.getCvFrame()
            rgb_msg = self.bridge.cv2_to_imgmsg(frame_rgb, encoding="bgr8")
            self.rgb_pub.publish(rgb_msg)

        if in_depth is not None:
            frame_depth = in_depth.getCvFrame()
            frame_depth = cv2.normalize(frame_depth, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8UC1)
            depth_msg = self.bridge.cv2_to_imgmsg(frame_depth, encoding="mono8")
            self.depth_pub.publish(depth_msg)

def main(args=None):
    rclpy.init(args=args)
    node = OakDPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
