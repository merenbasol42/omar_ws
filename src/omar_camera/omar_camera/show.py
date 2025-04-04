#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

from .config.show import *

class ImageViewerNode(Node):
    def __init__(self):
        super().__init__(NODE_NAME)
        # 'camera/image_raw' topic'inden Image mesajlarını dinle
        self.subscription = self.create_subscription(
            Image,
            TOP_NAME_CAMERA_RAW,
            self.image_callback,
            5
        )
        self.bridge = CvBridge()

    def image_callback(self, msg: Image):
        try:
            # ROS Image mesajını OpenCV formatına dönüştür (bgr8)
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Görüntü dönüşüm hatası: {e}")
            return

        # OpenCV penceresinde görüntüyü göster
        cv2.imshow("Image Viewer", cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ImageViewerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows() 
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
