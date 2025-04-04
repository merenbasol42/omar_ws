#!/usr/bin/env python3
import time

from threading import Thread

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge
import numpy as np
import cv2

from .config.decompressor import *

class CameraDecompressorNode(Node):
    def __init__(self):
        super().__init__(NODE_NAME)
        # Sıkıştırılmış görüntüleri dinleyen abonelik oluşturuluyor
        self.subscription = self.create_subscription(
            CompressedImage,
            TOP_NAME_CAMERA_COMPRESSED,
            self.decompress_cb,
            5
        )
        # Decompress edilmiş görüntüleri yayınlamak için publisher
        self.publisher_ = self.create_publisher(Image, TOP_NAME_CAMERA_RAW, 10)
        self.bridge = CvBridge()

        self.msg: CompressedImage
        self.buffer = None
        self.pub_flag: bool = False
        self.sub_flag: bool = False

        Thread(target=self.decompress_loop, daemon=True).start()
        Thread(target=self.publish_loop, daemon=True).start()

    def decompress_loop(self):
        while True:
            if self.sub_flag:
                np_arr = np.frombuffer(self.msg.data, np.uint8)
                # JPEG formatındaki veriyi OpenCV formatına dönüştür
                self.buffer = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                if self.buffer is None:
                    self.get_logger().error("Görüntü çözülemedi!")
                    return

                self.sub_flag = False
                self.pub_flag = True

            else:
                time.sleep(0.005)     


    def publish_loop(self):
        while True:
            if self.pub_flag:
                try:
                    image_msg = self.bridge.cv2_to_imgmsg(self.buffer, "bgr8")
                    self.publisher_.publish(image_msg)
                    # self.get_logger().info("Decompress edilmiş görüntü yayınlandı.")
                except Exception as e:
                    self.get_logger().error(f"cv_bridge hatası: {e}")
        
                self.pub_flag = False
            
            else:
                time.sleep(0.005)     

    def decompress_cb(self, msg: CompressedImage):
        self.msg = msg
        self.sub_flag = True


def main(args=None):
    rclpy.init(args=args)
    node = CameraDecompressorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
