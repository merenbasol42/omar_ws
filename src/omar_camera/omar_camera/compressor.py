#!/usr/bin/env python3
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from omar_interfaces.msg import CameraPerformance
import cv2
import numpy as np

from threading import Thread

from .config.compressor import *

class CompressorNode(Node):
    def __init__(self):
        super().__init__(NODE_NAME)
        self.img_pubber = self.create_publisher(CompressedImage, TOP_NAME_CAMERA_COMPRESSED, 10)
        self.performance_pubber = self.create_publisher(CameraPerformance, TOP_NAME_CAMERA_PERFORMANCE, 5)

        self.last_extract = time.time()
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("Dahili kamera açılmıyor!")
            exit()

        self.last_performance_calc: float = time.time()
        self.extract_count: int = 0
        self.pub_count: int = 0

        self.buffer = None
        self.flag: bool = False
        
        self.msg = CompressedImage()
        self.msg.header.frame_id = FRAME_ID
        self.msg.format = "jpeg"  # veya "jpeg; compressed" gibi format belirtebilirsin

        self.performance_msg = CameraPerformance()

        self.get_logger().info(f"HZ = {HZ}")

        self.create_timer(5.0, self.pub_performance)
        Thread(target=self.extract_loop, daemon=True).start()
        Thread(target=self.publish_loop, daemon=True).start()
    
    def pub_performance(self):
        dt = time.time() - self.last_performance_calc
        self.performance_msg.extract_fps = self.extract_count / dt
        self.performance_msg.publish_fps = self.pub_count / dt
        
        self.extract_count = 0
        self.pub_count = 0
        self.last_performance_calc = time.time()
        
        self.performance_pubber.publish(self.performance_msg)

    def extract_loop(self):    
        interval: float = 0.0
        while True:
            interval = time.time() - self.last_extract
            if interval > HZ:
                ret, frame = self.cap.read()
                if not ret:
                    self.get_logger().warning("Kameradan görüntü alınamadı!")
                    continue

                # Görüntüyü JPEG formatında sıkıştır
                ret2, buffer = cv2.imencode('.jpg', frame)
                if not ret2:
                    self.get_logger().error("Görüntü sıkıştırılamadı!")
                    continue
                self.buffer = buffer
                self.flag = True
                self.extract_count += 1

            else:
                time.sleep(HZ - interval)
                
    def publish_loop(self):
        while True:
            if self.flag:
                # CompressedImage mesajı oluşturma
                self.msg.header.stamp = self.get_clock().now().to_msg()
                self.msg.data = np.array(self.buffer).tobytes()
                self.img_pubber.publish(self.msg)
                self.flag = False
                self.pub_count += 1

            else:
                time.sleep(HZ)        


    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CompressorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
