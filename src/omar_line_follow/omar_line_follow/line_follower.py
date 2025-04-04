#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
from .config.line_follow import *


class SimpleLaneFollower(Node):
    def __init__(self):
        super().__init__('simple_lane_follower')
        # Abone olunacak görüntü konusunu ayarlayın (örneğin: /camera/image_raw)
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        # cmd_vel konusuna Twist mesajları yayınlanacak
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.bridge = CvBridge()
        self.get_logger().info("Simple Lane Follower düğümü başlatıldı.")

    def image_callback(self, msg):
        try:
            # ROS2 Image mesajını OpenCV formatına çevir
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error("Görüntü dönüştürme hatası: {}".format(e))
            return

        # Görüntüyü gri tonlamaya çevir
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Siyah şeridi belirginleştirmek için eşikleme işlemi uyguluyoruz.
        # Beyaz zemin üzerinde siyah şerit olduğundan, THRESH_BINARY_INV kullanarak
        # siyah alanları beyaz yapıyoruz.
        ret, thresh = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY_INV)

        # Eşiklenmiş görüntüde konturları bul
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            # Bulunan konturlar arasında en büyük alanlı olanı seçiyoruz
            largest_contour = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest_contour)
            if M["m00"] != 0:
                # Şeridin merkez koordinatlarını (cx, cy) hesapla
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
            else:
                cx, cy = 0, 0

            # Görüntü üzerine konturu ve merkez noktasını çiz
            cv2.drawContours(frame, [largest_contour], -1, (0, 255, 0), 2)
            cv2.circle(frame, (cx, cy), 5, (255, 0, 0), -1)

            # Kameranın (görüntünün) ortasını hesapla
            image_center_x = frame.shape[1] // 2
            # Hata: Görüntü merkezi ile şerit merkezinin farkı
            error = image_center_x - cx

            # Twist mesajı oluşturup yayınla
            twist = Twist()
            twist.linear.x = LIN_VEL  # Sabit ileri hız (duruma göre ayarlanabilir)
            twist.angular.z = K_STEER * error
            self.cmd_vel_pub.publish(twist)
        else:
            self.get_logger().info("Şerit bulunamadı.")
            # Şerit tespit edilemezse dur veya gerekli başka aksiyon alabilirsiniz
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)

        # Görüntü merkezini görselleştirmek için dikey bir çizgi çizelim
        cv2.line(frame, (frame.shape[1]//2, 0), (frame.shape[1]//2, frame.shape[0]), (0, 0, 255), 2)
        cv2.imshow("Şerit Tespiti", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleLaneFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Düğüm durduruluyor...")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
