import time
import math

from threading import Thread
from typing import Literal

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from .avoid_strategies.strategy3 import Strategy 
from .config.obs_avoid_props import *

#
# Constants
#

SCAN_MARGIN: int = 5
MAINLOOP_ZZZ: float = 0.05 # saniye

#
# Other Classes
#

class ScanData:
    def __init__(self):
        # names -> f: front, l: left, b: back, r: right
        self.names = ("f", "fl", "l", "bl", "b", "br", "r", "fr")
        self.ranges = []
        self.p_list: list[ScanData.Point] = []
        for _ in range(8):
            self.p_list.append(ScanData.Point(SCAN_MARGIN))

    def config(self, msg: LaserScan) -> dict[str, int]:
        self.__set_index(len(msg.ranges))
        log = {}
        for i, point in enumerate(self.p_list):
            log[self.names[i]] = point.index
        return log

    def __set_index(self, n: int) -> None:
        a = n // 8
        for i in range(8):
            self.p_list[i].index = i * a 

    def update(self, msg: LaserScan) -> None:
        self.ranges = msg.ranges
        for point in self.p_list:
            point.extract_val(msg.ranges)
    
    def get(self, key: Literal["f", "fl", "l", "bl", "b", "br", "r", "fr"]) -> float:
        return self.p_list[self.names.index(key)].value        

    def to_str(self):
        return (
            f"fl: {self.get('fl'):.2f} | f: {self.get('f'):.2f} | fr: {self.get('fr'):.2f} \n"
            f"l: {self.get('l'):.2f} |        | r: {self.get('r'):.2f} \n" 
            f"bl: {self.get('bl'):.2f} | b: {self.get('b'):.2f} | br: {self.get('br'):.2f}" 
        )

    class Point:
        def __init__(self, margin: int):
            self.index: int = 0
            self.margin: int = margin
            self.value: float = float('inf')

        def extract_val(self, ranges: list[float]):
            self.value = min(
                *get_range(
                    ranges,
                    self.index - self.margin,
                    self.index + self.margin
                )
            )

#
# Logic
#

class AsyncObsAvo(Node):
    def __init__(self):
        super().__init__("kasva_obs_avo")

        self.last_scan: ScanData = ScanData()
        self.curr_pose = None  # (x, y, theta)

        # ROS Definations
        self.scan_subber = self.create_subscription(LaserScan, "scan", self.config_scan, 10)
        self.odom_subber = self.create_subscription(Odometry, "odom", self.odom_callback, 10)
        self.cmd_vel_pubber = self.create_publisher(Twist, "cmd_vel", 10)


        def get_curr_pose():
            return self.curr_pose

        def get_ranges():
            return self.last_scan.ranges

        self.avoid_strategy = Strategy(
            get_curr_pose,
            get_ranges,
            self.pub_cmd_vel
        )

        Thread(target=self.mainloop, daemon=True).start()

    #
    # ROS Methods
    #

    def config_scan(self, msg: LaserScan):
        # LaserScan konfigürasyonu
        config_log = self.last_scan.config(msg)
        self.get_logger().info(f"indexs = {config_log}")
        # İlk konfigürasyon sonrası asıl callback'e geçiyoruz.
        self.scan_subber.callback = self.scan_cb
        self.get_logger().info(f"scan konfigurasyonu tamamlandi.")

    def scan_cb(self, msg: LaserScan):
        n = len(msg.ranges)
        msg.ranges = msg.ranges[n//2:] + msg.ranges[:n//2]
        self.last_scan.update(msg)
        print(self.last_scan.to_str())

    def odom_callback(self, msg: Odometry):
        # Odometry bilgisinden pozisyon ve yönelim (yaw) bilgisini alıyoruz.
        pos = msg.pose.pose.position
        orient = msg.pose.pose.orientation
        # Basit bir quaternion'dan yaw hesaplaması
        siny_cosp = 2.0 * (orient.w * orient.z + orient.x * orient.y)
        cosy_cosp = 1.0 - 2.0 * (orient.y * orient.y + orient.z * orient.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        self.curr_pose = (pos.x, pos.y, yaw)

    def pub_cmd_vel(self, linear: float, angular: float):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.cmd_vel_pubber.publish(msg)
    
    #
    # External Access
    #

    def mainloop(self):
        self.get_logger().info("başladım")
        while rclpy.ok():
            if self.detect():
                self.stop()
                if not self.warn():
                    self.avoid()  # Kaçınma manevrası, ayrı bir thread tarafından çalıştırılabilir.
            else:
                time.sleep(MAINLOOP_ZZZ)

    #
    # Logic
    # 

    def detect(self) -> bool:
        if self.last_scan.get('f') < OBS_DETECT_DIST:
            self.get_logger().warn("Engel tespit edildi")
            return True
        return False

    def warn(self) -> bool:
        a: float = time.time()
        while True:
            if not self.detect():
                return True
            if time.time() - a > WARN_TIME:
                return False
            self.get_logger().warn("Engel uyarılıyor ...")
            time.sleep(0.1)

    def avoid(self):
        self.get_logger().warn("Kaçınma aşaması başladı.")

        self.avoid_strategy.avoid()

        self.get_logger().warn("Engelden kaçınma sonlandı.")

    #
    # Tools
    # 

    def stop(self):
        for _ in range(5):
            self.pub_cmd_vel(0.0, 0.0)
        time.sleep(0.5)

    def exactly_turn(self, radian: float):
        """
        Robotun mevcut odometry verisine göre, tam olarak verilen radian kadar dönmesini sağlar.
        Başlangıçta alınan yaw (theta) ile güncel yaw arasındaki fark istenen değere ulaşana kadar
        robot sabit açısal hız ile döner.
        """
        # Başlangıç pozisyonunu al
        start_pose = self.curr_pose
        if start_pose is None:
            self.get_logger().error("Odometry verisi alınamadı, dönme iptal edildi.")
            return
        _, _, theta0 = start_pose

        # Sabit bir açısal hız belirle (örneğin 0.3 rad/s)
        angular_speed = 0.3
        # Dönme yönünü istenen radian'a göre ayarla
        if radian < 0:
            angular_speed = -abs(angular_speed)
        else:
            angular_speed = abs(angular_speed)

        def angle_diff(current: float, start: float) -> float:
            """
            İki açı arasındaki farkı [-pi, pi] aralığında hesaplar.
            """
            diff = current - start
            while diff > math.pi:
                diff -= 2 * math.pi
            while diff < -math.pi:
                diff += 2 * math.pi
            return diff

        # Dönen açıyı takip et
        turned = 0.0
        while rclpy.ok():
            current_pose = self.curr_pose
            if current_pose is None:
                self.get_logger().warn("Odometry verisi alınamıyor, bekleniyor...")
                time.sleep(0.05)
                continue
            _, _, theta_current = current_pose
            turned = angle_diff(theta_current, theta0)
            # İstenen açıyı döndüysek döngüden çık
            if abs(turned) >= abs(radian):
                break
            self.pub_cmd_vel(0.0, angular_speed)
            time.sleep(0.05)

        self.stop()
        self.get_logger().info(
            f"exactly_turn tamamlandı: İstenen {radian} rad, dönülen {turned:.2f} rad"
        )

#
# Yardımcı Fonksiyonlar
#

def get_range(lst: list, a: int, b: int) -> list:
    """
    Listenin indekslerini dairesel olarak değerlendirir.
    Örnek:
      liste = [11, 12, 13, 14, 15, 16, 17]
      get_range(liste, -2, 2)  =>  [16, 17, 11, 12]
    """
    n = len(lst)
    a = a % n
    b = b % n
    if a < b:
        return lst[a:b]
    else:
        return lst[a:] + lst[:b]
    # return lst[a:b]

def calc_diff_line(x0: float, x1: float, y0: float, y1: float, theta0: float) -> float:
    return abs((x1 - x0) * math.sin(theta0) - (y1 - y0) * math.cos(theta0))

def calc_pisagor(x0: float, x1: float, y0: float, y1: float):
    return math.sqrt((x1 - x0) ** 2 + (y1 - y0) ** 2)

def normalise(val: float, top: float, bot: float | None = None):
    if bot is None: bot = -top
    if val > top:   val = top
    elif val < bot: val = bot
    return val

#
# Entry Point
#

def main():
    rclpy.init()
    node = AsyncObsAvo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()
