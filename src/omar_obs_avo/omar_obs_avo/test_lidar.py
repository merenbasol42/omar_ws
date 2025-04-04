import time
import math

from threading import Thread
from typing import Literal

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from .avoid_strategies.strategy4 import Strategy 

#
# Constants
#

SCAN_MARGIN: int = 5

WARN_TIME: float = 5.0             # saniye
OBS_DETECT_DIST: float = 0.35      # metre

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
            f"l: {self.get('l'):.2f} |          | r: {self.get('r'):.2f} \n" 
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

        # ROS Definations
        self.scan_subber = self.create_subscription(LaserScan, "scan", self.config_scan, 10)
    
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
        self.last_scan.update(msg)
        print("---")
        print(self.last_scan.to_str())
        print("---")
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
