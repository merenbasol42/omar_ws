import time
from array import array
from threading import Thread

import zlib
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import ByteMultiArray

from .config.decompressor import *

FILTER_MARGIN = 10

def listeyi_kaydir(liste, kaydirma_miktari):
    kaydirma_miktari %= len(liste)  # Kaydırma miktarını listenin uzunluğuna göre ayarla
    return liste[-kaydirma_miktari:] + liste[:-kaydirma_miktari]

class Decompressor(Node):
    def __init__(self):
        super().__init__(NODE_NAME)
                
        self.scan_msg: LaserScan = LaserScan()
        self.byte_msg: ByteMultiArray = None 
        self.msg_flag: bool = False

        self.scan_msg.header.frame_id = LASER_FRAME_ID
        
        self.subber = self.create_subscription(ByteMultiArray, TOP_NAME_SCAN_COMPRESSED, self.scan_raw_cb, QOS_PROFILE)   
        self.pubber = self.create_publisher(LaserScan, TOP_NAME_SCAN_FILTERED, 2)

        Thread(target=self.proccess_loop).start()
        
    def scan_raw_cb(self, msg: LaserScan):
        if self.msg_flag: return
        self.byte_msg = msg
        self.msg_flag = True

    def proccess_loop(self):
        while True:
            if not self.msg_flag: 
                self.get_logger().info("ZZZ")
                time.sleep(ZZZ_TIME)     
                continue

            self.get_logger().info("WORK")
            self.decode()
            self.filter_lidar_data()
            # self.rotate_lidar_data()
            # Damgaları değiştir
            self.scan_msg.header.stamp = self.get_clock().now().to_msg()
            # Yayımla 
            self.pubber.publish(self.scan_msg)
            # Yeni bir mesaj işlemeye hazır olduğunu haber ver
            self.msg_flag = False

    def decode(self):
        datas = np.frombuffer(
            zlib.decompress(
                b''.join(self.byte_msg.data)
            ),
            dtype=np.float32
        ).tolist()

        self.scan_msg.angle_min = datas[0]
        self.scan_msg.angle_max = datas[1]
        self.scan_msg.angle_increment = datas[2]
        self.scan_msg.time_increment = datas[3]
        self.scan_msg.scan_time = datas[4]
        self.scan_msg.range_min = datas[5]
        self.scan_msg.range_max = datas[6]
        
        len_ = int(datas[7])
        self.scan_msg.ranges = datas[8:len_+8]
        self.scan_msg.intensities = datas[len_+8:]

    def rotate_lidar_data(self):
        self.get_logger().info("gggg")
        # self.scan_msg.ranges.reverse()
        self.scan_msg.ranges = listeyi_kaydir(self.scan_msg.ranges, len(self.scan_msg.ranges) // 2)

    def filter_lidar_data(self):    
        # self.get_logger().info("amedim")
        A = len(self.scan_msg.ranges) // 4   
        inf = float('inf')
        # Float tipinde bir array oluşturuyoruz
        self.scan_msg.ranges[A:-A] = array('f', [inf] * len(self.scan_msg.ranges[A:-A]))



def main(args=None):
    rclpy.init(args=args)
    node = Decompressor()
    
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
