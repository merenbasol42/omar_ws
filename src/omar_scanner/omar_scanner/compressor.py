import time
from threading import Thread

import zlib
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import ByteMultiArray

from .config.compressor import *

class Compressor(Node):

    def __init__(self):
        super().__init__(NODE_NAME)
        self.create_subscription(
            LaserScan,
            TOP_NAME_SCAN,
            self.scan_callback,
            10)
        self.msg: LaserScan = LaserScan()
        self.msg_flag: bool = False
        self.publisher_ = self.create_publisher(ByteMultiArray, TOP_NAME_SCAN_COMPRESSED, QOS_PROFILE)
        Thread(target=self.proccess_loop).start()

    def scan_callback(self, msg: LaserScan):
        if self.msg_flag: return
        self.msg = msg
        self.msg_flag = True
    
    def proccess_loop(self):
        while True:
            if not self.msg_flag:
                time.sleep(ZZZ_TIME)
                continue
            
            datas = [
                self.msg.angle_min,
                self.msg.angle_max,
                self.msg.angle_increment,
                self.msg.time_increment,
                self.msg.scan_time,
                self.msg.range_min,
                self.msg.range_max,
                len(self.msg.ranges),
                *self.msg.ranges,
                *self.msg.intensities
            ]
            
            float_array = np.array(datas, dtype=np.float32)
            self.get_logger().info(f"uncompress: {len(float_array.tobytes())}")
            compressed_bytes = zlib.compress(float_array.tobytes())
            self.get_logger().info(f"compress: {len(compressed_bytes)}")
            self.publisher_.publish(
                ByteMultiArray(
                    data = [bytes([
                        b]) for b in compressed_bytes]
                )        
            )
            self.msg_flag = False

def main(args=None):
    rclpy.init(args=args)
    node = Compressor()
    
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
