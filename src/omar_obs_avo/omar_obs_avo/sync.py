
import rclpy
from rclpy.node import Node

#
# Logic
#

class MyNode(Node):
    def __init__(self):
        super().__init__("my_node")
    
#
# Entry Point
#

def main():
    rclpy.init()
    node = MyNode()
    try: rclpy.spin(node)
    except: pass
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()