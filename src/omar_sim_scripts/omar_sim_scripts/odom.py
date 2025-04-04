import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from nav_msgs.msg import Odometry
from example_interfaces.msg import String
from geometry_msgs.msg import TransformStamped

#
# Logic
#

class OdomNode(Node):
    def __init__(self):
        super().__init__("odom")
        self.create_subscription(Odometry, "odom", self.odom_cb, 10)
        self.tf_br = TransformBroadcaster(self)
        self.stamped_tf = TransformStamped()
        self.stamped_tf.header.frame_id = "odom"
        self.stamped_tf.child_frame_id = "base_link"

    def odom_cb(self, msg: Odometry):
        self.stamped_tf.header.stamp = msg.header.stamp
        self.stamped_tf.transform.translation.x = msg.pose.pose.position.x
        self.stamped_tf.transform.translation.y = msg.pose.pose.position.y
        self.stamped_tf.transform.translation.z = msg.pose.pose.position.z
        self.stamped_tf.transform.rotation = msg.pose.pose.orientation
        self.tf_br.sendTransform(self.stamped_tf)

#
# Entry Point
#

def main():
    rclpy.init()
    node = OdomNode()
    
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()