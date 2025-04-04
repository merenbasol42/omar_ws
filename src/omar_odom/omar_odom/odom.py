#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

from kasva_interfaces.msg import EncoderStats, EcoOdom
import transformations as tf
from .config.odom import *

class OdomNode(Node):

    def __init__(self):
        super().__init__(NODE_NAME)
        #
        # Fields
        #         
        self.x_ = 0.0
        self.y_ = 0.0
        self.theta_ = 0.0
        self.tf_br = TransformBroadcaster(self)
        #
        # ROS Messaging Definations
        #
        self.create_subscription(EncoderStats, TOP_NAME_ENC_STATS, self.enc_stats_cb, 10)
        self.odom_pubber = self.create_publisher(Odometry, TOP_NAME_ODOM, 10)
        self.eco_odom_pubber = self.create_publisher(EcoOdom, TOP_NAME_ECO_ODOM, 51)
        #
        # ROS Message Buffers
        #
        self.eco_odom_msg = EcoOdom()

        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = FRAME_ODOM
        self.odom_msg.child_frame_id = FRAME_ODOM_CHILD
        self.odom_msg.pose.pose.orientation.x = 0.0
        self.odom_msg.pose.pose.orientation.y = 0.0
        self.odom_msg.pose.pose.orientation.z = 0.0
        self.odom_msg.pose.pose.orientation.w = 1.0

        self.stamped_tf = TransformStamped()
        self.stamped_tf.header.frame_id = FRAME_ODOM
        self.stamped_tf.child_frame_id = FRAME_ODOM_CHILD
        self.get_logger().info(f"kankam kuulandıklarım: seperation:{WHEEL_SEPERATION}, radius:{WHEEL_RADIUS}")
    
    def enc_stats_cb(self, msg:EncoderStats):
        # self.get_logger().info("dap_left: %f , dap_right: %f" %(msg.dap_left,msg.dap_right))

        linear = (msg.vel_l + msg.vel_r) / 2 # linear velocity
        angular = (msg.vel_r - msg.vel_l) / WHEEL_SEPERATION # angular velocity

        # self.get_logger().info("linear: %f , angular: %f" %(linear,angular))

        # dap: delta angular position (of wheel)
        dx_left = msg.dap_l * WHEEL_RADIUS # delta x(yol) left wheel 
        dx_right = msg.dap_r * WHEEL_RADIUS # delta x(yol) right wheel

        d_s = (dx_left + dx_right) / 2  # İki tekerin aldığı  yol ortalaması 
        d_theta = (dx_right - dx_left) / WHEEL_SEPERATION  # delta theta

        self.theta_ += d_theta # Mevcut theta'ya eklendi
        self.x_ += d_s * math.cos(self.theta_)  # x coordinate calculation
        self.y_ += d_s * math.sin(self.theta_)  # y coordinate calculation

        # self.get_logger().info("x: %f , y: %f , theta: %f" %(self.x_,self.y_,self.theta_))
        
        # q = tf.quaternion_from_euler(0, 0, self.theta_) # euler angle to quartenion for robot theta
        self.eco_odom_msg.pose.x = self.x_
        self.eco_odom_msg.pose.y = self.y_
        self.eco_odom_msg.pose.theta = self.theta_
        self.eco_odom_msg.vel.linear = linear
        self.eco_odom_msg.vel.angular = angular
        self.eco_odom_pubber.publish(self.eco_odom_msg)
        
        q = tf.quaternion_from_euler(0, 0, self.theta_)
        self.odom_msg.pose.pose.orientation.w = q[0]
        self.odom_msg.pose.pose.orientation.x = q[1]
        self.odom_msg.pose.pose.orientation.y = q[2]
        self.odom_msg.pose.pose.orientation.z = q[3]
                                
        self.odom_msg.header.stamp = self.get_clock().now().to_msg() # Time stamping 
        self.odom_msg.pose.pose.position.x = self.x_
        self.odom_msg.pose.pose.position.y = self.y_
        self.odom_msg.twist.twist.linear.x = linear
        self.odom_msg.twist.twist.angular.z = angular
        self.odom_pubber.publish(self.odom_msg)
       
        self.stamped_tf.transform.translation.x = self.x_ 
        self.stamped_tf.transform.translation.y = self.y_
        self.stamped_tf.transform.rotation.w = q[0]
        self.stamped_tf.transform.rotation.x = q[1]
        self.stamped_tf.transform.rotation.y = q[2]
        self.stamped_tf.transform.rotation.z = q[3]
        self.stamped_tf.header.stamp = self.get_clock().now().to_msg()
        self.tf_br.sendTransform(self.stamped_tf)

def main():
    rclpy.init()
    node = OdomNode()

    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()