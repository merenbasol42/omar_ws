import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class FakeRobotPublisher(Node):
    def __init__(self):
        super().__init__('fake_robot_publisher')
        self.br = TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        # base_link to laser
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'  # Ana frame olarak base_link kullanılıyor
        t.child_frame_id = 'laserim'  # LIDAR için yeni frame adı

        # LIDAR'ın konumu
        t.transform.translation.x = 0.2  # LIDAR 0.2 metre önde
        t.transform.translation.y = 0.0  # Y ekseninde kayma yok
        t.transform.translation.z = 0.1  # LIDAR 0.1 metre yukarıda

        # LIDAR'ın yönü (180 derece z ekseninde dönüş)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 1.0  # Z ekseninde 180 derece dönüş için 1.0
        t.transform.rotation.w = 0.0  # W ekseni z ekseninde 180 derece dönüş için 0.0

        # Transform'u yayınla
        self.br.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = FakeRobotPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
