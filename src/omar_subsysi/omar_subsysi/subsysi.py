
import rclpy
from rclpy.node import Node
from .config.subsysi import *

from geometry_msgs.msg import Quaternion, Vector3
from std_msgs.msg import String

from omar_interfaces.msg import VelocityCommand, EncoderStats
from omar_interfaces.srv import SetCommander

#
# Logic
#

class SubsysiNode(Node):
    def __init__(self):
        super().__init__(NODE_NAME)
        self.curr_cmder: str = START_CMDER
        
        self.enc_stats_pubber = self.create_publisher(EncoderStats, TOP_NAME_ENC_STATS, 5)
        self.create_service(SetCommander, SRV_NAME_SET_CMDER, self.set_cmder_srv_cb)
        self.create_subscription(VelocityCommand, TOP_NAME_CMD_VEL, self.cmd_vel_cb)

        self.mc_cmd_vel_pubber = self.create_publisher(Vector3, TOP_NAME_MC_CMD_VEL, 5)
        self.create_subscription(Quaternion, TOP_NAME_MC_ENC_STATS, self.mc_enc_stats_cb, 5)
        self.create_subscription(String, TOP_NAME_MC_OUT, self.mc_out_cb, 5)

        # Message buffer
        self.enc_stats_msg = EncoderStats()
        self.mc_cmd_vel_msg = Vector3()

    def mc_enc_stats_cb(self, msg: Quaternion):
        self.enc_stats_msg.dap_l = msg.x
        self.enc_stats_msg.dap_r = msg.y
        self.enc_stats_msg.vel_l = msg.z
        self.enc_stats_msg.vel_r = msg.w
        self.enc_stats_pubber.publish(self.enc_stats_msg)

    def mc_out_cb(self, msg: String):
        self.get_logger().info(msg.data)

    def set_cmder_srv_cb(self, req: SetCommander.Request, res: SetCommander.Response):
        res.success = False
        try: # evet büyük eşit kullanırsam ölürüm
            aha: bool = not (CMDER.LIST.index(self.curr_cmder) < CMDER.LIST.index(req.commander))
        except ValueError:
            self.get_logger().warn(f"{req.commander} komutan listesinde yok")
            return res
        
        if aha:
            res.success = True
            self.curr_cmder = req.commander if req.target else DEFAULT_CMDER
            self.get_logger(f"şuanki komutan: {self.curr_cmder}")

        return res

    def cmd_vel_cb(self, msg: VelocityCommand):
        if msg.commander != self.curr_cmder: return

        self.mc_cmd_vel_msg.x = msg.vel.linear
        self.mc_cmd_vel_msg.y = msg.vel.angular
        self.mc_cmd_vel_pubber.publish(self.mc_cmd_vel_msg)

#
# Entry Point
#

def main():
    rclpy.init()
    node = SubsysiNode()
    try: rclpy.spin(node)
    except: pass
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()