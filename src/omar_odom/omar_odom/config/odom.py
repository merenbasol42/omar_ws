from .messaging import *
from omar_config.config.robot import ROBOT


NODE_NAME: str = "odom"
FRAME_ODOM: str = "odom"
FRAME_ODOM_CHILD: str = "base_link"

WHEEL_SEPERATION: float = ROBOT.WH_SEP
WHEEL_RADIUS: float = ROBOT.WH_RAD