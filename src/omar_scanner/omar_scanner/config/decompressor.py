from omar_config.config.general import PI
from numpy import deg2rad
from .qos_profile import QOS_PROFILE
from .lidar import *
from .filter import *

from .messaging import TOP_NAME_SCAN_COMPRESSED, TOP_NAME_SCAN_FILTERED

NODE_NAME: str = "scanner__decompressor"

LASER_FRAME_ID: str = "laser"

ROTATE_DEGREE: float = 90.0
ANGLE_OFFSET: float = deg2rad(ROTATE_DEGREE)

ZZZ_TIME: float = 0.03