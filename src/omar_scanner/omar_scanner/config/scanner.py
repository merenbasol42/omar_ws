from omar_config.config.general import PI
from numpy import deg2rad

NODE_NAME: str = "scanner"

INPUT_TOPIC: str = "scan"
OUTPUT_TOPIC: str = "scan/filtered"
LASER_FRAME_ID: str = "laserim"

ROTATE_DEGREE: float = 90.0
ANGLE_OFFSET: float = deg2rad(ROTATE_DEGREE)

ZZZ_TIME: float = 0.03