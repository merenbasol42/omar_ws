from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

QOS_PROFILE = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=2,  
)