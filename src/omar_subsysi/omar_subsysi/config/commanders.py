from enum import Enum

class CMDER(Enum):
    MANUAL: str = "manual"
    OBSTACLE: str = "obstacle"
    LINE: str = "line"
    EMPTY: str = "empty"
    LIST: tuple[str] = (
        MANUAL,
        OBSTACLE,
        LINE,
        EMPTY
    )
