

class IStrategy:
    def __init__(self, f_get_curr_pose, f_get_curr_ranges, f_pub_cmd_vel):
        self.__f_get_curr_pose = f_get_curr_pose
        self.__f_get_curr_ranges = f_get_curr_ranges
        self.__f_pub_cmd_vel = f_pub_cmd_vel

    def get_curr_pose(self) -> tuple[float, float, float]:
        return self.__f_get_curr_pose()

    def get_curr_ranges(self) -> list[float]:
        return self.__f_get_curr_ranges()

    def pub_cmd_vel(self, linear: float, angular: float) -> None:
        self.__f_pub_cmd_vel(linear, angular)

    def avoid(self) -> None:
        ...