import math
import time

from .istrategy import IStrategy

PI: float = 3.141592

OBS_AVO_DIST: float = 0.35        # metre
OBS_AVO_MAX_ANG_VEL: float = 100.0   # metre / samiye
ANG_KP: float = 5.0

class Strategy(IStrategy):
    def __init__(self, f_get_curr_pose, f_get_curr_ranges, f_pub_cmd_vel):
        super().__init__(f_get_curr_pose, f_get_curr_ranges, f_pub_cmd_vel)

    def avoid(self):
        #*** 1. Robotun başlangıç pozisyonunu al *** 
        start_x, start_y, start_theta = self.get_curr_pose()

        #*** 2. Robotu sola döndür ***
        self.exactly_turn(math.pi/2)
        self.stop()

        #*** 3. Engel etrafından sür ***
        while True:
            # Güncel pozisyonu al
            curr_x, curr_y, curr_theta = self.get_curr_pose()

            # Başlangıç pozisyonundan, başlangıç yönü (theta0) doğrultusunda geçen bir çizgiye
            # robotun ne kadar uzak olduğunu hesapla.
            c = calc_pisagor(start_x, curr_x, start_y, curr_y)
            diff = calc_diff_line(start_x, curr_x, start_y, curr_y, start_theta)
            if  diff < 0.1 and c > 1.0:
                self.stop()
                break

            err = OBS_AVO_DIST - min(*self.get_curr_ranges()[290:359])
            print(f"err {err}")
            angular = ANG_KP * err
            angular = normalise(angular, PI)

            lin = min(0.2, (0.2*((PI - abs(angular) / PI))))

            self.pub_cmd_vel(lin, angular)
            time.sleep(0.05)

        #*** 4. Pozisyona geri dön ***
        _, _, curr_theta = self.get_curr_pose()
        turn_angle = start_theta - curr_theta
        # Açı normalizasyonu: -pi ile pi arası
        turn_angle = math.atan2(math.sin(turn_angle), math.cos(turn_angle))
        self.exactly_turn(turn_angle)
        self.stop()

    def stop(self):
        for _ in range(5):
            self.pub_cmd_vel(0.0, 0.0)
        time.sleep(0.5)

    def exactly_turn(self, radian: float):
        """
        Robotun mevcut odometry verisine göre, tam olarak verilen radian kadar dönmesini sağlar.
        Başlangıçta alınan yaw (theta) ile güncel yaw arasındaki fark istenen değere ulaşana kadar
        robot sabit açısal hız ile döner.
        """
        # Başlangıç pozisyonunu al
        _, _, theta0 = self.get_curr_pose()

        # Sabit bir açısal hız belirle (örneğin 0.3 rad/s)
        angular_speed = 0.3
        # Dönme yönünü istenen radian'a göre ayarla
        if radian < 0:
            angular_speed = -abs(angular_speed)
        else:
            angular_speed = abs(angular_speed)

        def angle_diff(current: float, start: float) -> float:
            """
            İki açı arasındaki farkı [-pi, pi] aralığında hesaplar.
            """
            diff = current - start
            while diff > math.pi:
                diff -= 2 * math.pi
            while diff < -math.pi:
                diff += 2 * math.pi
            return diff

        # Dönen açıyı takip et
        turned = 0.0
        while True:
            current_pose = self.get_curr_pose()
            if current_pose is None:
                time.sleep(0.05)
                continue
            _, _, theta_current = current_pose
            turned = angle_diff(theta_current, theta0)
            # İstenen açıyı döndüysek döngüden çık
            if abs(turned) >= abs(radian): break
            self.pub_cmd_vel(0.0, angular_speed)
            time.sleep(0.05)

        self.stop()

#
#
#

def calc_diff_line(x0: float, x1: float, y0: float, y1: float, theta0: float) -> float:
    return abs((x1 - x0) * math.sin(theta0) - (y1 - y0) * math.cos(theta0))

def calc_pisagor(x0: float, x1: float, y0: float, y1: float):
    return math.sqrt((x1 - x0) ** 2 + (y1 - y0) ** 2)

def normalise(val: float, top: float, bot: float | None = None):
    if bot is None: bot = -top
    if val > top:   val = top
    elif val < bot: val = bot
    return val
