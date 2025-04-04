import math
import time

from .istrategy import IStrategy

# Sabitler (Parametreleri çalışma ortamına göre ayarla)
PI = math.pi
DESIRED_DISTANCE = 0.55      # Sağ duvardan istenen mesafe (m)
FRONT_THRESHOLD    = 0.45    # Ön engel için kritik mesafe (m)
MAX_RANGE          = 3.5     # Lazer tarayıcı maksimum ölçüm mesafesi (m)

KP = 1.0                   # Duvar takip için orantısal kazanç
MAX_ANG_VEL = 0.5          # Maksimum açısal hız (rad/s)
FORWARD_SPEED = 0.2        # İlerleme hızı (m/s)
LOOKAHEAD_DIST = 0.5       # Lookahead mesafe (m)
WALL_LOST_THRESHOLD = 1.5  # Sağdaki mesafe bu değerin üzerinde ise duvar kaybolmuş kabul edilir

class Strategy(IStrategy):
    def __init__(self, f_get_curr_pose, f_get_curr_ranges, f_pub_cmd_vel):
        super().__init__(f_get_curr_pose, f_get_curr_ranges, f_pub_cmd_vel)

    def avoid(self):
        self.exactly_turn(PI/2)
        """
        Sağ duvarı takip eden, hiçbir zaman çarpmayan ve kararlı çalışması amaçlanan üst düzey kaçınma algoritması.
        - Ön sensör bölgesi engel tespitinde önceliklidir.
        - Sağ duvardan alınan iki lazer okumasıyla duvarın açısı hesaplanır ve lookahead yöntemi ile
          öngörülen mesafe üzerinden P-kontrol uygulanır.
        - Duvar kaybolursa robot sağa dönerek duvarı arar.
        """
        while True:
            ranges = self.get_curr_ranges()
            if ranges is None:
                time.sleep(0.05)
                continue

            # 1. Güvenlik: Ön engel kontrolü
            # 350°-359° ve 0°-10° arası, robotun tam önündeki bölge
            front_samples = ranges[350:360] + ranges[0:10]
            front_distance = sum(front_samples) / len(front_samples)
            if front_distance < FRONT_THRESHOLD:
                # Ön engel tespit edildi: Ani sola dönüş
                self.pub_cmd_vel(0.0, MAX_ANG_VEL)
                time.sleep(0.1)
                continue

            # 2. Sağ duvar ölçümleri (filtreli, küçük pencere ortalamaları)
            # d_r: 268°-272° arası (yaklaşık 270°; duvara dik ölçüm)
            d_right_samples = ranges[268:273]
            d_right = sum(d_right_samples) / len(d_right_samples)
            if d_right > MAX_RANGE:
                d_right = MAX_RANGE

            # d_fr: 313°-317° arası (yaklaşık 315°; sağ-öne eğik ölçüm)
            d_front_right_samples = ranges[313:318]
            d_front_right = sum(d_front_right_samples) / len(d_front_right_samples)
            if d_front_right > MAX_RANGE:
                d_front_right = MAX_RANGE

            # 3. Sağ duvar kayıp kontrolü: Eğer sağ mesafe çok yüksekse, duvar kaybolmuş kabul edilir.
            if d_right > WALL_LOST_THRESHOLD:
                # Duvarı bulmak için hafif sağa dönerek arama yap
                self.pub_cmd_vel(FORWARD_SPEED * 0.5, -0.3)
                time.sleep(0.05)
                continue

            # 4. Geometri tabanlı duvar takip kontrolü
            # Sabit açı: 45° (radyan cinsinden)
            theta = math.radians(45)  # 0.7854 rad
            # Bölünen değerin sıfıra yakın olmaması için kontrol
            if d_front_right < 0.001:
                d_front_right = 0.001

            # Duvarın robotla yaptığı açıyı hesapla (alpha)
            alpha = math.atan((d_front_right * math.cos(theta) - d_right) / (d_front_right * math.sin(theta)))
            # Lookahead kullanılarak öngörülen duvar mesafesi
            d_pred = d_right * math.cos(alpha) + LOOKAHEAD_DIST * math.sin(alpha)
            error = DESIRED_DISTANCE - d_pred

            # P-kontrolör ile açısal hız üretiliyor
            angular_z = KP * error
            # Açısal hızı sınırla
            angular_z = max(-MAX_ANG_VEL, min(MAX_ANG_VEL, angular_z))
            # Dönüş miktarı arttıkça ileri hızı düşür (daha keskin dönüşlerde yavaş git)
            forward_speed = FORWARD_SPEED * (1 - abs(angular_z) / MAX_ANG_VEL)

            self.pub_cmd_vel(forward_speed, angular_z)
            time.sleep(0.05)

    def stop(self):
        """Robotu tamamen durdurur."""
        for _ in range(5):
            self.pub_cmd_vel(0.0, 0.0)
        time.sleep(0.5)

    def exactly_turn(self, radian: float):
        """
        Odometri verisine göre robotu tam olarak verilen radyan kadar döndürür.
        """
        _, _, theta0 = self.get_curr_pose()
        angular_speed = 0.3 if radian >= 0 else -0.3

        def angle_diff(current: float, start: float) -> float:
            diff = current - start
            while diff > math.pi:
                diff -= 2 * math.pi
            while diff < -math.pi:
                diff += 2 * math.pi
            return diff

        while True:
            current_pose = self.get_curr_pose()
            if current_pose is None:
                time.sleep(0.05)
                continue
            _, _, theta_current = current_pose
            if abs(angle_diff(theta_current, theta0)) >= abs(radian):
                break
            self.pub_cmd_vel(0.0, angular_speed)
            time.sleep(0.05)
        self.stop()
