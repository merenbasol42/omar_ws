import math
import time

from .istrategy import IStrategy

# Sabitler ve parametreler (ortama ve robot dinamiklerine göre ayarla)
PI = math.pi
DESIRED_DISTANCE = 0.45    # Sağ duvardan istenen mesafe (m)
FRONT_THRESHOLD  = 0.35    # Ön engel için kritik mesafe (m)
BACK_THRESHOLD   = 0.35    # Arka engel için kritik mesafe (m)
FORWARD_SPEED    = 0.2     # Temel ileri hız (m/s)
MAX_ANG_VEL      = 0.3     # Maksimum açısal hız (rad/s)
KP               = 0.3     # Duvar takip için orantısal kazanç
LOOKAHEAD_DIST   = 0.5     # Lookahead mesafe (m)
DEADBAND         = 0.05    # Hata için deadband
MAX_RANGE        = 3.5     # Lidar ölçüm sınırı

class Strategy(IStrategy):
    def __init__(self, f_get_curr_pose, f_get_curr_ranges, f_pub_cmd_vel):
        super().__init__(f_get_curr_pose, f_get_curr_ranges, f_pub_cmd_vel)

    def avoid(self):
        while True:
            ranges = self.get_curr_ranges()
            if ranges is None:
                time.sleep(0.05)
                continue

            # 1. Ön Engel Kontrolü: 350°–359° ve 0°–10° arası
            front_angles = list(range(350, 360)) + list(range(0, 11))
            front_vals = [ranges[i] for i in front_angles if ranges[i] < MAX_RANGE]
            front_distance = sum(front_vals)/len(front_vals) if front_vals else MAX_RANGE
            if front_distance < FRONT_THRESHOLD:
                # Öndeki engel tespit edildiyse: sabit bir ani sola dönüş uygulayıp döngünün başına dön
                self.pub_cmd_vel(0.0, MAX_ANG_VEL)
                time.sleep(0.05)
                continue

            # 2. Arka Engel Kontrolü: 150°–210° arası
            back_angles = list(range(150, 211))
            back_vals = [ranges[i] for i in back_angles if ranges[i] < MAX_RANGE]
            back_distance = min(back_vals) if back_vals else MAX_RANGE
            # Arka tarafta engel varsa, küçük bir düzeltme ekleyelim (örneğin, 0.2 rad/s yönlendirme)
            back_correction = 0.2 if back_distance < BACK_THRESHOLD else 0.0

            # 3. Sağ Duvar Takibi için Ölçümler:
            # a: 265°–275° (yaklaşık 270° – duvara dik ölçüm)
            right_angles = list(range(265, 276))
            right_vals = [ranges[i] for i in right_angles if ranges[i] < MAX_RANGE]
            a = sum(right_vals)/len(right_vals) if right_vals else MAX_RANGE

            # b: 310°–320° (yaklaşık 315° – sağ-öne eğik ölçüm)
            b_angles = list(range(310, 321))
            b_vals = [ranges[i] for i in b_angles if ranges[i] < MAX_RANGE]
            b_val = sum(b_vals)/len(b_vals) if b_vals else MAX_RANGE

            # 4. Geometri Tabanlı Hesaplamalar:
            theta_rad = math.radians(45)  # 45° = 0.7854 rad
            # Bölme sıfıra yaklaşmaması için küçük epsilon ekle
            denom = a * math.sin(theta_rad) + 1e-6
            alpha = math.atan((a * math.cos(theta_rad) - b_val) / denom)
            # Lookahead mesafesi kullanılarak öngörülen duvar mesafesi
            d_pred = a * math.cos(alpha) + LOOKAHEAD_DIST * math.sin(alpha)
            error = DESIRED_DISTANCE - d_pred

            # Deadband uygulaması (küçük hataları göz ardı et)
            if abs(error) < DEADBAND:
                error = 0.0

            # P-kontrol ile açısal düzeltme
            angular_z = KP * error
            # Arka düzeltmeyi de ekleyelim (basitçe; gerekirse daha sofistike bir yöntem kullanılabilir)
            angular_z += back_correction

            # Açısal hızı sınırla
            angular_z = max(-MAX_ANG_VEL, min(MAX_ANG_VEL, angular_z))

            # 5. İleri Hızın Hesaplanması:
            # İleri hız, ani dönüşlerde bile en az %50 oranında korunmalı
            # (abs(angular_z)/MAX_ANG_VEL) ne kadar yüksekse, hızı %50 oranında azaltıyoruz.
            forward_speed = FORWARD_SPEED * (1 - 0.5 * (abs(angular_z) / MAX_ANG_VEL))
            if forward_speed < FORWARD_SPEED * 0.5:
                forward_speed = FORWARD_SPEED * 0.5

            self.pub_cmd_vel(forward_speed, angular_z)
            time.sleep(0.05)

    def stop(self):
        for _ in range(5):
            self.pub_cmd_vel(0.0, 0.0)
        time.sleep(0.5)

    def exactly_turn(self, radian: float):
        _, _, theta0 = self.get_curr_pose()
        angular_speed = 0.2 if radian >= 0 else -0.2

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
