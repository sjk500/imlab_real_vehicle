class CurvatureSpeedPlanner:
    def __init__(self, v_max=10.0, v_c=8.0, k_c=5.0):
        self.v_max = v_max
        self.v_c = v_c
        self.k_c = k_c

    def compute_target_speed(self, curvature):
        target_speed = self.v_c / (1 + self.k_c * abs(curvature))
        return min(target_speed, self.v_max)

class PIDController:
    def __init__(self, Kp, Ki, Kd, dt=0.1):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.dt = dt
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, target, current):
        error = target - current
        self.integral += error * self.dt
        derivative = (error - self.prev_error) / self.dt

        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.prev_error = error

        return output

# ===================
# Example main loop
# ===================

# 초기화
speed_planner = CurvatureSpeedPlanner(v_max=12.0, v_c=10.0, k_c=4.0)
pid = PIDController(Kp=0.5, Ki=0.05, Kd=0.1, dt=0.1)

# 테스트용 현재 속도 (시뮬레이션)
current_speed = 0.0

# 임의의 곡률 시퀀스 (테스트용)
curvature_sequence = [0.0, 0.1, 0.2, 0.3, 0.1, 0.0, 0.5, 0.0]

for curvature in curvature_sequence:
    # 1️⃣ 곡률 기반 목표 속도 계산
    target_speed = speed_planner.compute_target_speed(curvature)
    print(f"[INFO] Curvature={curvature:.2f} → TargetSpeed={target_speed:.2f} m/s")

    # 2️⃣ PID 제어로 throttle/brake 계산
    control_output = pid.compute(target_speed, current_speed)

    if control_output > 0:
        throttle = min(control_output, 1.0)
        brake = 0.0
    else:
        throttle = 0.0
        brake = min(abs(control_output), 1.0)

    print(f"Throttle: {throttle:.2f}, Brake: {brake:.2f}")

    # 3️⃣ 시뮬레이션: current_speed 업데이트 (단순 모델)
    current_speed += (throttle - brake) * 0.5  # 예시용 간단한 속도 변화 모델
    current_speed = max(current_speed, 0.0)  # 음수 속도 방지

    print(f"Updated CurrentSpeed: {current_speed:.2f} m/s\n")
