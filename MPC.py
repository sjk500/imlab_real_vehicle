# 코드 실행 상태가 초기화되었으므로 다시 파일을 저장합니다.

from pathlib import Path

# -------------------------------
# 📦 Import
# -------------------------------
import numpy as np
import pandas as pd
import casadi as ca
# import can  # 실차 적용 시 사용 예정

# -------------------------------
# 📁 1. 기준 경로 파일 로드 함수
# -------------------------------
def load_reference_path(file_path):
    "CSV 경로 파일을 불러와 latitude, longitude, yaw를 반환"
    df = pd.read_csv(file_path)
    return df[['latitude', 'longitude', 'yaw']].to_numpy()

# -------------------------------
# 🧭 2. 실시간 GNSS + IMU 후처리 함수
# -------------------------------
def preprocess_gnss_data(gnss_lat, gnss_lon, gnss_yaw, origin_lat, origin_lon):
    "위경도 및 yaw를 ENU 좌표계 기준으로 변환하고, yaw를 라디안으로 변환"
    from pyproj import Proj, transform
    proj_wgs84 = Proj(proj='latlong', datum='WGS84')
    proj_enu = Proj(proj='utm', zone=52, ellps='WGS84')  # 예시: 한국 중부

    x, y = transform(proj_wgs84, proj_enu, gnss_lon, gnss_lat)
    yaw_rad = np.deg2rad(gnss_yaw)
    return np.array([x, y, yaw_rad])

# -------------------------------
# 🔧 3. MPC 제어 모델 정의 및 최적화 함수
# -------------------------------
def setup_mpc_model(N=20, dt=0.1, L=2.5):
    "CasADi 기반 MPC 모델 및 solver 반환"
    # 상태: x, y, psi
    x = ca.SX.sym('x'); y = ca.SX.sym('y'); psi = ca.SX.sym('psi')
    delta = ca.SX.sym('delta')  # 제어 입력
    v = ca.SX.sym('v')          # 속도는 측정값으로 외부 입력

    states = ca.vertcat(x, y, psi)
    n_states = 3

    rhs = ca.vertcat(
        v * ca.cos(psi),
        v * ca.sin(psi),
        v / L * ca.tan(delta)
    )

    f = ca.Function('f', [states, delta, v], [rhs])

    # 최적화 변수
    U = ca.SX.sym('U', N)
    X = ca.SX.sym('X', n_states, N + 1)
    P = ca.SX.sym('P', n_states + n_states * N + 1)  # 초기상태 + ref + v

    Q = np.diag([10, 10, 1])
    R = 1.0
    Rd = 10.0
    obj = 0
    g = []

    g.append(X[:,0] - P[0:n_states])
    for k in range(N):
        st = X[:,k]; con = U[k]
        ref = P[n_states + k*3: n_states + (k+1)*3]
        obj += ca.mtimes([(st - ref).T, Q, (st - ref)]) + R * con**2
        if k > 0:
            obj += Rd * (U[k] - U[k-1])**2
        st_next = st + dt * f(st, con, P[-1])
        g.append(X[:,k+1] - st_next)

    OPT_variables = ca.vertcat(ca.reshape(X, -1, 1), U)
    nlp_prob = {'f': obj, 'x': OPT_variables, 'g': ca.vertcat(*g), 'p': P}
    opts = {"ipopt.print_level": 0, "print_time": 0}
    solver = ca.nlpsol('solver', 'ipopt', nlp_prob, opts)

    return solver, f

# -------------------------------
# 🎯 4. MPC 기반 조향각 계산 함수
# -------------------------------
def compute_steering_angle(solver, f, current_state, ref_traj, v_meas, N=20):
    "MPC를 이용하여 최적 조향각 계산"
    n_states = 3
    p = np.concatenate([current_state, ref_traj.flatten(), [v_meas]])
    x_init = np.tile(current_state.reshape(-1,1), (1, N+1))
    u_init = np.zeros((N, 1))
    init = np.vstack((x_init.flatten(), u_init.flatten()))

    sol = solver(x0=init, p=p, lbg=0, ubg=0)
    u_star = sol['x'][-N:].full().flatten()
    return u_star[0]  # 첫 스텝의 조향각

# -------------------------------
# 🚗 5. 조향각 CAN 송신 함수 (실차용, 현재 제외)
# -------------------------------
def send_steering_angle_to_can(angle_rad):
    "조향각을 차량에 송신 (실차용 placeholder)"
    pass  # 향후 can.Message 등으로 구현

# -------------------------------
# ▶️ 6. 실행부
# -------------------------------
def main():
    path = load_reference_path("Path_downsampled.csv")
    origin_lat, origin_lon = path[0,0], path[0,1]

    solver, f = setup_mpc_model()

    while True:
        # 실시간 GNSS 데이터 입력 예시 (추후 실제 연결 필요)
        gnss_lat, gnss_lon, gnss_yaw = 37.61245, 126.99414, -1.28  # 예시값
        current_state = preprocess_gnss_data(gnss_lat, gnss_lon, gnss_yaw, origin_lat, origin_lon)

        # 예시: 기준 경로 중 가까운 20개 추출 (추후 개선 필요)
        ref_traj = path[:20]  # 단순히 앞부분 사용
        ref_traj_enu = [preprocess_gnss_data(lat, lon, yaw, origin_lat, origin_lon) for lat, lon, yaw in ref_traj]

        steering_angle = compute_steering_angle(solver, f, current_state, np.array(ref_traj_enu), v_meas=2.5)
        print(f"조향각(rad): {steering_angle:.4f}")
        # send_steering_angle_to_can(steering_angle)

# -------------------------------
# 🔚 종료
# -------------------------------
if __name__ == "__main__":
    main()