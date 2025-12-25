import numpy as np
from system.rigid_body_nd import RigidBody3D
from sensors.gyro_nd import GyroND
from estimators.kalman_nd import KalmanND
from controllers.pid import PIDND
from controllers.attitude import AttitudeController
from math_utils.quaternion import quat_norm

# ---------------- CONFIG ----------------
DT = 0.01
STEPS = 300  # diminui para teste rápido

Q_REF = np.array([1.0, 0.0, 0.0, 0.0])
OMEGA_REF_MAX = 0.5  # rad/s
SLEW_RATE = 1.0      # rad/s²

# ---------------- SYSTEM ----------------
rb = RigidBody3D(
    inertia=np.array([1.0, 1.0, 1.0]),
    dt=DT
)
rb.q = quat_norm(np.array([0.98, 0.1, -0.05, 0.02]))

imu = GyroND(dim=3, noise_std=0.05, bias=[0.5, -0.25, 0.1])
kf = KalmanND(dim=3, q_omega=1e-3, q_bias=1e-6, r=0.05**2)

attitude = AttitudeController(kp=6.0, kd=1.2)

pid = PIDND(
    kp=[6,6,6],
    kd=[0.4,0.4,0.4],
    ki=[0,0,0],
    dt=DT,
    torque_limit=[1.0,1.0,1.0],
    d_cutoff=15.0
)

# ---------------- LOOP ----------------
omega_ref_prev = np.zeros(3)

for step in range(STEPS):
    omega_true = rb.omega.copy()
    z = imu.read(omega_true)
    kf.predict()
    omega_est, _ = kf.update(z)
    omega_est = np.array(omega_est)

    # -------- Attitude control --------
    omega_ref_cmd = attitude.step(Q_REF, rb.q, omega_est)
    # Saturação absoluta
    omega_ref_cmd = np.clip(omega_ref_cmd, -OMEGA_REF_MAX, OMEGA_REF_MAX)
    # Slew rate
    omega_ref = omega_ref_prev + np.clip(
        omega_ref_cmd - omega_ref_prev,
        -SLEW_RATE*DT,
        SLEW_RATE*DT
    )
    omega_ref_prev = omega_ref.copy()

    # -------- Rate control --------
    rate_error = omega_ref - omega_est
    error_dot = -omega_est  # amortecimento viscoso
    torque = pid.step(rate_error, error_dot)

    # -------- Dynamics --------
    rb.step(torque)

    if step % 10 == 0:
        print(f"STEP {step:03d} | "
              f"OMEGA: {rb.omega} | "
              f"OMEGA_REF: {omega_ref} | "
              f"TORQUE: {torque} | "
              f"Q: {rb.q}")