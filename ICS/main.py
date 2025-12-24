import random
from system.rigid_body_nd import RigidBodyND
from sensors.gyro_nd import GyroND
from estimators.kalman_nd import KalmanND
from controllers.pid import PIDND
# ---------------- CONFIG ----------------
DT = 0.01
STEPS = 1000
N = 2

SETPOINT = [1.0, -0.5]

# ---------------- SYSTEM ----------------
rb = RigidBodyND(
    dim=2,
    inertia=[1.0, 1.0],
    dt=DT
)

imu = GyroND(
    dim=2,
    noise_std=0.05,
    bias=[0.5, -0.25]
)

kf = KalmanND(
    dim=2,
    q_omega=1e-3,
    q_bias=1e-6,
    r=0.05**2
)

pid = PIDND(
    kp=[2.0, 2.0],
    ki=[0.0, 0.0],
    kd=[0.1, 0.1],
    dt=DT
)
# ---------------- LOOP ----------------
for step in range(STEPS):
    # TRUE dynamics
    omega_true = rb.omega

    # IMU measurement
    z = imu.read(omega_true)

    # Kalman
    kf.predict()
    omega_est, bias_est = kf.update(z)

    # PID error (note o sinal!)
    error = [
        omega_est[i] - SETPOINT[i]
        for i in range(len(SETPOINT))
    ]

    torque = pid.step(error)

    # Apply torque to rigid body
    rb.apply_torque(torque)

    if step % 10 == 0:
        print(
            f"STEP {step:03d} | "
            f"TRUE: {omega_true} | "
            f"EST: {omega_est} | "
            f"BIAS: {bias_est} | "
            f"TORQUE: {torque}"
        )