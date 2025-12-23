import math

from system.rigid_body_1d import RigidBody1D
from sensors.imu import Gyro1D
from estimators.kalman_gyro_bias_1d import KalmanGyroBias1D
from controllers.pid import PID


# =====================
# Parâmetros globais
# =====================
dt = 0.01
steps = 500

SETPOINT = 1.0  # rad/s (velocidade angular desejada)

# =====================
# Sistema físico
# =====================
body = RigidBody1D(
    omega0=0.0,
    inertia=1.0
)

# =====================
# Sensor (giroscópio)
# =====================
TRUE_BIAS = 0.05

gyro = Gyro1D(
    noise_std=0.05,
    bias=TRUE_BIAS
)

# =====================
# Estimador (Kalman)
# =====================
kf = KalmanGyroBias1D(
    omega0=0.0,
    bias0=0.0,
    P0=1.0,
    q_omega=1e-3,
    q_bias=1e-6,
    r_meas=0.05**2
)

# =====================
# Controlador PID
# =====================
pid = PID(
    kp=2.0,
    ki=0.5,
    kd=0.1,
    dt=dt,
    u_min=-5.0,
    u_max=5.0
)

# =====================
# Loop de simulação
# =====================
for i in range(steps):

    # ---------------------------------
    # Sensor lê o estado verdadeiro
    # ---------------------------------
    z = gyro.read(body.omega)

    # ---------------------------------
    # Estimador
    # ---------------------------------
    kf.predict()
    omega_est, bias_est = kf.update(z)

    # ---------------------------------
    # Controle (usa estado estimado!)
    # ---------------------------------
    torque = pid.compute(SETPOINT, omega_est)

    # ---------------------------------
    # Dinâmica do sistema
    # ---------------------------------
    body.update(torque, dt)

    # ---------------------------------
    # Log
    # ---------------------------------
    print(
        f"STEP {i:03d} | "
        f"SP: {SETPOINT: .2f} | "
        f"TRUE_OMEGA: {body.omega: .3f} | "
        f"MEAS: {z: .3f} | "
        f"EST_OMEGA: {omega_est: .3f} | "
        f"BIAS_EST: {bias_est: .3f} | "
        f"TORQUE: {torque: .3f}"
    )