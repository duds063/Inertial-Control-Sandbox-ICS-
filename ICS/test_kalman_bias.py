import math
from sensors.gyro_1d import Gyro1D
from estimators.kalman_gyro_bias_1d import KalmanGyroBias1D

gyro = Gyro1D(
    noise_std=0.05,
    bias=0.05
)

kf = KalmanGyroBias1D(
    omega0=0.0,
    bias0=0.0,
    P0=1.0,
    q_omega = 1e-2,    # omega MUDA
    q_bias  = 1e-6,    # bias MUDA MUITO POUCO
    R=0.05**2
)

for i in range(300):
    TRUE_OMEGA = 1.0 + 0.2 * math.sin(0.05 * i)

    z = gyro.read(TRUE_OMEGA)

    kf.predict()
    omega_est, bias_est = kf.update(z)

    if i % 30 == 0:
        print(
            f"TRUE_OMEGA: {TRUE_OMEGA:.3f} | "
            f"MEAS: {z:.3f} | "
            f"OMEGA_EST: {omega_est:.3f} | "
            f"BIAS_EST: {bias_est:.3f}"
        )