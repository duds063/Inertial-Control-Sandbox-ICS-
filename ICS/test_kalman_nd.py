from sensors.gyro_nd import GyroND
from estimators.kalman_nd import KalmanND

gyro = GyroND(
    dim=2,
    noise_std=[0.05, 0.05],
    bias=[0.03, -0.02]
)

kf = KalmanND(
    dim=2,
    q_omega=1e-3,
    q_bias=1e-7,
    r=0.05**2
)

true_omega = [1.0, -0.5]

print("=== TESTE KALMAN ND ===")

for step in range(50):
    z = gyro.read(true_omega)
    kf.predict()
    omega_est, bias_est = kf.update(z)

    if step % 5 == 0:
        print(
            f"STEP {step:02d} | "
            f"MEAS: {z} | "
            f"OMEGA_EST: {omega_est} | "
            f"BIAS_EST: {bias_est}"
        )