from sensors.gyro_nd import GyroND

gyro = GyroND(
    dim=2,
    noise_std=[0.05, 0.05],
    bias=[0.02, -0.01]
)

true_omega = [1.0, -0.5]

print("=== TESTE GYRO ND ===")

for i in range(10):
    z = gyro.read(true_omega)
    print(f"TRUE: {true_omega} | MEAS: {z}")