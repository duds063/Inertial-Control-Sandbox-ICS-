import random
from sensors.imu import IMUSensor
from ICS.estimators.kalman_nd import Kalman1D

TRUE_OMEGA = 1.0

imu = IMUSensor(
    gyro_noise_std=0.05,
    bias=0.00
)

kf = Kalman1D(
    x0=0.0,
    P0=1.0,
    Q=1e-4,
    R=0.05**2
)

for i in range(200):
    z = imu.read({"omega": TRUE_OMEGA})["gyro"]

    kf.predict()
    est = kf.update(z)

    if i % 20 == 0:
        print(f"MEAS: {z:.3f} | EST: {est:.3f}")