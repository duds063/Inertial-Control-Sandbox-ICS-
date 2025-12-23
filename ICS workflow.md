ICS workflow
# 1. System
system = RigidBody1D(inertia=J, dt=dt)

# 2. Sensor
imu = IMUSensor(
    system=system,
    gyro_noise=sigma_g,
    accel_noise=sigma_a,
    bias_drift=True
)

# 3. Estimator
if filter == 1:
    estimator = KalmanEstimator(system, imu)
elif filter == 2:
    estimator = GaussianSmoother()
elif filter == 3:
    estimator = WienerEstimator()
elif filter == 4:
    estimator = AdaptiveEstimator(type="LMS" or "RLS")

# 4. Controller
controller = PID(kp=Kp, ki=Ki, kd=Kd)

# 5. Simulation loop
for t in simulation_time:
    measured = imu.read(system.state)
    estimated = estimator.update(measured)
    control = controller.compute(estimated)
    system.apply(control)
    system.update()
