# controllers/pid.py
import numpy as np
class PIDND:
    def __init__(self, kp, kd, ki, dt, torque_limit):
        self.kp = np.array(kp)
        self.kd = np.array(kd)
        self.ki = np.array(ki)
        self.dt = dt
        self.torque_limit = np.array(torque_limit)

        self.integral = np.zeros(len(kp))

    def reset(self):
        self.integral[:] = 0.0

    def step(self, error):
        # Integral
        self.integral += error * self.dt

        # PID law
        torque = (
            self.kp * error
            + self.ki * self.integral
            - self.kd * error  # derivativo implícito (−ω)
        )

        # Anti-windup (clamp-based)
        saturated = np.abs(torque) > self.torque_limit
        self.integral[saturated] -= error[saturated] * self.dt

        # Saturation
        torque = np.clip(torque, -self.torque_limit, self.torque_limit)
        return torque