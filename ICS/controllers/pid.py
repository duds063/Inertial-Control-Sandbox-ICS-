# controllers/pid.py
import numpy as np

class PIDND:
    def __init__(self, kp, kd, ki=None, dt=0.01, torque_limit=None, d_cutoff=20.0):
        self.kp = kp
        self.kd = kd
        self.ki = ki if ki is not None else [0.0] * len(kp)
        self.dt = dt
        self.dim = len(kp)

        # filtro D
        tau = 1.0 / (2 * np.pi * d_cutoff)
        self.alpha = tau / (tau + dt)
        self.d_filt = [0.0] * self.dim

        self.integral = [0.0] * self.dim

        if torque_limit is None:
            self.torque_limit = [float("inf")] * self.dim
        else:
            self.torque_limit = torque_limit

    def step(self, error, error_dot):
        torque = []

        for i in range(self.dim):
            # integral
            self.integral[i] += error[i] * self.dt

            # filtro passa-baixa no D
            self.d_filt[i] = (
                self.alpha * self.d_filt[i]
                + (1 - self.alpha) * error_dot[i]
            )

            u = (
                - self.kp[i] * error[i]
                - self.kd[i] * self.d_filt[i]
                - self.ki[i] * self.integral[i]
            )

            u_sat = max(
                -self.torque_limit[i],
                min(self.torque_limit[i], u)
            )

            if u != u_sat:
                self.integral[i] -= error[i] * self.dt

            torque.append(u_sat)

        return torque