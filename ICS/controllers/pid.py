class PIDND:
    def __init__(self, kp, kd, ki=None, dt=0.01):
        self.kp = kp
        self.kd = kd
        self.ki = ki if ki is not None else [0.0] * len(kp)
        self.dt = dt

        self.integral = [0.0] * len(kp)

    def step(self, error, error_dot=None):
        if error_dot is None:
            error_dot = [0.0] * len(error)

        torque = []

        for i in range(len(error)):
            self.integral[i] += error[i] * self.dt

            u = (
                - self.kp[i] * error[i]
                - self.kd[i] * error_dot[i]
                - self.ki[i] * self.integral[i]
            )

            torque.append(u)

        return torque