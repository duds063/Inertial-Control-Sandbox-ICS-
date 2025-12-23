class PID:
    def __init__(self, kp, ki, kd, dt, u_min=None, u_max=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt

        self.integral = 0.0
        self.prev_error = 0.0

        self.u_min = u_min
        self.u_max = u_max

    def compute(self, setpoint, measurement):
        error = setpoint - measurement

        self.integral += error * self.dt
        derivative = (error - self.prev_error) / self.dt

        u = (
            self.kp * error +
            self.ki * self.integral +
            self.kd * derivative
        )

        self.prev_error = error

        if self.u_min is not None:
            u = max(self.u_min, u)
        if self.u_max is not None:
            u = min(self.u_max, u)

        return u