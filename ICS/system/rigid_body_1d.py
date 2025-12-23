
class RigidBody1D:
    def __init__(self, omega0=0.0, inertia=1.0):
        self.omega = omega0
        self.I = inertia

    def update(self, torque, dt):
        alpha = torque / self.I
        self.omega += alpha * dt
        return self.omega