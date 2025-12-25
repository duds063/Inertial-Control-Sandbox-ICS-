import numpy as np
from math_utils.quaternion import quat_mul, quat_norm

class RigidBodyND:
    def __init__(self, dim, inertia, dt):
        self.dim = dim
        self.dt = dt

        self.I = inertia
        self.I_inv = [1.0 / i for i in inertia]

        self.omega = [0.0] * dim
        self.theta = [0.0] * dim  # integração simples (debug)

    def cross(self, a, b):
        return [
            a[1]*b[2] - a[2]*b[1],
            a[2]*b[0] - a[0]*b[2],
            a[0]*b[1] - a[1]*b[0]
        ]

    def step(self, torque):
        if self.dim == 3:
            Iw = [
                self.I[0] * self.omega[0],
                self.I[1] * self.omega[1],
                self.I[2] * self.omega[2],
            ]

            gyro = self.cross(self.omega, Iw)

            domega = [
                self.I_inv[i] * (torque[i] - gyro[i])
                for i in range(3)
            ]
        else:
            # modelo simplificado (correto para ND ≠ 3)
            domega = [
                self.I_inv[i] * torque[i]
                for i in range(self.dim)
            ]

        # integração
        for i in range(self.dim):
            self.omega[i] += domega[i] * self.dt
            self.theta[i] += self.omega[i] * self.dt
import numpy as np

class RigidBody3D:
    def __init__(self, inertia, dt):
        self.dt = dt

        inertia = np.array(inertia)
        self.I = np.diag(inertia)
        self.I_inv = np.linalg.inv(self.I)

        self.omega = np.zeros(3)
        self.q = np.array([1.0, 0.0, 0.0, 0.0])

    def step(self, torque):
        # Dinâmica
        gyro = np.cross(self.omega, self.I @ self.omega)
        domega = self.I_inv @ (torque - gyro)
        self.omega += domega * self.dt

        # Cinemática (quaternion)
        omega_quat = np.array([0.0, *self.omega])
        q_dot = 0.5 * quat_mul(self.q, omega_quat)

        self.q += q_dot * self.dt
        self.q = quat_norm(self.q)
