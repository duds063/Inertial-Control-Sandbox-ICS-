# controllers/attitude.py
import numpy as np
from math_utils.quaternion import quat_mul, quat_norm

class AttitudeController:
    def __init__(self, kp=2.0, kd=0.5):
        self.kp = kp
        self.kd = kd

    def step(self, q_ref, q, omega):
        q_ref = np.array(q_ref, dtype=float)
        q = np.array(q, dtype=float)
        omega = np.array(omega, dtype=float)

        # garante unitários
        q_ref = quat_norm(q_ref)
        q = quat_norm(q)

        # q_err = q_ref ⊗ q⁻¹  (erro no body frame)
        q_conj = np.array([q[0], -q[1], -q[2], -q[3]])
        q_err = quat_mul(q_ref, q_conj)

        # menor rotação
        if q_err[0] < 0:
            q_err = -q_err

        # vetor de erro de atitude (body frame)
        e_rot = q_err[1:]

        # lei clássica SO(3)
        omega_ref = self.kp * e_rot - self.kd * omega

        return omega_ref