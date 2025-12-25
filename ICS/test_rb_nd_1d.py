from math_utils.quaternion import quat_mul, quat_norm
import numpy as np
from system.rigid_body_nd import RigidBody3D
rb = RigidBody3D(np.array([1.0, 1.0, 1.0]), dt=0.01)

for i in range(200):
    rb.apply_torque(np.array([0.1, 0.0, 0.0]))
    rb.step()

    print(rb.q)

rb = RigidBody3D(np.array([1.0, 2.0, 3.0]), dt=0.01)
rb.omega = np.array([1.0, 0.5, 0.0])

for i in range(500):
    rb.step()
    print(rb.omega)