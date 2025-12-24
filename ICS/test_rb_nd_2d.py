from system.rigid_body_nd import RigidBodyND

dt = 0.01
rb = RigidBodyND(
    dim=2,
    inertia=[1.0, 2.0],
    dt=dt
)

print("=== TESTE 2D ===")

for step in range(10):
    torque = [1.0, -0.5]   # torques independentes
    rb.apply_torque(torque)
    state = rb.update()

    print(
        f"STEP {step:02d} | "
        f"theta: {state['theta']} | "
        f"omega: {state['omega']}"
    )