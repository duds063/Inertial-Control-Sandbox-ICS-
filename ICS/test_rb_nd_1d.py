from system.rigid_body_nd import RigidBodyND

dt = 0.01
rb = RigidBodyND(
    dim=1,
    inertia=1.0,
    dt=dt
)

print("=== TESTE 1D ===")

for step in range(10):
    rb.apply_torque([1.0])  # torque constante
    state = rb.update()

    print(
        f"STEP {step:02d} | "
        f"theta: {state['theta'][0]:.6f} | "
        f"omega: {state['omega'][0]:.6f}"
    )