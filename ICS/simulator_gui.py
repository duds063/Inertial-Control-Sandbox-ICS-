import tkinter as tk
from tkinter import ttk
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

from system.rigid_body_nd import RigidBody3D
from controllers.pid import PIDND


class SimulatorGUI:
    def __init__(self, root):
        self.root = root
        root.title("Inertial Control Sandbox")

        # ------------------------
        # Simulation parameters
        # ------------------------
        self.dt = 0.01
        self.time = 0.0

        self.reaction_steps = 0
        self.torque_queue = []

        # ------------------------
        # System & controller
        # ------------------------
        self.inertia = np.array([1.0, 1.0, 1.0])

        self.body = RigidBody3D(
            inertia=self.inertia,
            dt=self.dt
        )

        self.pid = PIDND(
            kp=[2.0, 2.0, 2.0],
            kd=[0.5, 0.5, 0.5],
            ki=[0.0, 0.0, 0.0],
            dt=self.dt,
            torque_limit=[1.0, 1.0, 1.0]
        )

        # ------------------------
        # History
        # ------------------------
        self.t_history = []
        self.omega_history = []
        self.torque_history = []

        # ------------------------
        # GUI Controls
        # ------------------------
        self.controls = ttk.Frame(root)
        self.controls.pack(side=tk.LEFT, fill=tk.Y, padx=10)

        def labeled_control(label, from_, to_, default):
            ttk.Label(self.controls, text=label).pack(anchor="w")

            var = tk.DoubleVar(value=default)

            frame = ttk.Frame(self.controls)
            frame.pack(fill=tk.X)

            scale = ttk.Scale(frame, from_=from_, to=to_, variable=var)
            scale.pack(side=tk.LEFT, fill=tk.X, expand=True)

            entry = ttk.Entry(frame, width=7, textvariable=var)
            entry.pack(side=tk.RIGHT)

            return var

        self.kp = labeled_control("Kp", 0, 10, 2)
        self.kd = labeled_control("Kd", 0, 5, 0.5)
        self.ki = labeled_control("Ki", 0, 5, 0.0)

        self.omega_ref_x = labeled_control("ω_ref X", -3, 3, 0)
        self.omega_ref_y = labeled_control("ω_ref Y", -3, 3, 0)
        self.omega_ref_z = labeled_control("ω_ref Z", -3, 3, 0)

        self.torque_limit = labeled_control("Torque limit (Nm)", 0.1, 5, 1)
        self.omega_limit = labeled_control("Omega limit (rad/s)", 0.5, 10, 2)
        self.reaction_time = labeled_control("Reaction time (s)", 0, 0.5, 1.5)

        ttk.Button(
            self.controls,
            text="Reset",
            command=self.reset
        ).pack(pady=10)

        # ------------------------
        # Plot
        # ------------------------
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, figsize=(6, 5))
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        self.canvas.get_tk_widget().pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

        self.root.after(10, self.simulation_step)

    # ------------------------
    def reset(self):
        self.time = 0.0
        self.body.reset()
        self.pid.reset()
        self.torque_queue.clear()
        self.t_history.clear()
        self.omega_history.clear()
        self.torque_history.clear()

    # ------------------------
    def simulation_step(self):
        # Update controller parameters
        kp = self.kp.get()
        kd = self.kd.get()
        ki = self.ki.get()

        self.pid.kp = np.array([kp, kp, kp])
        self.pid.kd = np.array([kd, kd, kd])
        self.pid.ki = np.array([ki, ki, ki])
        self.pid.torque_limit = np.array([
            self.torque_limit.get(),
            self.torque_limit.get(),
            self.torque_limit.get()
        ])

        # Reaction delay
        self.reaction_steps = int(self.reaction_time.get() / self.dt)

        omega = self.body.omega.copy()
        omega_ref = np.array([
            self.omega_ref_x.get(),
            self.omega_ref_y.get(),
            self.omega_ref_z.get()
        ])

        # ------------------------
        # PID control (PURE)
        # ------------------------
        rate_error = omega_ref - omega
        torque_cmd = self.pid.step(rate_error)

        # Reaction delay queue
        self.torque_queue.append(torque_cmd)
        if len(self.torque_queue) > self.reaction_steps:
            torque = self.torque_queue.pop(0)
        else:
            torque = np.zeros(3)

        # Dynamics
        self.body.step(torque)
        omega = self.body.omega

        # Omega saturation
        omega = np.clip(
            omega,
            -self.omega_limit.get(),
            self.omega_limit.get()
        )
        self.body.omega = omega

        # History
        self.time += self.dt
        self.t_history.append(self.time)
        self.omega_history.append(omega.copy())
        self.torque_history.append(torque.copy())

        self.update_plot()
        self.root.after(10, self.simulation_step)

    # ------------------------
    def update_plot(self):
        if len(self.t_history) < 2:
            return

        self.ax1.clear()
        self.ax2.clear()

        omega_hist = np.array(self.omega_history)
        torque_hist = np.array(self.torque_history)

        self.ax1.plot(self.t_history, omega_hist)
        self.ax1.set_title("Angular Velocity (rad/s)")
        self.ax1.grid()

        self.ax2.plot(self.t_history, torque_hist)
        self.ax2.set_title("Torque (Nm)")
        self.ax2.grid()

        self.canvas.draw()


if __name__ == "__main__":
    root = tk.Tk()
    app = SimulatorGUI(root)
    root.mainloop()