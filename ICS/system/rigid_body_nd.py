class RigidBodyND:
    def __init__(self, dim, inertia, dt):
        """
        Corpo rígido genérico ND (1D, 2D, 3D, ...)

        dim     : int            -> número de dimensões
        inertia : float | list   -> inércia por eixo
        dt      : float          -> passo de simulação
        """

        self.dim = dim
        self.dt = dt

        # Inércia por eixo
        if isinstance(inertia, (int, float)):
            self.I = [float(inertia)] * dim
        else:
            if len(inertia) != dim:
                raise ValueError("Inertia must have length = dim")
            self.I = [float(i) for i in inertia]

        # Estado
        self.theta = [0.0] * dim
        self.omega = [0.0] * dim

    def apply_torque(self, torque):
        """
        Aplica torque (vetorial) ao corpo
        torque : list[float] de tamanho dim
        """
        if len(torque) != self.dim:
            raise ValueError("Torque must have length = dim")

        for i in range(self.dim):
            alpha = torque[i] / self.I[i]
            self.omega[i] += alpha * self.dt

    def update(self):
        """
        Integra o estado no tempo
        """
        for i in range(self.dim):
            self.theta[i] += self.omega[i] * self.dt

        return self.get_state()

    def get_state(self):
        """
        Retorna uma cópia segura do estado
        """
        return {
            "theta": self.theta.copy(),
            "omega": self.omega.copy()
        }