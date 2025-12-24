class RigidBodyND:
    def __init__(self, inertia, dt):
        """
        inertia: list[float]  -> momentos de inércia por eixo
        dt: float             -> passo de tempo
        """

        self.I = inertia
        self.dt = dt

        # Número de eixos inferido automaticamente
        self.N = len(inertia)

        # Estado: velocidade angular por eixo
        self.omega = [0.0] * self.N