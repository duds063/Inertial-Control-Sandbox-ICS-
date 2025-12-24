import random

class GyroND:
    def __init__(self, dim, noise_std, bias):
        """
        Giroscópio genérico ND

        dim       : int
        noise_std : float | list[float]
        bias      : float | list[float]
        """

        self.dim = dim

        # Ruído por eixo
        if isinstance(noise_std, (int, float)):
            self.noise_std = [float(noise_std)] * dim
        else:
            if len(noise_std) != dim:
                raise ValueError("noise_std must have length = dim")
            self.noise_std = [float(n) for n in noise_std]

        # Bias por eixo
        if isinstance(bias, (int, float)):
            self.bias = [float(bias)] * dim
        else:
            if len(bias) != dim:
                raise ValueError("bias must have length = dim")
            self.bias = [float(b) for b in bias]

    def read(self, true_omega):
        """
        Retorna medição do giroscópio

        true_omega : list[float] de tamanho dim
        """
        if len(true_omega) != self.dim:
            raise ValueError("true_omega must have length = dim")

        measurement = []

        for i in range(self.dim):
            noise = random.gauss(0.0, self.noise_std[i])
            z = true_omega[i] + self.bias[i] + noise
            measurement.append(z)

        return measurement