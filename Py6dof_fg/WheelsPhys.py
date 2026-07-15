import numpy as np

class ModeloPneuPacejka:
    def __init__(self, B, C, D, E):
        self.B = B
        self.C = C
        self.D = D
        self.E = E

    def calcular_forca(self, slip):
        # A implementação da "Magic Formula"
        phi = slip
        # Cálculo do argumento do seno
        arg = self.B * phi - self.E * (self.B * phi - np.arctan(self.B * phi))
        return self.D * np.sin(self.C * np.arctan(arg))

# Integração na classe Roda
from GroundPhys import ContatoSolo

class RodaComSuspensaoPacejka(ContatoSolo):
    def __init__(self, *args, modelo_pneu: ModeloPneuPacejka = None, **kwargs):
        super().__init__(*args, **kwargs)
        self.pneu = modelo_pneu

    def calcular_forca_lateral(self, angulo_deriva):
        # angulo_deriva: ângulo entre a direção da roda e a direção do vetor velocidade
        if self.pneu:
            return self.pneu.calcular_forca(angulo_deriva)
        return 0.0