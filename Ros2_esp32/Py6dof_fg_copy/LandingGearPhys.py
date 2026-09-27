from GroundPhys import ContatoSolo
from WheelsPhys import RodaComSuspensaoPacejka

# A classe LandingGear agora é filha de RodaComSuspensaoPacejka (que herda de ContatoSolo)
class LandingGear(RodaComSuspensaoPacejka):
    def __init__(self, rigidez_mola, amortecimento, coef_atrito, freio_maximo, modelo_pneu=None):
        super().__init__(rigidez_mola, amortecimento, coef_atrito, modelo_pneu=modelo_pneu)
        self.freio_maximo = freio_maximo
        self.recolhido = False
        
    def aplicar_freio(self, intensidade_freio):
        """Aeronaves freiam pelas rodas, mas aceleram pelo motor principal"""
        if self.recolhido or self.calcular_forca_normal() <= 0:
            return 0.0
            
        # intensidade_freio de 0.0 a 1.0
        forca_frenagem = self.freio_maximo * intensidade_freio
        limite_atrito = self.calcular_forca_normal() * self.mu
        
        return min(forca_frenagem, limite_atrito)
        
    def acionar_trem_de_pouso(self, estado: bool):
        self.recolhido = estado
        if self.recolhido:
            self.compressao = 0.0 # Zera a mola ao recolher