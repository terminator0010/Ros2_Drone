import math

# Assumindo que esta é a classe que você já tem (ou similar)
class ContatoSolo:
    def __init__(self, rigidez_mola, amortecimento, coef_atrito):
        self.k = rigidez_mola
        self.c = amortecimento
        self.mu = coef_atrito
        self.compressao = 0.0
        self.vel_compressao = 0.0

    def calcular_forca_normal(self):
        # F = k*x + c*v (apenas atua se estiver comprimido)
        if self.compressao > 0:
            return (self.k * self.compressao) + (self.c * self.vel_compressao)
        return 0.0

# 1. Nova classe para Rodas que herda o contato com o solo
class RodaComSuspensao(ContatoSolo):
    def __init__(self, rigidez_mola, amortecimento, coef_atrito, raio, tracionada=False, direcional=False):
        # Inicializa a classe pai (aproveitando o cálculo de amortecimento)
        super().__init__(rigidez_mola, amortecimento, coef_atrito)
        
        # Características exclusivas de rodas
        self.raio = raio
        self.tracionada = tracionada   # Recebe torque do motor?
        self.direcional = direcional   # Pode virar (esterçar)?
        self.angulo_estercamento = 0.0 # Em radianos
        self.velocidade_angular = 0.0

    def calcular_forca_tracao(self, torque_motor):
        """Converte o torque do eixo em força linear (F = Torque / Raio)"""
        if self.tracionada and self.calcular_forca_normal() > 0:
            forca_tracao = torque_motor / self.raio
            # Aqui você aplicaria o limite de tração baseado no atrito (F_max = Normal * mu)
            limite_atrito = self.calcular_forca_normal() * self.mu
            return min(forca_tracao, limite_atrito)
        return 0.0

    def atualizar_direcao(self, angulo):
        if self.direcional:
            self.angulo_estercamento = angulo

# 2. Classe principal do Veículo
class VeiculoSobreRodas:
    def __init__(self, massa, inercia):
        self.massa = massa
        self.inercia = inercia
        self.rodas = []
        # Posição, velocidade, etc.
    
    def adicionar_roda(self, roda: RodaComSuspensao, posicao_relativa: tuple):
        """Adiciona uma roda especificando sua posição em relação ao centro de massa (x, y, z)"""
        self.rodas.append({
            'roda': roda,
            'posicao_relativa': posicao_relativa
        })

    def atualizar_dinamica(self, dt, torque_entrada, angulo_volante):
        forca_resultante_x = 0
        forca_resultante_y = 0
        forca_resultante_z = 0 # Normal / Suspensão

        for item in self.rodas:
            roda = item['roda']
            
            # Atualiza o esterçamento se a roda for direcional
            roda.atualizar_direcao(angulo_volante)
            
            # Aproveita o cálculo de contato com o solo (Amortecimento)
            forca_z = roda.calcular_forca_normal()
            forca_resultante_z += forca_z
            
            # Calcula tração
            forca_x = roda.calcular_forca_tracao(torque_entrada)
            
            # Decompõe a força de tração caso a roda esteja esterçada
            forca_resultante_x += forca_x * math.cos(roda.angulo_estercamento)
            forca_resultante_y += forca_x * math.sin(roda.angulo_estercamento)

        # Após somar todas as forças, você integraria as acelerações (F = m*a)
        # para atualizar a posição do veículo.