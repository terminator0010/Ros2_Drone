import math
import random

class AtmosphereModel:
    """
    Simula a atmosfera, incluindo vento constante e rajadas (turbulência).
    """
    def __init__(self, wind_speed_knots=0.0, wind_dir_deg=0.0, turbulence_intensity=0.0):
        # Converte nós para m/s
        self.wind_speed = wind_speed_knots * 0.514444
        # Direção de onde o vento vem
        self.wind_dir = math.radians(wind_dir_deg)
        self.turbulence_intensity = turbulence_intensity
        
        # Componentes do vento em NED (North, East, Down)
        # O vento que vem de 090 (Leste) tem direção do vetor para Oeste (270)
        dir_to = self.wind_dir + math.pi
        self.wind_ned = [
            self.wind_speed * math.cos(dir_to), # North
            self.wind_speed * math.sin(dir_to), # East
            0.0 # Down
        ]
        
        # Estado do filtro da turbulência [North, East, Down]
        self.turb_state = [0.0, 0.0, 0.0]
        
    def get_wind_body(self, dt, phi, theta, psi):
        """
        Gera as componentes do vento e turbulência rotacionadas para o eixo do Corpo.
        
        Parâmetros:
        - dt: Delta time (para a filtragem da turbulência)
        - phi, theta, psi: Ângulos de Euler (Roll, Pitch, Yaw) em radianos
        
        Retorna:
        - wind_u, wind_v, wind_w: Componentes do vento nos eixos X, Y e Z do corpo.
        """
        # Gerar turbulência (Filtro Passa-Baixo num Random Walk / Ruído Branco)
        tau = 1.0 # Constante de tempo (segundos)
        alpha_filter = dt / (tau + dt) if dt > 0 else 1.0
        
        for i in range(3):
            # Turbulência vertical costuma ser ligeiramente menor/diferente, 
            # mas simplificamos aqui assumindo a mesma intensidade.
            noise = random.gauss(0, 1) * self.turbulence_intensity
            self.turb_state[i] = self.turb_state[i] + alpha_filter * (noise - self.turb_state[i])
            
        wind_n = self.wind_ned[0] + self.turb_state[0]
        wind_e = self.wind_ned[1] + self.turb_state[1]
        wind_d = self.wind_ned[2] + self.turb_state[2]
        
        # Matriz de Rotação de NED para Body (Euler rotation matrix)
        # Body = R * NED
        c_phi = math.cos(phi)
        s_phi = math.sin(phi)
        c_th = math.cos(theta)
        s_th = math.sin(theta)
        c_psi = math.cos(psi)
        s_psi = math.sin(psi)
        
        # Elementos da Matriz de Rotação
        R11 = c_th * c_psi
        R12 = c_th * s_psi
        R13 = -s_th
        
        R21 = s_phi * s_th * c_psi - c_phi * s_psi
        R22 = s_phi * s_th * s_psi + c_phi * c_psi
        R23 = s_phi * c_th
        
        R31 = c_phi * s_th * c_psi + s_phi * s_psi
        R32 = c_phi * s_th * s_psi - s_phi * c_psi
        R33 = c_phi * c_th
        
        # Vento no referencial do corpo
        wind_u = R11 * wind_n + R12 * wind_e + R13 * wind_d
        wind_v = R21 * wind_n + R22 * wind_e + R23 * wind_d
        wind_w = R31 * wind_n + R32 * wind_e + R33 * wind_d
        
        return wind_u, wind_v, wind_w
