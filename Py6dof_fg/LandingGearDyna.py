import math
from LandingGearPhys import LandingGear
from WheelsPhys import ModeloPneuPacejka

class LandingGearDynamics:
    """
    Modelo de reações de solo (Ground Reactions) para um trem de pouso triciclo,
    agora utilizando a nova arquitetura orientada a objetos (ContatoSolo / RodaComSuspensaoPacejka / LandingGear).
    """
    def __init__(self):
        # Configuração das rodas em relação ao Centro de Gravidade (CG)
        # Pneu genérico de asfalto para a Fórmula de Pacejka
        pneu_asfalto = ModeloPneuPacejka(B=10.0, C=1.9, D=1.0, E=0.97)
        
        self.nose_gear = LandingGear(rigidez_mola=30000, amortecimento=3000, coef_atrito=0.6, freio_maximo=0.0, modelo_pneu=pneu_asfalto)
        self.left_gear = LandingGear(rigidez_mola=50000, amortecimento=5000, coef_atrito=0.8, freio_maximo=10000, modelo_pneu=pneu_asfalto)
        self.right_gear = LandingGear(rigidez_mola=50000, amortecimento=5000, coef_atrito=0.8, freio_maximo=10000, modelo_pneu=pneu_asfalto)
        
        # Eixos: X (Frente), Y (Direita), Z (Baixo) - Medidas aproximadas Cessna 172
        self.wheels = {
            "nose":  {"obj": self.nose_gear,  "pos": [ 1.2,  0.0,  1.5]},
            "left":  {"obj": self.left_gear,  "pos": [-0.5, -1.2,  1.5]},
            "right": {"obj": self.right_gear, "pos": [-0.5,  1.2,  1.5]}
        }
        
        # Elevação do solo (0 = nível do mar)
        self.ground_elevation = 0.0 

    def calculate_forces(self, altitude, pitch, roll, u, v, w, p, q, r, brakes_applied=0.0):
        """
        Calcula as forças e momentos injetados pelo chão no avião (Eixos do Corpo).
        """
        Fx_total, Fy_total, Fz_total = 0.0, 0.0, 0.0
        Mx_total, My_total, Mz_total = 0.0, 0.0, 0.0
        
        # Matriz de rotação simplificada
        cos_p = math.cos(pitch)
        sin_p = math.sin(pitch)
        cos_r = math.cos(roll)
        sin_r = math.sin(roll)

        on_ground = False

        for name, wheel_data in self.wheels.items():
            wx, wy, wz = wheel_data["pos"]
            gear_obj = wheel_data["obj"]
            
            # 1. Distância vertical do CG até esta roda no eixo da Terra
            z_earth_offset = -wx * sin_p + wy * sin_r * cos_p + wz * cos_p * cos_r
            
            # Altitude exata da roda
            wheel_alt = altitude - z_earth_offset
            
            if wheel_alt < self.ground_elevation:
                on_ground = True
                
                # 2. Compressão do amortecedor
                # Atualiza o estado da compressão na classe base
                gear_obj.compressao = self.ground_elevation - wheel_alt
                
                # Velocidade vertical local da roda (w_local é positivo descendo)
                w_local = w - q * wx + p * wy
                gear_obj.vel_compressao = w_local
                
                # Força Normal extraída do modelo físico (sempre positiva empurrando)
                Fz_normal = gear_obj.calcular_forca_normal()
                
                # Fz é negativo no corpo (empurra para cima, -Z)
                Fz = -Fz_normal
                
                # 3. Fricção Longitudinal (X) e Lateral (Y)
                # Velocidade local nos eixos
                u_local = u - r * wy + q * wz
                v_local = v + r * wx - p * wz
                
                # Fricção Longitudinal
                # Se freios aplicados e o pneu possui freio, usa a lógica de LandingGear
                if brakes_applied > 0 and gear_obj.freio_maximo > 0:
                    forca_freio = gear_obj.aplicar_freio(brakes_applied)
                    Fx = -math.copysign(1, u_local) * forca_freio
                else:
                    # Rolling resistance natural (atrito de rolamento 0.02)
                    Fx = -math.copysign(1, u_local) * 0.02 * Fz_normal
                
                # Previne jitter se quase parado
                if abs(u_local) < 0.1: 
                    Fx = -u_local * 1000 
                
                # Fricção Lateral usando WheelsPhys (Fórmula de Pacejka)
                # Cálculo do slip angle (angulo de deriva)
                if abs(u_local) > 0.1:
                    slip_angle = math.atan2(v_local, abs(u_local))
                else:
                    slip_angle = 0.0
                
                # A Fórmula Mágica retorna um multiplicador (ex: de 0.0 a ~1.0)
                pacejka_mult = gear_obj.calcular_forca_lateral(slip_angle)
                
                # Multiplicamos pela Força Normal e Coeficiente de Atrito para a força real
                Fy_pacejka = pacejka_mult * Fz_normal * gear_obj.mu
                
                # A força deve se opor à velocidade lateral
                Fy = -math.copysign(1, v_local) * abs(Fy_pacejka)
                
                if abs(v_local) < 0.1:
                    Fy = -v_local * 1000 # Previne jitter lateral
                
                # 4. Somar forças totais
                Fx_total += Fx
                Fy_total += Fy
                Fz_total += Fz
                
                # 5. Momentos gerados por estas forças
                Mx_total += (wy * Fz - wz * Fy)
                My_total += (wz * Fx - wx * Fz)
                Mz_total += (wx * Fy - wy * Fx)
                
            else:
                # Se a roda saiu do chão, zera a compressão para que Fz=0 no próximo cálculo
                gear_obj.compressao = 0.0
                gear_obj.vel_compressao = 0.0

        return (Fx_total, Fy_total, Fz_total), (Mx_total, My_total, Mz_total), on_ground