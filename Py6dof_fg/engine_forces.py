import math

class EngineDynamics:
    """
    Simula as dinâmicas e forças geradas pelo motor e pela hélice.
    """
    def __init__(self, max_thrust):
        self.max_thrust = max_thrust
        self.prop_diameter = 1.9 # metros (típico de um Cessna 172)
        # Constantes de torque e p-factor ajustáveis para calibração
        self.torque_constant = 0.05
        self.p_factor_constant = 0.08
        
    def _get_air_density(self, alt_m):
        """Calcula a densidade do ar baseada na Atmosfera Padrão (ISA)."""
        T_sea = 288.15 # Kelvin
        P_sea = 101325.0 # Pascal
        L = 0.0065 # Taxa de decaimento de temperatura (K/m)
        R = 287.05 # Constante do gás para o ar

        T = T_sea - L * alt_m
        if T < 216.65: 
            T = 216.65 # Troposfera limite
        
        P = P_sea * (T / T_sea)**5.25588
        rho = P / (R * T)
        return rho

    def calculate(self, throttle, alt_m, tas_knots, alpha_rad):
        """
        Calcula as forças produzidas pelo motor (Empuxo, Torque, P-Factor).
        
        Parâmetros:
        - throttle: percentagem de acelerador [0.0, 1.0]
        - alt_m: altitude atual em metros
        - tas_knots: True Airspeed em nós
        - alpha_rad: Ângulo de ataque em radianos
        
        Retorna:
        - thrust (N): Empuxo na direção X do corpo
        - torque (N.m): Momento de rolagem (Roll) 
        - p_factor (N.m): Momento de guinada (Yaw)
        """
        rho = self._get_air_density(alt_m)
        rho_sea = 1.225
        density_ratio = rho / rho_sea
        
        tas_ms = tas_knots * 0.514444
        
        # Modelo simplificado de eficiência da hélice (diminui à medida que o TAS aumenta)
        # Uma hélice de passo fixo perde eficiência em altas velocidades.
        prop_efficiency = max(0.1, 1.0 - (tas_ms / 120.0))
        
        # Empuxo ajustado pela densidade (perda de potência em altitude) e eficiência
        thrust = self.max_thrust * throttle * density_ratio * prop_efficiency
        
        # Torque (Momento de Roll - eixo X)
        # Assume-se que a hélice gira no sentido horário visto do cockpit.
        # A reação no avião é um momento no sentido anti-horário (Roll para a esquerda, negativo).
        torque = -thrust * self.torque_constant * self.prop_diameter
        
        # P-Factor (Momento de Yaw - eixo Z)
        # Em alto ângulo de ataque, a pá descendente tem maior ângulo de ataque relativo,
        # produzindo mais tração, resultando numa tendência de guinada para a esquerda (negativo).
        p_factor = -thrust * alpha_rad * self.p_factor_constant * self.prop_diameter
        
        return thrust, torque, p_factor
