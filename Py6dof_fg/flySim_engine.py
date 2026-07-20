import math

class RigidBody6DOF:
    def __init__(self, start_lat, start_lon, start_alt, start_phi=0.0, start_theta=0.0, start_psi=0.0):
        self.EARTH_RADIUS = 6378137.0
        self.GRAVITY = 9.81

        # Conversão de Euler para Quaternion inicial
        cy = math.cos(start_psi * 0.5)
        sy = math.sin(start_psi * 0.5)
        cp = math.cos(start_theta * 0.5)
        sp = math.sin(start_theta * 0.5)
        cr = math.cos(start_phi * 0.5)
        sr = math.sin(start_phi * 0.5)

        q0 = cr * cp * cy + sr * sp * sy
        q1 = sr * cp * cy - cr * sp * sy
        q2 = cr * sp * cy + sr * cp * sy
        q3 = cr * cp * sy - sr * sp * cy

        # Estado da Aeronave [u, v, w, p, q, r, lat, lon, alt, q0, q1, q2, q3]
        self.state = [
            50.0, 0.0, 0.0,            # u, v, w (m/s) -> u=50m/s (~100 nós) inicial
            0.0, 0.0, 0.0,             # p, q, r (rad/s)
            start_lat, start_lon, start_alt, 
            q0, q1, q2, q3             # q0, q1, q2, q3 (quaternion)
        ]
        
        self.is_crashed = False

    def _get_air_density(self, alt_m):
        """Calcula a densidade do ar baseada na Atmosfera Padrão (ISA)."""
        T_sea = 288.15 # Kelvin
        P_sea = 101325.0 # Pascal
        L = 0.0065 # Taxa de decaimento de temperatura (K/m)
        R = 287.05 # Constante do gás para o ar

        T = T_sea - L * alt_m
        if T < 216.65: T = 216.65 # Troposfera limite
        
        P = P_sea * (T / T_sea)**5.25588
        rho = P / (R * T)
        return rho

    def _compute_derivatives(self, state, wind_u, wind_v, wind_w, thrust_n, torque_l, p_factor_n, delta_a, delta_e, delta_r, flap_setting=0.0, forces_ground=(0.0, 0.0, 0.0), moments_ground=(0.0, 0.0, 0.0)):
        """
        Derivadas RK4 com Forças Aerodinâmicas e Newton-Euler completas.
        Controles esperados em Radianos: delta_a (aileron), delta_e (elevator), delta_r (rudder)
        """
        u, v, w, p, q, r, lat, lon, alt, q0, q1, q2, q3 = state

        # Conversão de Quaternion para Euler (para telemetria e gear_model)
        sinr_cosp = 2.0 * (q0 * q1 + q2 * q3)
        cosr_cosp = 1.0 - 2.0 * (q1 * q1 + q2 * q2)
        phi = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (q0 * q2 - q3 * q1)
        if abs(sinp) >= 1.0:
            theta = math.copysign(math.pi / 2, sinp)
        else:
            theta = math.asin(sinp)

        siny_cosp = 2.0 * (q0 * q3 + q1 * q2)
        cosy_cosp = 1.0 - 2.0 * (q2 * q2 + q3 * q3)
        psi = math.atan2(siny_cosp, cosy_cosp)

        # Velocidade relativa ao ar
        air_u = u - wind_u
        air_v = v - wind_v
        air_w = w - wind_w

        # 1. Velocidade do Ar e Ângulos Aerodinâmicos
        V_a = math.sqrt(air_u**2 + air_v**2 + air_w**2)
        if V_a < 0.1: V_a = 0.1 # Evita divisão por zero no solo
        
        alpha = math.atan2(air_w, air_u)            # Ângulo de Ataque (rad)
        beta = math.asin(air_v / V_a)           # Ângulo de Derrapagem (rad)

        # 2. Pressão Dinâmica (Q)
        rho = self._get_air_density(alt)
        Q = 0.5 * rho * (V_a**2)

        # 3. Forças Aerodinâmicas (Referencial do Vento)
        # Sustentação (Lift)
        C_L = self.C_L0 + (self.C_L_alpha * alpha)
        Lift = Q * self.wing_area * C_L

        # Arrasto (Drag)
        # A. Efeito Solo (Ground Effect) no Arrasto Induzido
        h = max(0.1, alt)
        K_ge = 1.0
        if h < self.wing_span:
            ratio = (16.0 * h / self.wing_span)**2
            K_ge = ratio / (1.0 + ratio)
            
        C_D_induced = K_ge * (C_L**2) / (math.pi * self.e * self.AR)
        
        # C. Arrasto de Derrapagem (Sideslip / Beta Drag)
        Delta_CD_beta = 0.5 * (math.sin(beta)**2)
        
        # D. Arrasto de Superfícies e Flaps
        Delta_CD_flap = 0.05 * flap_setting
        Delta_CD_surf = 0.01 * (abs(delta_a) + abs(delta_e) + abs(delta_r))
        
        C_D_normal = self.C_D0 + C_D_induced + Delta_CD_beta + Delta_CD_flap + Delta_CD_surf
        
        # B. Arrasto Pós-Estol (Post-Stall Drag)
        alpha_crit = 0.26 # ~15 graus
        if abs(alpha) > alpha_crit:
            C_D_stall = 2.0 * (math.sin(alpha)**2)
            # Blend suave ao longo de ~10 graus (0.17 rad) após o estol
            blend = min(1.0, (abs(alpha) - alpha_crit) / 0.17)
            C_D = C_D_normal * (1.0 - blend) + C_D_stall * blend
        else:
            C_D = C_D_normal

        Drag = Q * self.wing_area * C_D

        # Força Lateral (Sideforce)
        Y_force = Q * self.wing_area * (beta * -0.2)

        # 4. Transformação de Forças do Vento para o Corpo (Eixos X, Y, Z)
        # O Empuxo atua diretamente no eixo X do corpo.
        F_x_aero =  Lift * math.sin(alpha) - Drag * math.cos(alpha)
        F_z_aero = -Lift * math.cos(alpha) - Drag * math.sin(alpha)
        
        # O utilizador solicitou que o peso (mass * g) seja calculado como força primeiro e adicionado ao Sum_Fz.
        # As componentes da gravidade no corpo são:
        g_x = 2.0 * (q1 * q3 - q0 * q2) * self.GRAVITY
        g_y = 2.0 * (q2 * q3 + q0 * q1) * self.GRAVITY
        g_z = (q0**2 - q1**2 - q2**2 + q3**2) * self.GRAVITY

        F_x = F_x_aero + thrust_n + (self.mass * g_x) + forces_ground[0]
        F_y = Y_force + (self.mass * g_y) + forces_ground[1]
        F_z = F_z_aero + (self.mass * g_z) + forces_ground[2]

        # 5. Momentos Aerodinâmicos (L, M, N)
        # Roll Moment (L)
        L_aero = Q * self.wing_area * self.wing_span * (
            (self.C_l_beta * beta) + 
            (self.C_l_p * p * self.wing_span / (2 * V_a)) + 
            (self.C_l_da * delta_a) + 
            (self.C_l_dr * delta_r)
        ) + torque_l

        # Pitch Moment (M)
        M_aero = Q * self.wing_area * self.chord * (
            self.C_m0 + 
            (self.C_m_alpha * alpha) + 
            (self.C_m_q * q * self.chord / (2 * V_a)) + 
            (self.C_m_de * delta_e)
        )

        # Sombra Aerodinâmica (Rudder Blanking)
        # Atenua a estabilidade direcional e eficácia do leme em altos ângulos de ataque
        tail_factor = 1.0
        if alpha > 0.17:  # Aproximadamente 10 graus
            # Fator de redução que decai com o cosseno do ângulo de ataque
            tail_factor = math.cos(alpha)

        eff_C_n_beta = self.C_n_beta * tail_factor
        eff_C_n_dr = self.C_n_dr * tail_factor

        # Yaw Moment (N)
        N_aero = Q * self.wing_area * self.wing_span * (
            (eff_C_n_beta * beta) + 
            (self.C_n_r * r * self.wing_span / (2 * V_a)) + 
            (self.C_n_da * delta_a) + 
            (eff_C_n_dr * delta_r)
        ) + p_factor_n

        # 6. Equações de Movimento Linear (Newton) - Incluindo Coriolis
        dot_u = (F_x / self.mass) - (q * w - r * v)
        dot_v = (F_y / self.mass) - (r * u - p * w)
        dot_w = (F_z / self.mass) - (p * v - q * u)

        # 7. Equações de Movimento Angular (Euler)
        L_total = L_aero + moments_ground[0]
        M_total = M_aero + moments_ground[1]
        N_total = N_aero + moments_ground[2]

        dot_p = (L_total + (self.Iyy - self.Izz) * q * r) / self.Ixx
        dot_q = (M_total + (self.Izz - self.Ixx) * p * r) / self.Iyy
        dot_r = (N_total + (self.Ixx - self.Iyy) * p * q) / self.Izz

        # 8. Cinemática Rotacional (Derivadas de Quaternion)
        dot_q0 = -0.5 * (q1 * p + q2 * q + q3 * r)
        dot_q1 =  0.5 * (q0 * p - q3 * q + q2 * r)
        dot_q2 =  0.5 * (q3 * p + q0 * q - q1 * r)
        dot_q3 =  0.5 * (-q2 * p + q1 * q + q0 * r)

        # 9. Posição na Terra
        # As velocidades de translação são calculadas com os ângulos Euler extraídos do quaternion acima
        v_north = u * math.cos(theta) * math.cos(psi) - v * (math.cos(phi)*math.sin(psi) - math.sin(phi)*math.sin(theta)*math.cos(psi)) + w * (math.sin(phi)*math.sin(psi) + math.cos(phi)*math.sin(theta)*math.cos(psi))
        v_east = u * math.cos(theta) * math.sin(psi) + v * (math.cos(phi)*math.cos(psi) + math.sin(phi)*math.sin(theta)*math.sin(psi)) - w * (math.sin(phi)*math.cos(psi) - math.cos(phi)*math.sin(theta)*math.sin(psi))
        v_down = -u * math.sin(theta) + v * math.sin(phi)*math.cos(theta) + w * math.cos(phi)*math.cos(theta)

        cos_lat = math.cos(lat) if abs(math.cos(lat)) > 0.0001 else 0.0001
        
        dot_lat = v_north / self.EARTH_RADIUS
        dot_lon = v_east / (self.EARTH_RADIUS * cos_lat)
        dot_alt = -v_down 

        return [dot_u, dot_v, dot_w, dot_p, dot_q, dot_r, 
                dot_lat, dot_lon, dot_alt, dot_q0, dot_q1, dot_q2, dot_q3]

    def check_collision_and_limits(self, on_ground):
        if self.is_crashed:
            return

        w = self.state[2]
        theta = self.theta
        phi = self.phi

        if on_ground or self.alt < 1.0:
            # Pouso muito duro (Hard Landing)
            if w > 3.0:
                self.is_crashed = True
            # Batida de Asa (Wing Strike)
            elif abs(phi) > 0.26:
                self.is_crashed = True
            # Batida de Nariz/Hélice (Prop Strike)
            elif theta < -0.08:
                self.is_crashed = True
            # Batida de Cauda (Tail Strike)
            elif theta > 0.26:
                self.is_crashed = True

    def update(self, dt, wind_u, wind_v, wind_w, thrust_n, torque_l, p_factor_n, delta_a, delta_e, delta_r, flap_setting=0.0, forces_ground=(0.0, 0.0, 0.0), moments_ground=(0.0, 0.0, 0.0), on_ground=False):
        """
        Método de Integração RK4.
        delta_a: Aileron (+ direita desce)
        delta_e: Elevator (+ profundor desce = nariz cai)
        delta_r: Rudder (+ leme para a direita = guinada à direita)
        """
        if on_ground:
            self.check_collision_and_limits(on_ground)

        if self.is_crashed:
            # Zera as velocidades lineares e angulares
            self.state[0] = 0.0 # u
            self.state[1] = 0.0 # v
            self.state[2] = 0.0 # w
            self.state[3] = 0.0 # p
            self.state[4] = 0.0 # q
            self.state[5] = 0.0 # r
            return

        y = self.state

        k1 = self._compute_derivatives(y, wind_u, wind_v, wind_w, thrust_n, torque_l, p_factor_n, delta_a, delta_e, delta_r, flap_setting, forces_ground, moments_ground)
        y_k2 = [y[i] + 0.5 * dt * k1[i] for i in range(13)]
        k2 = self._compute_derivatives(y_k2, wind_u, wind_v, wind_w, thrust_n, torque_l, p_factor_n, delta_a, delta_e, delta_r, flap_setting, forces_ground, moments_ground)
        y_k3 = [y[i] + 0.5 * dt * k2[i] for i in range(13)]
        k3 = self._compute_derivatives(y_k3, wind_u, wind_v, wind_w, thrust_n, torque_l, p_factor_n, delta_a, delta_e, delta_r, flap_setting, forces_ground, moments_ground)
        y_k4 = [y[i] + dt * k3[i] for i in range(13)]
        k4 = self._compute_derivatives(y_k4, wind_u, wind_v, wind_w, thrust_n, torque_l, p_factor_n, delta_a, delta_e, delta_r, flap_setting, forces_ground, moments_ground)

        for i in range(13):
            self.state[i] = y[i] + (dt / 6.0) * (k1[i] + 2*k2[i] + 2*k3[i] + k4[i])

        # Normalização rigorosa do Quaternion
        q0, q1, q2, q3 = self.state[9], self.state[10], self.state[11], self.state[12]
        norm = math.sqrt(q0**2 + q1**2 + q2**2 + q3**2)
        if norm > 0.0:
            self.state[9] = q0 / norm
            self.state[10] = q1 / norm
            self.state[11] = q2 / norm
            self.state[12] = q3 / norm

    # Propriedades de leitura
    @property
    def lat(self): return self.state[6]
    @property
    def lon(self): return self.state[7]
    @property
    def alt(self): return self.state[8]

    @property
    def phi(self):
        q0, q1, q2, q3 = self.state[9], self.state[10], self.state[11], self.state[12]
        sinr_cosp = 2.0 * (q0 * q1 + q2 * q3)
        cosr_cosp = 1.0 - 2.0 * (q1 * q1 + q2 * q2)
        return math.atan2(sinr_cosp, cosr_cosp)

    @property
    def theta(self):
        q0, q1, q2, q3 = self.state[9], self.state[10], self.state[11], self.state[12]
        sinp = 2.0 * (q0 * q2 - q3 * q1)
        if abs(sinp) >= 1.0:
            return math.copysign(math.pi / 2, sinp)
        return math.asin(sinp)

    @property
    def psi(self):
        q0, q1, q2, q3 = self.state[9], self.state[10], self.state[11], self.state[12]
        siny_cosp = 2.0 * (q0 * q3 + q1 * q2)
        cosy_cosp = 1.0 - 2.0 * (q2 * q2 + q3 * q3)
        return math.atan2(siny_cosp, cosy_cosp)