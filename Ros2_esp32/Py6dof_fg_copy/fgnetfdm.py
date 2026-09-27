import struct

class FGNetFDM:
    # A string de formatação define os tipos primitivos em C++ no formato Network (Big-Endian = !)
    # I = uint32_t (4 bytes), i = int32_t (4 bytes)
    # d = double (8 bytes), f = float (4 bytes)
    # Total da formatação abaixo: Exatamente 408 bytes.
    PACK_FORMAT = "! 2I 3d 6f 11f 3f 2f I 4I 36f I 4f I 3I 9f I i f 10f"

    def __init__(self):
        # --- 1. HEADER (2I) ---
        self.version = 24
        self.padding = 0

        # --- 2. POSIÇÕES (3d, 6f) ---
        self.longitude = 0.0       # double: Radianos
        self.latitude = 0.0        # double: Radianos
        self.altitude = 0.0        # double: Metros acima do nível do mar
        self.agl = 0.0             # float: Metros acima do solo
        self.phi = 0.0             # float: Roll (Radianos)
        self.theta = 0.0           # float: Pitch (Radianos)
        self.psi = 0.0             # float: Yaw / True Heading (Radianos)
        self.alpha = 0.0           # float: Ângulo de ataque (Radianos)
        self.beta = 0.0            # float: Ângulo de derrapagem/sideslip (Radianos)

        # --- 3. VELOCIDADES (11f) ---
        self.phidot = 0.0          # float: Roll rate (Radianos/seg)
        self.thetadot = 0.0        # float: Pitch rate (Radianos/seg)
        self.psidot = 0.0          # float: Yaw rate (Radianos/seg)
        self.vcas = 0.0            # float: Calibrated airspeed (Nós)
        self.climb_rate = 0.0      # float: Pés por segundo (ft/s)
        self.v_north = 0.0         # float: Velocidade norte no frame local (ft/s)
        self.v_east = 0.0          # float: Velocidade leste no frame local (ft/s)
        self.v_down = 0.0          # float: Velocidade descida no frame local (ft/s)
        self.v_body_u = 0.0        # float: Velocidade eixo U corpo (ft/s)
        self.v_body_v = 0.0        # float: Velocidade eixo V corpo (ft/s)
        self.v_body_w = 0.0        # float: Velocidade eixo W corpo (ft/s)

        # --- 4. ACELERAÇÕES (3f) ---
        self.a_x_pilot = 0.0       # float: Aceleração X corpo (ft/s²)
        self.a_y_pilot = 0.0       # float: Aceleração Y corpo (ft/s²)
        self.a_z_pilot = 0.0       # float: Aceleração Z corpo (ft/s²)

        # --- 5. ESTOL E DERRAPAGEM (2f) ---
        self.stall_warning = 0.0   # float: Aviso de estol (0.0 a 1.0)
        self.slip_deg = 0.0        # float: Deflexão da bolha de slip (Graus)

        # --- 6. MOTORES (I, 4I, 36f) ---
        self.num_engines = 1       # uint32: Quantidade de motores válidos
        self.eng_state = [2]*4     # 4x uint32: 0=Off, 1=Cranking, 2=Running
        self.rpm = [0.0]*4         # 4x float: Rotações por minuto
        self.fuel_flow = [0.0]*4   # 4x float: Galões por hora
        self.fuel_px = [0.0]*4     # 4x float: Pressão do combustível (psi)
        self.egt = [0.0]*4         # 4x float: Temperatura exaustão (Fahrenheit)
        self.cht = [0.0]*4         # 4x float: Temperatura cabeçote (Fahrenheit)
        self.mp_osi = [0.0]*4      # 4x float: Pressão de admissão
        self.tit = [0.0]*4         # 4x float: Temperatura entrada turbina
        self.oil_temp = [0.0]*4    # 4x float: Temp óleo (Fahrenheit)
        self.oil_px = [0.0]*4      # 4x float: Pressão óleo (psi)

        # --- 7. COMBUSTÍVEL / CONSUMÍVEIS (I, 4f) ---
        self.num_tanks = 1         # uint32: Quantidade de tanques
        self.fuel_quantity = [0.0]*4 # 4x float: Quantidade no tanque

        # --- 8. TREM DE POUSO (I, 3I, 9f) ---
        self.num_wheels = 3        # uint32: Quantidade de rodas
        self.wow = [0]*3           # 3x uint32: Weight on Wheels (Booleano 0 ou 1)
        self.gear_pos = [0.0]*3    # 3x float: Posição trem de pouso (0.0 a 1.0)
        self.gear_steer = [0.0]*3  # 3x float: Esterçamento da roda (Graus)
        self.gear_compression = [0.0]*3 # 3x float: Compressão do amortecedor (Metros)

        # --- 9. AMBIENTE (I, i, f) ---
        self.cur_time = 0          # uint32: Unix time atual
        self.warp = 0              # int32: Offset do tempo
        self.visibility = 0.0      # float: Visibilidade (Metros)

        # --- 10. SUPERFÍCIES DE CONTROLE (10f) ---
        # A maioria varia de -1.0 a 1.0, ou 0.0 a 1.0 dependendo da superfície
        self.elevator = 0.0
        self.elevator_trim_tab = 0.0
        self.left_flap = 0.0
        self.right_flap = 0.0
        self.left_aileron = 0.0
        self.right_aileron = 0.0
        self.rudder = 0.0
        self.nose_wheel = 0.0
        self.speedbrake = 0.0
        self.spoilers = 0.0

    def pack(self) -> bytes:
        """
        Coleta todas as variáveis locais e as empacota (serialize) de 
        forma bruta no padrão Big-Endian exigido pelo FlightGear.
        """
        # Desempacotando as listas para envio sequencial
        eng_state_tuple = tuple(self.eng_state)
        rpm_tuple = tuple(self.rpm)
        fuel_flow_tuple = tuple(self.fuel_flow)
        fuel_px_tuple = tuple(self.fuel_px)
        egt_tuple = tuple(self.egt)
        cht_tuple = tuple(self.cht)
        mp_osi_tuple = tuple(self.mp_osi)
        tit_tuple = tuple(self.tit)
        oil_temp_tuple = tuple(self.oil_temp)
        oil_px_tuple = tuple(self.oil_px)
        
        fuel_qty_tuple = tuple(self.fuel_quantity)
        
        wow_tuple = tuple(self.wow)
        gear_pos_tuple = tuple(self.gear_pos)
        gear_steer_tuple = tuple(self.gear_steer)
        gear_comp_tuple = tuple(self.gear_compression)

        return struct.pack(
            self.PACK_FORMAT,
            self.version, self.padding,
            self.longitude, self.latitude, self.altitude,
            self.agl, self.phi, self.theta, self.psi, self.alpha, self.beta,
            self.phidot, self.thetadot, self.psidot, self.vcas, self.climb_rate,
            self.v_north, self.v_east, self.v_down, self.v_body_u, self.v_body_v, self.v_body_w,
            self.a_x_pilot, self.a_y_pilot, self.a_z_pilot,
            self.stall_warning, self.slip_deg,
            self.num_engines,
            *eng_state_tuple, *rpm_tuple, *fuel_flow_tuple, *fuel_px_tuple,
            *egt_tuple, *cht_tuple, *mp_osi_tuple, *tit_tuple, *oil_temp_tuple, *oil_px_tuple,
            self.num_tanks,
            *fuel_qty_tuple,
            self.num_wheels,
            *wow_tuple, *gear_pos_tuple, *gear_steer_tuple, *gear_comp_tuple,
            self.cur_time, self.warp, self.visibility,
            self.elevator, self.elevator_trim_tab, self.left_flap, self.right_flap,
            self.left_aileron, self.right_aileron, self.rudder, self.nose_wheel,
            self.speedbrake, self.spoilers
        )