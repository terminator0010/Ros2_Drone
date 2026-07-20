import os
import sys

if "ROS_DISTRO" not in os.environ:
    print("ROS environment not sourced. Auto-sourcing ROS Humble and restarting...")
    os.execl("/bin/bash", "bash", "-c", f"source /opt/ros/humble/setup.bash && python3 {' '.join(sys.argv)}")

import socket
import time
import math
from fgnetfdm import FGNetFDM
from flySim_engine import RigidBody6DOF
import pygame # Importar pygame para o loop de eventos
from cessna172p_config import CESSNA_172P
from instrumentsPanel import VirtualPanel # <-- IMPORTAÇÃO DO PAINEL
from engine_forces import EngineDynamics
from atmosphere_model import AtmosphereModel
from LandingGearDyna import LandingGearDynamics
from ActuatorDynamics import ActuatorDynamics
from TerrainElevationReader import TerrainElevationReader

# NOTA: Para leitura dinâmica da elevação do terreno, lembre-se de rodar o FlightGear com:
# fgfs --httpd=8080 ...

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

class JoystickNode(Node):
    def __init__(self):
        super().__init__('joystick_ros_node')
        self.subscription = self.create_subscription(
            Joy,
            '/flightgear_attitude_echo',
            self.listener_callback,
            1)
        self.current_axes = []
        self.current_buttons = []

    def listener_callback(self, msg):
        self.current_axes = list(msg.axes)
        self.current_buttons = list(msg.buttons)

# Configurações de Rede e Simulação
UDP_IP = "127.0.0.1"
UDP_PORT = 5500
HZ = 60
DT = 1.0 / HZ

# Sub-stepping de física (alta frequência para estabilidade de contato rígido)
PHYSICS_HZ = 1200
PHYSICS_DT = 1.0 / PHYSICS_HZ
STEPS_PER_FRAME = int(PHYSICS_HZ / HZ)

# Instanciar a Rede
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Posição inicial (Ex: Sorocaba)
lat_inicial = math.radians(-23.4709)
lon_inicial = math.radians(-47.4851)
alt_inicial = 1500.0 # Metros

# Instanciar a Estrutura Binária do FG e o Motor Físico
fdm = FGNetFDM()
sim = RigidBody6DOF(lat_inicial, lon_inicial, alt_inicial)
engine = EngineDynamics(max_thrust=CESSNA_172P["max_thrust"])
atm = AtmosphereModel(wind_speed_knots=0.0, wind_dir_deg=0.0, turbulence_intensity=0.0)
gear = LandingGearDynamics()
actuators = ActuatorDynamics(tau_aileron=0.15, tau_elevator=0.2, tau_rudder=0.25)

# Inicializar a janela do Painel Virtual
painel = VirtualPanel() # <-- INICIALIZAÇÃO DO PAINEL

# Sincronizar propriedades da aeronave com as constantes do Cessna 172P definidas no config
sim.mass = CESSNA_172P["mass"]
sim.wing_area = CESSNA_172P["wing_area"]
sim.wing_span = CESSNA_172P["wing_span"]
sim.chord = CESSNA_172P["chord"]
sim.AR = (sim.wing_span**2) / sim.wing_area
sim.Ixx = CESSNA_172P["ixx"]
sim.Iyy = CESSNA_172P["iyy"]
sim.Izz = CESSNA_172P["izz"]
sim.C_D0 = CESSNA_172P["C_D0"]
sim.C_L_alpha = CESSNA_172P["C_L_alpha"]
sim.C_L0 = CESSNA_172P["C_L0"]
sim.e = CESSNA_172P["e"]
sim.C_m0 = CESSNA_172P["C_m0"]
sim.C_m_alpha = CESSNA_172P["C_m_alpha"]
sim.C_m_q = CESSNA_172P["C_m_q"]
sim.C_m_de = CESSNA_172P["C_m_de"]
sim.C_l_p = CESSNA_172P["C_l_p"]
sim.C_l_da = CESSNA_172P["C_l_da"]
sim.C_l_beta = CESSNA_172P["C_l_beta"]
sim.C_l_dr = CESSNA_172P["C_l_dr"]
sim.C_n_beta = CESSNA_172P["C_n_beta"]
sim.C_n_r = CESSNA_172P["C_n_r"]
sim.C_n_dr = CESSNA_172P["C_n_dr"]
sim.C_n_da = CESSNA_172P["C_n_da"]

print(f"A iniciar simulação 6DOF a {HZ}Hz...")

rclpy.init()
joy_node = JoystickNode()

try:
    terrain_reader = TerrainElevationReader()
    last_terrain_update = time.time()

    while True:
        start_time = time.time()

        # Atualiza elevação do terreno a 10Hz (0.1s)
        if start_time - last_terrain_update >= 0.1:
            gear.ground_elevation = terrain_reader.get_elevation()
            last_terrain_update = start_time

        # ---------------------------------------------------------
        # COMANDOS DA SIMULAÇÃO
        
        for event in pygame.event.get():
            pass  # Mantém o processamento de eventos do pygame para o VirtualPanel não congelar

        rclpy.spin_once(joy_node, timeout_sec=0.0)

        empuxo = 0.0  # Newtons
        target_roll = 0.0  # Comando positivo = rolar para a direita
        target_pitch = 0.0 # Comando positivo = nariz para cima
        target_yaw = 0.0   # Comando positivo = guinar para a direita        
        flap_setting = 0.0 # Comando de flaps (0.0 a 1.0)
        
        # COMANDOS DA SIMULAÇÃO (Hardware-in-the-loop via ROS 2 ou fallback)
        if len(joy_node.current_axes) >= 4:
            def apply_deadzone(val, deadzone=0.05):
                if abs(val) < deadzone: return 0.0
                return (val - deadzone) / (1.0 - deadzone) if val > 0 else (val + deadzone) / (1.0 - deadzone)

            # Extração dos eixos (índices padrão, ajuste se necessário)
            roll_raw = joy_node.current_axes[0] 
            pitch_raw = joy_node.current_axes[1] 
            yaw_raw = joy_node.current_axes[2] 
            throttle_raw = joy_node.current_axes[3]

            roll_input = apply_deadzone(roll_raw)
            pitch_input = apply_deadzone(pitch_raw) # Inversão de Y (pitch) mantida do código original
            yaw_input = apply_deadzone(yaw_raw)     # Inversão de Z (yaw) mantida
            
            # Mapeamento do throttle [-1, 1] para [0, 1]
            throttle_input = (throttle_raw + 1.0) / 2.0            
            
            # Comandos em radianos para a física (baseados nos limites reais da aeronave)
            target_roll = roll_input * math.radians(CESSNA_172P["limits"]["aileron"])
            target_pitch = pitch_input * math.radians(CESSNA_172P["limits"]["elevator"])
            target_yaw = yaw_input * math.radians(CESSNA_172P["limits"]["rudder"])
            
        else:
            # Fallback para valores padrão ou para um voo "autônomo" simples
            throttle_input = 0.7 # 70% de empuxo para manter sustentação
            target_roll = 0.0 # Sem roll
            target_pitch = 0.0 # Sem pitch
            target_yaw = 0.0 # Sem yaw

        # Passar os comandos pelo filtro passa-baixo dos atuadores
        cmd_roll, cmd_pitch, cmd_yaw = actuators.update(target_roll, target_pitch, target_yaw, DT)

        # Valores normalizados (-1 a 1) para a visualização no FlightGear baseados nos comandos filtrados
        surf_aileron = cmd_roll / math.radians(CESSNA_172P["limits"]["aileron"])
        surf_elevator = cmd_pitch / math.radians(CESSNA_172P["limits"]["elevator"])
        surf_rudder = cmd_yaw / math.radians(CESSNA_172P["limits"]["rudder"])
        # ---------------------------------------------------------
        
        u, v, w = sim.state[0], sim.state[1], sim.state[2]
        p, q, r = sim.state[3], sim.state[4], sim.state[5]
        phi, theta, psi = sim.phi, sim.theta, sim.psi

        # Vento no eixo do corpo (calculado com estado antes do update para estimar forcas do motor)
        wind_u, wind_v, wind_w = atm.get_wind_body(DT, phi, theta, psi)
        
        air_u = u - wind_u
        air_v = v - wind_v
        air_w = w - wind_w
        
        v_air = math.sqrt(air_u**2 + air_v**2 + air_w**2)
        tas_knots = v_air * 1.94384
        alpha_rad = math.atan2(air_w, air_u) if air_u != 0 else 0.0

        # Forças do Motor
        thrust, torque, p_factor = engine.calculate(throttle_input, sim.alt, tas_knots, alpha_rad)

        # Forças do Trem de Pouso (Solo)
        # brakes_applied pode ser associado a um input do joystick futuramente
        brakes_applied = 0.0

        # Sub-stepping: Rodar a física múltiplas vezes por frame para evitar instabilidade
        for _ in range(STEPS_PER_FRAME):
            # 1. Calcula as forças do solo no estado atual
            forces_ground, moments_ground, on_ground = gear.calculate_forces(
                sim.alt, sim.theta, sim.phi, 
                sim.state[0], sim.state[1], sim.state[2], 
                sim.state[3], sim.state[4], sim.state[5], 
                brakes_applied
            )
            
            # 2. Avança a física no passo reduzido (PHYSICS_DT) injetando as forças do solo
            sim.update(
                PHYSICS_DT, wind_u, wind_v, wind_w, 
                thrust, torque, p_factor, 
                cmd_roll, cmd_pitch, cmd_yaw, 
                flap_setting, 
                forces_ground=forces_ground, 
                moments_ground=moments_ground,
                on_ground=on_ground
            )

        # Mapeamento do Estado Físico para a Estrutura Binária do FlightGear
        fdm.latitude = sim.lat
        fdm.longitude = sim.lon
        fdm.altitude = sim.alt
        fdm.phi = sim.phi
        fdm.theta = sim.theta
        fdm.psi = sim.psi

        # Extrair velocidades e taxas pós-update
        u, v, w = sim.state[0], sim.state[1], sim.state[2]
        p, q, r = sim.state[3], sim.state[4], sim.state[5]
        
        # Recalcular v_air pós-update para os instrumentos (usando os mesmos wind_u,v,w para simplificar)
        air_u = u - wind_u
        air_v = v - wind_v
        air_w = w - wind_w
        v_air = math.sqrt(air_u**2 + air_v**2 + air_w**2)
        
        # --- CÁLCULOS PARA O PAINEL VIRTUAL ---
        # True Airspeed (TAS) - Velocidade real em relação à massa de ar
        tas_knots = v_air * 1.94384 
        
        # Indicated Airspeed (IAS) - Velocidade medida pelo pitot (afetada pela densidade do ar)
        alt_pes = sim.alt * 3.28084
        densidade_ratio = max(0.0, 1.0 - (0.000006875 * alt_pes)) ** 4.256
        ias_knots = tas_knots * math.sqrt(densidade_ratio)

        # Atualiza a interface gráfica do painel (Pitch, Roll, IAS, TAS, Altitudes)
        alt_msl = alt_pes
        alt_agl = (sim.alt - gear.ground_elevation) * 3.28084
        painel.update(theta, phi, ias_knots, tas_knots, alt_msl, alt_agl)
        # --------------------------------------

        # O instrumento visual do FlightGear (vcas) baterá com o seu painel
        fdm.vcas = ias_knots
        
        fdm.phidot = p                  # rad/s
        fdm.thetadot = q                # rad/s
        fdm.psidot = r                  # rad/s
        fdm.v_body_u = u * 3.28084      # m/s para ft/s
        fdm.v_body_v = v * 3.28084
        fdm.v_body_w = w * 3.28084
        fdm.alpha = math.atan2(air_w, air_u) if air_u != 0 else 0.0
        fdm.beta = math.asin(air_v / v_air) if v_air > 0.1 else 0.0

        # Velocidades locais e Taxa de Subida (Climb Rate)
        v_down = -u * math.sin(theta) + v * math.sin(phi)*math.cos(theta) + w * math.cos(phi)*math.cos(theta)
        fdm.v_north = (u * math.cos(theta) * math.cos(psi)) * 3.28084 # Simplificado
        fdm.climb_rate = -v_down * 3.28084 # m/s para ft/s (usado pelo VSI)
        
        # O FlightGear usa as superfícies visualmente, podemos enviar os comandos
        # normalizados para que a animação 3D use o curso total
        fdm.left_aileron = -surf_aileron
        fdm.right_aileron = surf_aileron
        fdm.elevator = surf_elevator
        fdm.rudder = surf_rudder

        # Empacota em 408 bytes e dispara
        try:
            payload = fdm.pack()
        except OverflowError:
            print("OVERFLOW ERROR in FDM. Variables:")
            for k, v in fdm.__dict__.items():
                print(f"{k}: {v}")
            raise
        sock.sendto(payload, (UDP_IP, UDP_PORT))

        # Controlo de tempo rigoroso para manter os 60Hz determinísticos
        elapsed = time.time() - start_time
        sleep_time = DT - elapsed
        if sleep_time > 0:
            time.sleep(sleep_time)

except KeyboardInterrupt:
    print("\nSimulação 6DOF encerrada.")
finally:
    joy_node.destroy_node()
    rclpy.shutdown()
    sock.close()
    pygame.quit() # Garante que a janela do painel fecha graciosamente