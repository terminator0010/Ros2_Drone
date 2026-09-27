import pygame

class JoystickController:
    """
    Gerencia a leitura de inputs de um joystick conectado.
    Utiliza a biblioteca pygame para detecção e leitura de eixos.
    """
    def __init__(self, invert_y=False, invert_z=False, invert_throttle=False):
        pygame.init()
        pygame.joystick.init()
        self.joystick = None
        self.invert_y = invert_y
        self.invert_z = invert_z
        self.invert_throttle = invert_throttle
        self._connect_joystick()

        # Mapeamento de eixos (pode variar dependendo do joystick)
        # Estes são valores comuns para gamepads, ajuste conforme necessário para o seu joystick.
        # Você pode usar ferramentas como 'jstest' no Linux ou testar os eixos no seu OS
        # para identificar os índices corretos.
        self.AXIS_ROLL = 0      # Geralmente o eixo X do stick esquerdo
        self.AXIS_PITCH = 1     # Geralmente o eixo Y do stick esquerdo
        self.AXIS_YAW = 2       # Geralmente o eixo X do stick direito ou um eixo de torção
        self.AXIS_THROTTLE = 3  # Pode ser um slider, ou um trigger combinado. Assumindo um eixo.

        # Deadzone para evitar "drift" (pequenos movimentos não intencionais do joystick)
        self.DEADZONE = 0.05

        print("JoystickController inicializado.")
        if self.joystick:
            print(f"Joystick conectado: {self.joystick.get_name()} ({self.joystick.get_numaxes()} eixos, {self.joystick.get_numbuttons()} botões)")
        else:
            print("Nenhum joystick encontrado. A simulação usará valores padrão.")

    def _connect_joystick(self):
        """Tenta conectar ao primeiro joystick disponível."""
        if pygame.joystick.get_count() > 0:
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
        else:
            self.joystick = None

    def handle_event(self, event):
        """
        Processa eventos pygame relacionados ao joystick (conexão/desconexão).
        Deve ser chamado no loop principal de eventos do pygame.
        """
        if event.type == pygame.JOYDEVICEADDED:
            print(f"Novo joystick detectado: {event.device_index}")
            self._connect_joystick()
            if self.joystick:
                print(f"Joystick conectado: {self.joystick.get_name()}")
        elif event.type == pygame.JOYDEVICEREMOVED:
            print(f"Joystick desconectado: {event.device_index}")
            self.joystick = None
            self._connect_joystick() # Tenta reconectar se outro joystick estiver disponível
            if self.joystick:
                print(f"Reconectado a: {self.joystick.get_name()}")
            else:
                print("Nenhum joystick disponível após desconexão.")

    def _map_axis_to_range(self, value, deadzone=0.05):
        """
        Mapeia o valor bruto do eixo do joystick (-1 a 1) para um range de saída,
        aplicando uma deadzone.
        """
        if abs(value) < deadzone:
            return 0.0
        
        # Normaliza o valor fora da deadzone para o range [-1, 1]
        if value > 0:
            normalized_value = (value - deadzone) / (1.0 - deadzone)
        else:
            normalized_value = (value + deadzone) / (1.0 - deadzone)
        
        return normalized_value

    def get_controls(self):
        """
        Lê o estado atual dos eixos do joystick e retorna os comandos de controle normalizados.
        Retorna (throttle_0_1, roll_n1_1, pitch_n1_1, yaw_n1_1)
        """
        throttle, roll, pitch, yaw = 0.0, 0.0, 0.0, 0.0

        if self.joystick and self.joystick.get_init():
            try:
                roll = self._map_axis_to_range(self.joystick.get_axis(self.AXIS_ROLL), self.DEADZONE) if self.joystick.get_numaxes() > self.AXIS_ROLL else 0.0
                # Lê o eixo de pitch e aplica a inversão baseada na configuração
                pitch_raw = self._map_axis_to_range(self.joystick.get_axis(self.AXIS_PITCH), self.DEADZONE) if self.joystick.get_numaxes() > self.AXIS_PITCH else 0.0
                pitch = -pitch_raw if self.invert_y else pitch_raw
                # Lê o eixo de yaw e aplica a inversão baseada na configuração
                yaw_raw = self._map_axis_to_range(self.joystick.get_axis(self.AXIS_YAW), self.DEADZONE) if self.joystick.get_numaxes() > self.AXIS_YAW else 0.0
                yaw = -yaw_raw if self.invert_z else yaw_raw
                # Lê o eixo de throttle e mapeia para [0, 1]
                throttle_raw = self.joystick.get_axis(self.AXIS_THROTTLE) if self.joystick.get_numaxes() > self.AXIS_THROTTLE else -1.0 # Assume -1 se não houver eixo
                throttle = (throttle_raw + 1.0) / 2.0 # Mapeia de [-1, 1] para [0, 1]
                throttle = -throttle if self.invert_throttle else throttle # Inverte se necessário
            except pygame.error as e:
                print(f"Erro ao ler joystick: {e}. Verifique a conexão.")
                self.joystick = None # Marca como desconectado
        
        return throttle, roll, pitch, yaw

    def is_connected(self):
        """Verifica se um joystick está atualmente conectado e inicializado."""
        return self.joystick is not None and self.joystick.get_init()

    def __del__(self):
        """Garante que o pygame seja encerrado corretamente."""
        pygame.joystick.quit()
        pygame.quit()