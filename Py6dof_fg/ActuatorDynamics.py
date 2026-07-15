class ActuatorDynamics:
    """
    Simula o atraso mecânico/hidráulico das superfícies de controlo.
    Evita que uma mudança instantânea no joystick cause uma mudança instantânea na asa.
    """
    def __init__(self, tau_aileron=0.15, tau_elevator=0.2, tau_rudder=0.25):
        # Constantes de tempo (tau) em segundos. 
        # Quanto maior o tau, mais "pesado" e lento é o comando.
        self.tau_a = tau_aileron
        self.tau_e = tau_elevator
        self.tau_r = tau_rudder
        
        # Posições atuais reais das superfícies (radianos)
        self.pos_a = 0.0
        self.pos_e = 0.0
        self.pos_r = 0.0

    def update(self, cmd_a, cmd_e, cmd_r, dt):
        """
        Recebe os comandos alvo do piloto e move as superfícies gradualmente.
        Utiliza a discretização de um filtro passa-baixo de 1ª ordem.
        """
        # Fatores de suavização (Alpha) para cada eixo
        alpha_a = dt / (self.tau_a + dt)
        alpha_e = dt / (self.tau_e + dt)
        alpha_r = dt / (self.tau_r + dt)
        
        # Atualiza a posição real aproximando-a do comando
        self.pos_a += alpha_a * (cmd_a - self.pos_a)
        self.pos_e += alpha_e * (cmd_e - self.pos_e)
        self.pos_r += alpha_r * (cmd_r - self.pos_r)
        
        return self.pos_a, self.pos_e, self.pos_r