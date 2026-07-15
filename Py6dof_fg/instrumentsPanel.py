import pygame
import math

class VirtualPanel:
    def __init__(self, width=900, height=300):
        # Inicializa o subsistema de vídeo do pygame
        if not pygame.get_init():
            pygame.init()
        if not pygame.font.get_init():
            pygame.font.init()

        self.width = width
        self.height = height
        
        # Cria a janela principal do painel
        self.screen = pygame.display.set_mode((self.width, self.height))
        pygame.display.set_caption("Painel de Instrumentos 6DOF")
        
        # Fontes para os textos
        self.font_large = pygame.font.SysFont("Arial", 24, bold=True)
        self.font_small = pygame.font.SysFont("Arial", 16, bold=True)

        # Cores padrão
        self.COLOR_SKY = (50, 150, 255)
        self.COLOR_EARTH = (139, 69, 19)
        self.COLOR_BG = (30, 30, 30)
        self.COLOR_WHITE = (255, 255, 255)
        self.COLOR_BLACK = (0, 0, 0)
        self.COLOR_RED = (255, 50, 50)
        self.COLOR_YELLOW = (255, 255, 0)

    def draw_gauge(self, x, y, radius, value, max_value, title, unit="KTS"):
        """Desenha um relógio (velocímetro) circular genérico."""
        # Fundo do relógio
        pygame.draw.circle(self.screen, self.COLOR_BLACK, (x, y), radius)
        pygame.draw.circle(self.screen, self.COLOR_WHITE, (x, y), radius, 3) # Borda

        # Marcadores e números
        for i in range(0, max_value + 1, 20):
            # Mapeia 0 a max_value para os ângulos 210 a -30 graus
            angle_deg = 210 - (i / max_value) * 240
            angle_rad = math.radians(angle_deg)
            
            # Posição do traço e texto
            x_line_start = x + (radius - 10) * math.cos(angle_rad)
            y_line_start = y - (radius - 10) * math.sin(angle_rad)
            x_line_end = x + radius * math.cos(angle_rad)
            y_line_end = y - radius * math.sin(angle_rad)
            
            pygame.draw.line(self.screen, self.COLOR_WHITE, (x_line_start, y_line_start), (x_line_end, y_line_end), 2)
            
            text = self.font_small.render(str(i), True, self.COLOR_WHITE)
            text_rect = text.get_rect(center=(x + (radius - 25) * math.cos(angle_rad), y - (radius - 25) * math.sin(angle_rad)))
            self.screen.blit(text, text_rect)

        # Agulha (Ponteiro)
        needle_angle_deg = 210 - (min(value, max_value) / max_value) * 240
        needle_angle_rad = math.radians(needle_angle_deg)
        x_needle = x + (radius - 15) * math.cos(needle_angle_rad)
        y_needle = y - (radius - 15) * math.sin(needle_angle_rad)
        
        pygame.draw.line(self.screen, self.COLOR_RED, (x, y), (x_needle, y_needle), 4)
        pygame.draw.circle(self.screen, self.COLOR_WHITE, (x, y), 6) # Eixo central

        # Textos (Título, Valor e Unidade)
        title_surf = self.font_large.render(title, True, self.COLOR_WHITE)
        self.screen.blit(title_surf, title_surf.get_rect(center=(x, y - radius - 20)))

        val_surf = self.font_large.render(f"{int(value)} {unit}", True, self.COLOR_WHITE)
        self.screen.blit(val_surf, val_surf.get_rect(center=(x, y + radius / 2 + 10)))

    def draw_artificial_horizon(self, x, y, radius, pitch_rad, roll_rad):
        """Desenha o Horizonte Artificial reagindo a Pitch e Roll."""
        # Criamos uma superfície separada (maior) para desenhar o céu/terra e poder rodar sem cortar as bordas
        inner_surf = pygame.Surface((radius * 3, radius * 3), pygame.SRCALPHA)
        center_inner = radius * 1.5

        # O horizonte sobe e desce com o Pitch
        pixels_per_radian = radius * 1.5
        pitch_offset = pitch_rad * pixels_per_radian

        # Desenha Céu e Terra na superfície interna
        sky_rect = pygame.Rect(0, 0, inner_surf.get_width(), center_inner + pitch_offset)
        earth_rect = pygame.Rect(0, center_inner + pitch_offset, inner_surf.get_width(), inner_surf.get_height())
        
        pygame.draw.rect(inner_surf, self.COLOR_SKY, sky_rect)
        pygame.draw.rect(inner_surf, self.COLOR_EARTH, earth_rect)
        
        # Linha do horizonte branca
        pygame.draw.line(inner_surf, self.COLOR_WHITE, (0, center_inner + pitch_offset), (inner_surf.get_width(), center_inner + pitch_offset), 2)

        # Rotação pelo Roll (Pygame gira no sentido anti-horário com graus positivos)
        rotated_surf = pygame.transform.rotate(inner_surf, math.degrees(roll_rad))
        
        # CORREÇÃO AQUI: O centro da rotação deve ser o centro local da máscara (radius, radius)
        # e não as coordenadas globais da tela (x, y)
        rot_rect = rotated_surf.get_rect(center=(radius, radius))

        # Para fazer o horizonte ser circular, usamos clipping (máscara)
        clip_surf = pygame.Surface((radius * 2, radius * 2), pygame.SRCALPHA)
        pygame.draw.circle(clip_surf, (255, 255, 255, 255), (radius, radius), radius) # Máscara branca opaca
        
        # Cria uma superfície final para o instrumento colando o fundo rodado e aplicando a máscara
        final_instrument = pygame.Surface((radius * 2, radius * 2), pygame.SRCALPHA)
        final_instrument.blit(rotated_surf, rot_rect.topleft) # Cola usando as coordenadas locais corrigidas
        final_instrument.blit(clip_surf, (0, 0), special_flags=pygame.BLEND_RGBA_MIN) # Corta o que está fora do círculo

        # Desenha no ecrã principal
        self.screen.blit(final_instrument, (x - radius, y - radius))

        # Borda preta e branca (Moldura do instrumento)
        pygame.draw.circle(self.screen, self.COLOR_WHITE, (x, y), radius, 3)
        pygame.draw.circle(self.screen, self.COLOR_BLACK, (x, y), radius + 15, 15)

        # Referência fixa da aeronave (A marquinha amarela no centro)
        pygame.draw.line(self.screen, self.COLOR_YELLOW, (x - 40, y), (x - 10, y), 4)
        pygame.draw.line(self.screen, self.COLOR_YELLOW, (x + 10, y), (x + 40, y), 4)
        pygame.draw.line(self.screen, self.COLOR_YELLOW, (x, y + 10), (x, y), 4)

        # Título
        title_surf = self.font_large.render("ATTITUDE", True, self.COLOR_WHITE)
        self.screen.blit(title_surf, title_surf.get_rect(center=(x, y - radius - 20)))

    def update(self, pitch_rad, roll_rad, ias_knots, tas_knots):
        """Atualiza o ecrã a cada ciclo da simulação."""
        self.screen.fill(self.COLOR_BG) # Limpa o ecrã com cinzento escuro

        # Coordenadas dos três instrumentos (lado a lado)
        center_y = self.height // 2 + 10
        radius = 100
        spacing = self.width // 3
        
        # 1. Velocidade Relativa (Airspeed - IAS)
        self.draw_gauge(spacing // 2, center_y, radius, ias_knots, 200, "AIRSPEED (IAS)")

        # 2. Horizonte Artificial (Atitude)
        self.draw_artificial_horizon(spacing + spacing // 2, center_y, radius, pitch_rad, roll_rad)

        # 3. Velocidade Real (True Airspeed - TAS)
        self.draw_gauge(spacing * 2 + spacing // 2, center_y, radius, tas_knots, 200, "TRUE AIRSPEED")

        # Atualiza o display do Pygame
        pygame.display.flip()