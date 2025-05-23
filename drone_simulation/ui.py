# ui.py
import pygame
from .engine import SimulationEngine
from .reporter import Reporter

class SimulationUI:
    def __init__(self, config, rngs):
        # Configuración de ventana
        pygame.init()
        pygame.font.init()
        self.config = config
        self.screen = pygame.display.set_mode(
            (config.ANCHO_PANTALLA, config.ALTO_PANTALLA)
        )
        pygame.display.set_caption("Simulación Enjambre - Pygame")
        self.clock = pygame.time.Clock()

        # Motor de simulación y reporter
        self.engine = SimulationEngine(config, rngs)
        self.reporter = Reporter(config, rngs)

        # Fuentes para métricas y mensajes
        self.font_metrics = pygame.font.SysFont(None, 22)
        self.font_pause   = pygame.font.SysFont(None, 36)

        # Estado de ejecución
        self.running = False

    def run(self):
        """Inicia el bucle principal de Pygame."""
        self.running = True
        while self.running:
            # Manejo de eventos
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False

                elif event.type == pygame.KEYDOWN:
                    if event.key == self.config.TECLA_PAUSA_REANUDAR:
                        # Pausar / reanudar simulación
                        self.running = not self.running

                    elif event.key == self.config.TECLA_RESETEAR:
                        # Resetear estado e informes RNG
                        self.engine._init_state()
                        self.reporter.export_pdf()

                    elif event.key == self.config.TECLA_ANADIR_DRON:
                        # Añadir un dron extra
                        self.engine._spawn_drones(1)

                    elif event.key == self.config.TECLA_QUITAR_DRON:
                        # Quitar último dron
                        if self.engine.drones:
                            self.engine.drones.pop()

                    elif event.key == self.config.TECLA_EJECUTAR_RNG_TESTS:
                        # Ejecutar pruebas RNG en consola
                        self.reporter.export_pdf()

            # Actualizar simulación
            if self.running:
                self.engine.paso()

            # Dibujar escena
            self._draw()
            self.clock.tick(self.config.FPS)

        pygame.quit()

    def _draw(self):
        """Dibuja grilla, obstáculos, drones y métricas."""
        # Fondo y grilla de cobertura
        self.screen.fill(self.config.GRIS_CLARO)
        nx, ny = self.engine.grilla.shape
        size = self.config.TAMANO_CELDA_COBERTURA
        for i in range(nx):
            for j in range(ny):
                color = (
                    self.config.COLOR_CELDA_CUBIERTA
                    if self.engine.grilla[i, j] else
                    self.config.COLOR_CELDA_NO_CUBIERTA
                )
                rect = pygame.Rect(i*size, j*size, size, size)
                pygame.draw.rect(self.screen, color, rect)

        # Dibujar obstáculos y drones
        for obs in self.engine.obstaculos:
            obs.dibujar(self.screen)
        for dr in self.engine.drones:
            dr.dibujar(self.screen)

        # Métricas en pantalla
        texts = [
            f"Tiempo: {self.engine.time:.2f}s",
            f"Cobertura: {self.engine.coverage:.2f}%",
            f"Drones activos: {sum(d.esta_activo for d in self.engine.drones)}",
            f"Drones inactivos: {len(self.engine.drones) - sum(d.esta_activo for d in self.engine.drones)}",
            f"Colisiones críticas: {self.engine.critical_collisions}"
        ]
        y = 10
        for line in texts:
            surf = self.font_metrics.render(line, True, self.config.NEGRO)
            self.screen.blit(surf, (10, y))
            y += 20

        # Mensaje de pausa
        if not self.running:
            pause_surf = self.font_pause.render("PAUSADO (Espacio)", True, self.config.ROJO)
            pause_rect = pause_surf.get_rect(
                center=(self.config.ANCHO_PANTALLA//2, 50)
            )
            self.screen.blit(pause_surf, pause_rect)

        pygame.display.flip()
