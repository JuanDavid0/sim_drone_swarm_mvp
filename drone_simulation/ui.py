# drone_simulation/ui.py
import pygame
from .engine import SimulationEngine
import os, subprocess, sys

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

        # Fuentes para métricas y mensajes
        self.font_metrics = pygame.font.SysFont(None, 22)
        self.font_pause = pygame.font.SysFont(None, 36)
        self.font_speed = pygame.font.SysFont(None, 22) # Fuente para mostrar la velocidad

        # Estado de ejecución
        self.running = True # Iniciar la simulación corriendo por defecto
        self.paused = False # Nuevo estado para la pausa explícita

        # Control de velocidad de simulación
        self.simulation_speed_multiplier = 1.0  # 1.0 para velocidad normal
        self.simulation_time_accumulator = 0.0  # Para manejar multiplicadores fraccionales

    def run(self):
        """Inicia el bucle principal de Pygame."""
        # self.running se usa para el bucle principal de la aplicación
        # self.paused se usa para pausar/reanudar la lógica de la simulación
        
        app_running = True
        while app_running:
            # Manejo de eventos
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    app_running = False
                    self.running = False # Para asegurar que cualquier sub-bucle también termine

                elif event.type == pygame.KEYDOWN:
                    if event.key == self.config.TECLA_PAUSA_REANUDAR:
                        self.paused = not self.paused

                    elif event.key == self.config.TECLA_RESETEAR:
                        self.engine._init_state()

                    elif event.key == self.config.TECLA_ANADIR_DRON:
                        self.engine._spawn_drones(1)

                    elif event.key == self.config.TECLA_QUITAR_DRON:
                        if self.engine.drones: # Asegurarse que hay drones para quitar
                            self.engine.drones.pop()

                    elif event.key == self.config.TECLA_EJECUTAR_RNG_TESTS:
                        dash = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'rng_dashboard.py'))
                        subprocess.Popen([sys.executable, dash])

                    # Teclas para controlar la velocidad de simulación
                    elif event.key == pygame.K_PLUS or event.key == pygame.K_KP_PLUS:
                        self.simulation_speed_multiplier = min(self.simulation_speed_multiplier + 0.5, 5.0) # Cap max 5x
                    elif event.key == pygame.K_MINUS or event.key == pygame.K_KP_MINUS:
                        self.simulation_speed_multiplier = max(0.5, self.simulation_speed_multiplier - 0.5) # Cap min 0.5x


            # Actualizar simulación si no está pausada
            if not self.paused:
                # Determinar cuántos pasos de simulación ejecutar este fotograma visual
                steps_to_run_float = self.simulation_speed_multiplier
                
                # Parte entera de los pasos
                steps_this_frame = int(steps_to_run_float)
                
                # Acumular la parte fraccional
                self.simulation_time_accumulator += (steps_to_run_float - steps_this_frame)
                
                if self.simulation_time_accumulator >= 1.0:
                    steps_this_frame += int(self.simulation_time_accumulator)
                    self.simulation_time_accumulator -= int(self.simulation_time_accumulator)

                for _ in range(steps_this_frame):
                    if not self.paused: # Volver a verificar en caso de que se pause durante los sub-pasos
                        self.engine.paso()
                    else:
                        break
            
            # Dibujar escena
            self._draw()
            self.clock.tick(self.config.FPS) # Controla el FPS visual

        pygame.quit()

    def _draw(self):
        """Dibuja grilla, obstáculos, drones y métricas."""
        # Fondo y grilla de cobertura
        self.screen.fill(self.config.GRIS_CLARO)
        nx_grid, ny_grid = self.engine.grilla.shape # Corregido para usar las dimensiones reales de la grilla del engine
        cell_size = self.config.TAMANO_CELDA_COBERTURA # Corregido para usar el nombre correcto de la variable
        for i in range(nx_grid):
            for j in range(ny_grid):
                color = (
                    self.config.COLOR_CELDA_CUBIERTA
                    if self.engine.grilla[i, j] else
                    self.config.COLOR_CELDA_NO_CUBIERTA
                )
                rect = pygame.Rect(i * cell_size, j * cell_size, cell_size, cell_size)
                pygame.draw.rect(self.screen, color, rect)

        # Dibujar obstáculos y drones
        for obs in self.engine.obstaculos:
            obs.dibujar(self.screen)
        for dr in self.engine.drones:
            dr.dibujar(self.screen)

        # Métricas en pantalla
        y_offset = 10 # Renombrado para claridad
        texts = [
            f"Tiempo Sim: {self.engine.time:.2f}s",
            f"Cobertura: {self.engine.coverage:.2f}%",
            f"Drones activos: {sum(d.esta_activo for d in self.engine.drones)}",
            f"Drones inactivos: {len(self.engine.drones) - sum(d.esta_activo for d in self.engine.drones)}",
            f"Colisiones críticas: {self.engine.critical_collisions}"
        ]
        for line in texts:
            surf = self.font_metrics.render(line, True, self.config.NEGRO)
            self.screen.blit(surf, (10, y_offset))
            y_offset += 20

        # Mostrar velocidad de simulación
        speed_text_render = f"Velocidad Sim: x{self.simulation_speed_multiplier:.1f} (+/- para cambiar)"
        speed_surf = self.font_speed.render(speed_text_render, True, self.config.NEGRO)
        self.screen.blit(speed_surf, (10, y_offset))
        y_offset += 20

        # Mensaje de pausa
        if self.paused: # Usar el nuevo flag de pausa
            pause_surf = self.font_pause.render("PAUSADO (Espacio para reanudar)", True, self.config.ROJO)
            pause_rect = pause_surf.get_rect(
                center=(self.config.ANCHO_PANTALLA // 2, self.config.ALTO_PANTALLA - 30) # Posición del mensaje de pausa
            )
            self.screen.blit(pause_surf, pause_rect)

        pygame.display.flip()