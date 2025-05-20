# drone_simulation/simulation.py
import pygame
import numpy as np
from . import config
from .drone import Drone
from .obstaculo import Obstaculo
import random

class Simulation:
    def __init__(self):
        pygame.init()
        pygame.font.init()
        self.pantalla = pygame.display.set_mode((config.ANCHO_PANTALLA, config.ALTO_PANTALLA))
        pygame.display.set_caption("Simulación Enjambre - MVP Etapa 4: Frontera y Obst. Dinámicos")
        self.reloj = pygame.time.Clock()
        self.corriendo = True
        
        self.drones = []
        self.obstaculos = []
        # Ya no usamos punto_objetivo_global para la exploración principal de los drones
        # self.punto_objetivo_global = np.array([config.PUNTO_OBJETIVO_X, config.PUNTO_OBJETIVO_Y]) 
        
        self.num_celdas_x = config.ANCHO_PANTALLA // config.TAMANO_CELDA_COBERTURA
        self.num_celdas_y = config.ALTO_PANTALLA // config.TAMANO_CELDA_COBERTURA
        self.grilla_cobertura = np.zeros((self.num_celdas_x, self.num_celdas_y), dtype=int)
        self.celdas_totales_cobertura = self.num_celdas_x * self.num_celdas_y
        self.porcentaje_cobertura = 0.0
        
        self.font_metrics = pygame.font.SysFont(None, 24)

        self._crear_drones_iniciales()
        self._crear_obstaculos_iniciales()
        
        self.tiempo_desde_ultimo_obstaculo_generado = 0.0


    def _crear_drones_iniciales(self):
        # ... (sin cambios) ...
        colores = [config.AZUL, config.VERDE, config.ROJO, (255, 165, 0), (128,0,128)]
        for i in range(config.NUM_DRONES_INICIAL):
            x = random.uniform(config.RADIO_DRONE, config.ANCHO_PANTALLA - config.RADIO_DRONE)
            y = random.uniform(config.RADIO_DRONE, config.ALTO_PANTALLA - config.RADIO_DRONE)
            color_dron = colores[i % len(colores)]
            dron = Drone(x, y, config.RADIO_DRONE, color_dron, id_drone=i)
            dron.velocidad = np.array([random.uniform(-20, 20), random.uniform(-20, 20)], dtype=float)
            self.drones.append(dron)

    def _crear_obstaculos_iniciales(self):
        self.obstaculos = [] # Limpiar lista de obstáculos existentes
        for i in range(config.NUM_OBSTACULOS):
            self._generar_un_obstaculo(es_inicial=True)

    def _generar_un_obstaculo(self, es_inicial=False):
        if len(self.obstaculos) >= config.MAX_OBSTACULOS_SIMULTANEOS and not es_inicial:
            return

        # Posición aleatoria
        margin = config.MAX_TAMANO_OBSTACULO + 20 # Usa MAX_TAMANO_OBSTACULO aquí
        x = random.uniform(margin, config.ANCHO_PANTALLA - margin)
        y = random.uniform(margin, config.ALTO_PANTALLA - margin)
        radio_obs = random.uniform(config.MIN_TAMANO_OBSTACULO, config.MAX_TAMANO_OBSTACULO)
        
        es_dinamico = False
        tiempo_vida = float('inf')
        tiempo_respawn = 0
        
        if random.random() < config.OBSTACULOS_DINAMICOS_PORCENTAJE:
            es_dinamico = True
            # El tiempo de vida y respawn se establecerán dentro del constructor del obstáculo dinámico
            # O podrías pasarlos aquí desde el config.
            tiempo_vida = random.uniform(config.TIEMPO_VIDA_OBSTACULO_MIN, config.TIEMPO_VIDA_OBSTACULO_MAX)
            tiempo_respawn = random.uniform(config.TIEMPO_RESPAWN_OBSTACULO_MIN, config.TIEMPO_RESPAWN_OBSTACULO_MAX)

        # Verificar si se superpone con drones existentes (muy básico)
        # ... (puedes añadir una lógica similar a la de antes si es importante en la creación)
        
        self.obstaculos.append(Obstaculo(x, y, radio_obs, config.NEGRO, es_dinamico, tiempo_vida, tiempo_respawn))


    def _get_derivadas_estado(self, dron, temp_pos, temp_vel):
        # ... (sin cambios) ...
        aceleracion_temporal = dron.fuerza_actual / dron.masa
        return temp_vel, aceleracion_temporal

    def _integrador_rk4_paso(self, dron, dt):
        # ... (sin cambios) ...
        y_pos = dron.posicion; y_vel = dron.velocidad
        k1_vel, k1_accel = self._get_derivadas_estado(dron, y_pos, y_vel)
        temp_pos_k2 = y_pos + 0.5 * dt * k1_vel; temp_vel_k2 = y_vel + 0.5 * dt * k1_accel
        k2_vel, k2_accel = self._get_derivadas_estado(dron, temp_pos_k2, temp_vel_k2)
        temp_pos_k3 = y_pos + 0.5 * dt * k2_vel; temp_vel_k3 = y_vel + 0.5 * dt * k2_accel
        k3_vel, k3_accel = self._get_derivadas_estado(dron, temp_pos_k3, temp_vel_k3)
        temp_pos_k4 = y_pos + dt * k3_vel; temp_vel_k4 = y_vel + dt * k3_accel
        k4_vel, k4_accel = self._get_derivadas_estado(dron, temp_pos_k4, temp_vel_k4)
        nueva_posicion = y_pos + (dt / 6.0) * (k1_vel + 2*k2_vel + 2*k3_vel + k4_vel)
        nueva_velocidad = y_vel + (dt / 6.0) * (k1_accel + 2*k2_accel + 2*k3_accel + k4_accel)
        dron.actualizar_estado_simple(nueva_posicion, nueva_velocidad)

    def actualizar_grilla_cobertura(self):
        # ... (sin cambios) ...
        for dron in self.drones:
            celda_x = int(dron.posicion[0] // config.TAMANO_CELDA_COBERTURA)
            celda_y = int(dron.posicion[1] // config.TAMANO_CELDA_COBERTURA)
            if 0 <= celda_x < self.num_celdas_x and 0 <= celda_y < self.num_celdas_y:
                self.grilla_cobertura[celda_x, celda_y] = 1
        celdas_cubiertas = np.sum(self.grilla_cobertura)
        self.porcentaje_cobertura = (celdas_cubiertas / self.celdas_totales_cobertura) * 100 if self.celdas_totales_cobertura > 0 else 0


    def manejar_eventos(self):
        for evento in pygame.event.get():
            if evento.type == pygame.QUIT:
                self.corriendo = False
            # El clic del ratón ya no define el objetivo principal de exploración,
            # pero podríamos dejarlo para depuración o para un "punto de interés" manual.
            # if evento.type == pygame.MOUSEBUTTONDOWN:
            #     self.punto_objetivo_global = np.array(pygame.mouse.get_pos(), dtype=float)

    def paso_simulacion(self):
        # 0. Actualizar obstáculos dinámicos
        for obs in self.obstaculos:
            obs.actualizar(config.DELTA_T)
            
        # 0.1 Generar nuevos obstáculos dinámicos periódicamente
        self.tiempo_desde_ultimo_obstaculo_generado += config.DELTA_T
        if self.tiempo_desde_ultimo_obstaculo_generado >= config.GENERAR_NUEVOS_OBSTACULOS_INTERVALO:
            self._generar_un_obstaculo()
            self.tiempo_desde_ultimo_obstaculo_generado = 0.0

        # 1. Calcular todas las fuerzas primero
        obstaculos_activos = [obs for obs in self.obstaculos if obs.esta_activo]
        for dron_actual in self.drones:
            otros_drones = [d for d in self.drones if d.id != dron_actual.id]
            dron_actual.calcular_fuerzas(
                otros_drones, 
                obstaculos_activos, # Solo pasar obstáculos activos
                self.grilla_cobertura, # Pasar la grilla para la frontera
                config.TAMANO_CELDA_COBERTURA,
                self.num_celdas_x,
                self.num_celdas_y
            )

        # 2. Actualizar estado de cada dron usando RK4
        for dron_actual in self.drones:
            self._integrador_rk4_paso(dron_actual, config.DELTA_T)
            
        # 3. Actualizar grilla de cobertura
        self.actualizar_grilla_cobertura()

    def dibujar_elementos(self):
        self.pantalla.fill(config.GRIS_CLARO)
        
        for x_idx in range(self.num_celdas_x):
            for y_idx in range(self.num_celdas_y):
                rect = pygame.Rect(x_idx * config.TAMANO_CELDA_COBERTURA,
                                   y_idx * config.TAMANO_CELDA_COBERTURA,
                                   config.TAMANO_CELDA_COBERTURA,
                                   config.TAMANO_CELDA_COBERTURA)
                color_celda = config.COLOR_CELDA_CUBIERTA if self.grilla_cobertura[x_idx, y_idx] == 1 \
                              else config.COLOR_CELDA_NO_CUBIERTA
                pygame.draw.rect(self.pantalla, color_celda, rect)
        
        for obs in self.obstaculos: # Dibujar solo si está activo (ya manejado en obs.dibujar)
            obs.dibujar(self.pantalla)

        for dron in self.drones:
            dron.dibujar(self.pantalla)
            
        texto_cobertura = self.font_metrics.render(
            f"Cobertura: {self.porcentaje_cobertura:.2f}%", True, config.NEGRO)
        self.pantalla.blit(texto_cobertura, (10, 10))
        
        # Mostrar número de obstáculos activos
        num_obs_activos = sum(1 for obs in self.obstaculos if obs.esta_activo)
        texto_obs = self.font_metrics.render(f"Obstáculos Activos: {num_obs_activos}", True, config.NEGRO)
        self.pantalla.blit(texto_obs, (10, 30))

        pygame.display.flip()

    def bucle_principal(self):
        # ... (sin cambios) ...
        while self.corriendo:
            self.manejar_eventos()
            self.paso_simulacion()
            self.dibujar_elementos()
            self.reloj.tick(config.FPS)
        pygame.quit()