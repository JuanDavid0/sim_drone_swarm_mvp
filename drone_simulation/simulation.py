# drone_simulation/simulation.py
import pygame
import numpy as np
from . import config
from .drone import Drone
from .obstaculo import Obstaculo
import random

class Simulation:
    def __init__(self):
        # ... (inicialización de pygame, pantalla, reloj sin cambios) ...
        pygame.init()
        pygame.font.init()
        self.pantalla = pygame.display.set_mode((config.ANCHO_PANTALLA, config.ALTO_PANTALLA))
        pygame.display.set_caption("Simulación Enjambre - MVP Etapa 5: Fallos y Métricas")
        self.reloj = pygame.time.Clock()
        self.corriendo = True
        
        self.drones = []
        self.obstaculos = []
        
        self.num_celdas_x = config.ANCHO_PANTALLA // config.TAMANO_CELDA_COBERTURA
        self.num_celdas_y = config.ALTO_PANTALLA // config.TAMANO_CELDA_COBERTURA
        self.grilla_cobertura = np.zeros((self.num_celdas_x, self.num_celdas_y), dtype=int)
        self.celdas_totales_cobertura = self.num_celdas_x * self.num_celdas_y
        self.porcentaje_cobertura = 0.0
        
        self.font_metrics = pygame.font.SysFont(None, 24)

        self._crear_drones_iniciales()
        self._crear_obstaculos_iniciales()
        
        self.tiempo_desde_ultimo_obstaculo_generado = 0.0
        self.colisiones_criticas_contador = 0 # Nuevo contador


    def _crear_drones_iniciales(self):
        # ... (sin cambios) ...
        colores = [config.AZUL, config.VERDE, config.ROJO, (255, 165, 0), (128,0,128)] # Naranja, Morado
        for i in range(config.NUM_DRONES_INICIAL):
            x = random.uniform(config.RADIO_DRONE, config.ANCHO_PANTALLA - config.RADIO_DRONE)
            y = random.uniform(config.RADIO_DRONE, config.ALTO_PANTALLA - config.RADIO_DRONE)
            color_dron = colores[i % len(colores)]
            dron = Drone(x, y, config.RADIO_DRONE, color_dron, id_drone=i)
            dron.velocidad = np.array([random.uniform(-20, 20), random.uniform(-20, 20)], dtype=float)
            self.drones.append(dron)

    def _crear_obstaculos_iniciales(self):
        self.obstaculos = []
        for i in range(config.NUM_OBSTACULOS):
            self._generar_un_obstaculo(es_inicial=True)

    def _generar_un_obstaculo(self, es_inicial=False):
        if len(self.obstaculos) >= config.MAX_OBSTACULOS_SIMULTANEOS and not es_inicial:
            return
        margin = config.MAX_TAMANO_OBSTACULO + 20
        x = random.uniform(margin, config.ANCHO_PANTALLA - margin)
        y = random.uniform(margin, config.ALTO_PANTALLA - margin)
        radio_obs = random.uniform(config.MIN_TAMANO_OBSTACULO, config.MAX_TAMANO_OBSTACULO)
        es_dinamico = False
        tiempo_vida = float('inf')
        tiempo_respawn = 0
        if random.random() < config.OBSTACULOS_DINAMICOS_PORCENTAJE:
            es_dinamico = True
            tiempo_vida = random.uniform(config.TIEMPO_VIDA_OBSTACULO_MIN, config.TIEMPO_VIDA_OBSTACULO_MAX)
            tiempo_respawn = random.uniform(config.TIEMPO_RESPAWN_OBSTACULO_MIN, config.TIEMPO_RESPAWN_OBSTACULO_MAX)
        self.obstaculos.append(Obstaculo(x, y, radio_obs, config.NEGRO, es_dinamico, tiempo_vida, tiempo_respawn))

    def _get_derivadas_estado(self, dron, temp_pos, temp_vel):
        # ... (sin cambios) ...
        if not dron.esta_activo: # Si el dron está inactivo durante el RK4, no hay aceleración
            return temp_vel, np.array([0.0, 0.0])
        aceleracion_temporal = dron.fuerza_actual / dron.masa
        return temp_vel, aceleracion_temporal

    def _integrador_rk4_paso(self, dron, dt):
        # ... (sin cambios, pero llamará a _get_derivadas_estado que considera dron.esta_activo) ...
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
        for dron in self.drones:
            if not dron.esta_activo: # Drones inactivos no contribuyen a la cobertura
                continue
            celda_x = int(dron.posicion[0] // config.TAMANO_CELDA_COBERTURA)
            celda_y = int(dron.posicion[1] // config.TAMANO_CELDA_COBERTURA)
            if 0 <= celda_x < self.num_celdas_x and \
               0 <= celda_y < self.num_celdas_y:
                self.grilla_cobertura[celda_x, celda_y] = 1
        
        celdas_cubiertas = np.sum(self.grilla_cobertura)
        self.porcentaje_cobertura = (celdas_cubiertas / self.celdas_totales_cobertura) * 100 if self.celdas_totales_cobertura > 0 else 0

    def _detectar_y_manejar_colisiones(self):
        # Detección de colisión Dron-Obstáculo
        for dron in self.drones:
            if not dron.esta_activo:
                continue
            for obs in self.obstaculos:
                if not obs.esta_activo:
                    continue
                
                distancia_centros = np.linalg.norm(dron.posicion - obs.posicion)
                if distancia_centros < (dron.radio + obs.radio - config.DISTANCIA_COLISION_DRON_OBSTACULO): # Umbral de colisión
                    estado_previo_activo = dron.esta_activo
                    dron.manejar_colision("obstaculo")
                    if estado_previo_activo and not dron.esta_activo: # Si el dron falló en esta colisión
                        self.colisiones_criticas_contador +=1


        # Detección de colisión Dron-Dron (simple, O(N^2))
        for i in range(len(self.drones)):
            for j in range(i + 1, len(self.drones)):
                dron1 = self.drones[i]
                dron2 = self.drones[j]

                if not dron1.esta_activo or not dron2.esta_activo:
                    continue

                distancia_centros = np.linalg.norm(dron1.posicion - dron2.posicion)
                if distancia_centros < (dron1.radio + dron2.radio - config.DISTANCIA_COLISION_DRON_DRON): # Umbral de colisión
                    estado_previo_d1 = dron1.esta_activo
                    estado_previo_d2 = dron2.esta_activo
                    
                    dron1.manejar_colision("dron")
                    dron2.manejar_colision("dron") # Ambos pueden fallar independientemente

                    if (estado_previo_d1 and not dron1.esta_activo) or \
                       (estado_previo_d2 and not dron2.esta_activo):
                       self.colisiones_criticas_contador +=1 # Cuenta como una colisión crítica si al menos uno falla

    def manejar_eventos(self):
        # ... (sin cambios) ...
        for evento in pygame.event.get():
            if evento.type == pygame.QUIT:
                self.corriendo = False

    def paso_simulacion(self):
        # 0. Actualizar obstáculos y generar nuevos
        for obs in self.obstaculos:
            obs.actualizar(config.DELTA_T)
        self.tiempo_desde_ultimo_obstaculo_generado += config.DELTA_T
        if self.tiempo_desde_ultimo_obstaculo_generado >= config.GENERAR_NUEVOS_OBSTACULOS_INTERVALO:
            self._generar_un_obstaculo()
            self.tiempo_desde_ultimo_obstaculo_generado = 0.0

        # 1. Calcular fuerzas para drones activos
        obstaculos_activos_para_fuerzas = [obs for obs in self.obstaculos if obs.esta_activo]
        drones_activos_para_fuerzas = [d for d in self.drones if d.esta_activo]

        for dron_actual in self.drones: # Iterar sobre todos para que los inactivos no calculen fuerza
            if dron_actual.esta_activo:
                otros_drones_activos = [d for d in drones_activos_para_fuerzas if d.id != dron_actual.id]
                dron_actual.calcular_fuerzas(
                    otros_drones_activos, 
                    obstaculos_activos_para_fuerzas,
                    self.grilla_cobertura,
                    config.TAMANO_CELDA_COBERTURA,
                    self.num_celdas_x,
                    self.num_celdas_y
                )
            else: # Asegurar que los inactivos no tengan fuerza
                 dron_actual.fuerza_actual = np.array([0.0,0.0])


        # 2. Actualizar estado (RK4)
        for dron_actual in self.drones: # Actualizar todos (inactivos no se moverán por la lógica en actualizar_estado_simple)
            self._integrador_rk4_paso(dron_actual, config.DELTA_T)
            
        # 3. Detección de Colisiones y Manejo de Fallos
        self._detectar_y_manejar_colisiones()

        # 4. Actualizar grilla de cobertura (solo con drones activos)
        self.actualizar_grilla_cobertura()

    def dibujar_elementos(self):
        # ... (lógica para dibujar grilla, obstáculos, drones sin cambios significativos,
        # pero ahora el color del dron cambiará si está inactivo por la lógica en dron.manejar_colision) ...
        self.pantalla.fill(config.GRIS_CLARO)
        for x_idx in range(self.num_celdas_x):
            for y_idx in range(self.num_celdas_y):
                rect = pygame.Rect(x_idx * config.TAMANO_CELDA_COBERTURA, y_idx * config.TAMANO_CELDA_COBERTURA,
                                   config.TAMANO_CELDA_COBERTURA, config.TAMANO_CELDA_COBERTURA)
                color_celda = config.COLOR_CELDA_CUBIERTA if self.grilla_cobertura[x_idx, y_idx] == 1 \
                              else config.COLOR_CELDA_NO_CUBIERTA
                pygame.draw.rect(self.pantalla, color_celda, rect)
        
        for obs in self.obstaculos:
            obs.dibujar(self.pantalla) # Dibuja solo si está activo

        for dron in self.drones:
            dron.dibujar(self.pantalla) # El color cambiará si está inactivo
            
        # Métricas actualizadas
        drones_activos = sum(1 for d in self.drones if d.esta_activo)
        drones_inactivos = len(self.drones) - drones_activos

        texto_cobertura = self.font_metrics.render(f"Cobertura: {self.porcentaje_cobertura:.2f}%", True, config.NEGRO)
        texto_activos = self.font_metrics.render(f"Drones Activos: {drones_activos}", True, config.NEGRO)
        texto_inactivos = self.font_metrics.render(f"Drones Inactivos: {drones_inactivos}", True, config.NEGRO)
        texto_colisiones = self.font_metrics.render(f"Colisiones Críticas: {self.colisiones_criticas_contador}", True, config.NEGRO)

        self.pantalla.blit(texto_cobertura, (10, 10))
        self.pantalla.blit(texto_activos, (10, 30))
        self.pantalla.blit(texto_inactivos, (10, 50))
        self.pantalla.blit(texto_colisiones, (10, 70)) # Nueva métrica
        
        pygame.display.flip()

    def bucle_principal(self):
        # ... (sin cambios) ...
        while self.corriendo:
            self.manejar_eventos()
            self.paso_simulacion()
            self.dibujar_elementos()
            self.reloj.tick(config.FPS)
        pygame.quit()