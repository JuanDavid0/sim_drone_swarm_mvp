# drone_simulation/simulation.py
import pygame
import numpy as np
from . import config
from .drone import Drone
from .obstaculo import Obstaculo
from . import cbf as cbf_module # <--- IMPORTAR EL MÓDULO CBF
import random

class Simulation:
    def __init__(self):
        pygame.init()
        pygame.font.init()
        self.pantalla = pygame.display.set_mode((config.ANCHO_PANTALLA, config.ALTO_PANTALLA))
        pygame.display.set_caption("Simulación Enjambre - MVP Etapa 6: CBF y Controles")
        self.reloj = pygame.time.Clock()
        
        self.corriendo_simulacion = False # Para pausar/reanudar con teclado
        self.game_loop_activo = True # Para salir del bucle principal del juego

        self.drones = []
        self.obstaculos = []
        
        self.num_celdas_x = config.ANCHO_PANTALLA // config.TAMANO_CELDA_COBERTURA
        self.num_celdas_y = config.ALTO_PANTALLA // config.TAMANO_CELDA_COBERTURA
        self.grilla_cobertura = np.zeros((self.num_celdas_x, self.num_celdas_y), dtype=int)
        self.celdas_totales_cobertura = self.num_celdas_x * self.num_celdas_y
        self.porcentaje_cobertura = 0.0
        
        self.font_metrics = pygame.font.SysFont(None, 24)
        self.tiempo_simulacion_total = 0.0 # Nuevo para llevar el tiempo

        self._reset_simulation_state() # Llamar a un método de reseteo

    def _reset_simulation_state(self):
        """Resetea el estado de la simulación a sus valores iniciales."""
        print("Reseteando simulación...")
        Drone._id_counter = 0 # Asumiendo que tienes un _id_counter estático en Drone
        Obstaculo._id_counter = 0 # Si tienes id en obstaculo y es estático

        self.drones = []
        self.obstaculos = []
        self.grilla_cobertura = np.zeros((self.num_celdas_x, self.num_celdas_y), dtype=int)
        self.porcentaje_cobertura = 0.0
        self.tiempo_simulacion_total = 0.0
        self.colisiones_criticas_contador = 0
        cbf_module.reset_cbf_activation_count() # Resetea el contador de CBF
        
        self._crear_drones_iniciales()
        self._crear_obstaculos_iniciales()
        self.tiempo_desde_ultimo_obstaculo_generado = 0.0
        self.corriendo_simulacion = False # Iniciar pausado después del reset
        print("Simulación reseteada.")


    def _crear_drones_iniciales(self, cantidad=None):
        if cantidad is None:
            cantidad = config.NUM_DRONES_INICIAL
        
        colores = [config.AZUL, config.VERDE, config.ROJO, (255, 165, 0), (128,0,128)]
        for _ in range(cantidad): # Usar el _ si no necesitas el índice i para el ID del dron aquí
            x = random.uniform(config.RADIO_DRONE, config.ANCHO_PANTALLA - config.RADIO_DRONE)
            y = random.uniform(config.RADIO_DRONE, config.ALTO_PANTALLA - config.RADIO_DRONE)
            # El ID se asignará automáticamente por Drone._id_counter
            color_dron = colores[Drone._id_counter % len(colores)] # Usar el ID actual para el color
            dron = Drone(x, y, config.RADIO_DRONE, color_dron) 
            dron.velocidad = np.array([random.uniform(-20, 20), random.uniform(-20, 20)], dtype=float)
            self.drones.append(dron)
        print(f"Creados/Añadidos {cantidad} drones. Total: {len(self.drones)}")


    def _quitar_dron(self):
        if self.drones:
            dron_quitado = self.drones.pop(random.randrange(len(self.drones))) # Quita uno al azar
            print(f"Dron {dron_quitado.id} quitado. Total: {len(self.drones)}")
        else:
            print("No hay drones para quitar.")


    def _crear_obstaculos_iniciales(self):
        self.obstaculos = []
        for _ in range(config.NUM_OBSTACULOS): # Usar _ si el índice no se usa
            self._generar_un_obstaculo(es_inicial=True)

    def _generar_un_obstaculo(self, es_inicial=False):
        # ... (sin cambios) ...
        if len(self.obstaculos) >= config.MAX_OBSTACULOS_SIMULTANEOS and not es_inicial:
            return
        margin = config.MAX_TAMANO_OBSTACULO + 20
        x = random.uniform(margin, config.ANCHO_PANTALLA - margin)
        y = random.uniform(margin, config.ALTO_PANTALLA - margin)
        radio_obs = random.uniform(config.MIN_TAMANO_OBSTACULO, config.MAX_TAMANO_OBSTACULO)
        es_dinamico = False; tiempo_vida = float('inf'); tiempo_respawn = 0
        if random.random() < config.OBSTACULOS_DINAMICOS_PORCENTAJE:
            es_dinamico = True
            tiempo_vida = random.uniform(config.TIEMPO_VIDA_OBSTACULO_MIN, config.TIEMPO_VIDA_OBSTACULO_MAX)
            tiempo_respawn = random.uniform(config.TIEMPO_RESPAWN_OBSTACULO_MIN, config.TIEMPO_RESPAWN_OBSTACULO_MAX)
        self.obstaculos.append(Obstaculo(x, y, radio_obs, config.NEGRO, es_dinamico, tiempo_vida, tiempo_respawn))


    def _get_derivadas_estado(self, dron, temp_pos, temp_vel):
        # ... (sin cambios) ...
        if not dron.esta_activo: return temp_vel, np.array([0.0, 0.0])
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
            if not dron.esta_activo: continue
            celda_x = int(dron.posicion[0] // config.TAMANO_CELDA_COBERTURA)
            celda_y = int(dron.posicion[1] // config.TAMANO_CELDA_COBERTURA)
            if 0 <= celda_x < self.num_celdas_x and 0 <= celda_y < self.num_celdas_y:
                self.grilla_cobertura[celda_x, celda_y] = 1
        celdas_cubiertas = np.sum(self.grilla_cobertura)
        self.porcentaje_cobertura = (celdas_cubiertas / self.celdas_totales_cobertura) * 100 if self.celdas_totales_cobertura > 0 else 0

    def _detectar_y_manejar_colisiones(self):
        # ... (sin cambios) ...
        for dron in self.drones:
            if not dron.esta_activo: continue
            for obs in self.obstaculos:
                if not obs.esta_activo: continue
                distancia_centros = np.linalg.norm(dron.posicion - obs.posicion)
                if distancia_centros < (dron.radio + obs.radio - config.DISTANCIA_COLISION_DRON_OBSTACULO):
                    estado_previo_activo = dron.esta_activo
                    dron.manejar_colision("obstaculo")
                    if estado_previo_activo and not dron.esta_activo: self.colisiones_criticas_contador +=1
        for i in range(len(self.drones)):
            for j in range(i + 1, len(self.drones)):
                dron1 = self.drones[i]; dron2 = self.drones[j]
                if not dron1.esta_activo or not dron2.esta_activo: continue
                distancia_centros = np.linalg.norm(dron1.posicion - dron2.posicion)
                if distancia_centros < (dron1.radio + dron2.radio - config.DISTANCIA_COLISION_DRON_DRON):
                    e_p_d1 = dron1.esta_activo; e_p_d2 = dron2.esta_activo
                    dron1.manejar_colision("dron"); dron2.manejar_colision("dron")
                    if (e_p_d1 and not dron1.esta_activo) or (e_p_d2 and not dron2.esta_activo): self.colisiones_criticas_contador +=1
    
    def _aplicar_cbfs(self):
        if not config.CBF_ACTIVADO:
            return

        drones_activos = [d for d in self.drones if d.esta_activo]
        obstaculos_activos = [o for o in self.obstaculos if o.esta_activo]

        for i, dron1 in enumerate(drones_activos):
            # CBF Dron-Dron
            for j in range(i + 1, len(drones_activos)):
                dron2 = drones_activos[j]
                # d_min para CBF es la distancia segura deseada entre centros
                cbf_module.aplicar_cbf_simplificada(dron1, dron2, config.CBF_D_MIN_DRON_DRON, es_obstaculo=False)
                # Symmetrical application (dron2 vs dron1) - CBF should ideally consider both.
                # La implementación actual de aplicar_cbf_simplificada modifica dron_actual.velocidad
                # Para un sistema completo, se resolvería un QP para todas las restricciones.
                # Aquí, la segunda llamada podría usar la velocidad ya modificada de dron1.
                cbf_module.aplicar_cbf_simplificada(dron2, dron1, config.CBF_D_MIN_DRON_DRON, es_obstaculo=False)


            # CBF Dron-Obstáculo
            for obs in obstaculos_activos:
                # d_min para CBF dron-obstáculo: dist segura desde centro dron a centro obs
                # (config.CBF_D_MIN_DRON_OBSTACULO se definió como centro-a-superficie + radio_dron)
                # Así que d_min_centro_a_centro = config.CBF_D_MIN_DRON_OBSTACULO + obs.radio
                d_min_c_obs = config.CBF_D_MIN_DRON_OBSTACULO # Ajustar esta definición si es necesario
                cbf_module.aplicar_cbf_simplificada(dron1, obs, d_min_c_obs, es_obstaculo=True)


    def manejar_eventos(self):
        for evento in pygame.event.get():
            if evento.type == pygame.QUIT:
                self.game_loop_activo = False # Para salir del bucle principal
                self.corriendo_simulacion = False
            if evento.type == pygame.KEYDOWN:
                if evento.key == config.TECLA_PAUSA_REANUDAR:
                    self.corriendo_simulacion = not self.corriendo_simulacion
                    if self.corriendo_simulacion: print("Simulación reanudada.")
                    else: print("Simulación pausada.")
                if evento.key == config.TECLA_RESETEAR:
                    self._reset_simulation_state()
                if evento.key == config.TECLA_ANADIR_DRON:
                    self._crear_drones_iniciales(cantidad=1)
                if evento.key == config.TECLA_QUITAR_DRON:
                    self._quitar_dron()


    def paso_simulacion(self):
        if not self.corriendo_simulacion:
            return # No hacer nada si está pausado

        dt = config.DELTA_T
        self.tiempo_simulacion_total += dt

        # 0. Actualizar obstáculos y generar nuevos
        for obs in self.obstaculos:
            obs.actualizar(dt)
        self.tiempo_desde_ultimo_obstaculo_generado += dt
        if self.tiempo_desde_ultimo_obstaculo_generado >= config.GENERAR_NUEVOS_OBSTACULOS_INTERVALO:
            self._generar_un_obstaculo()
            self.tiempo_desde_ultimo_obstaculo_generado = 0.0

        # 1. Calcular fuerzas para drones activos
        obstaculos_activos_para_fuerzas = [obs for obs in self.obstaculos if obs.esta_activo]
        drones_activos_para_calculo_fuerzas = [d for d in self.drones if d.esta_activo]

        for dron_actual in self.drones:
            if dron_actual.esta_activo:
                otros_drones_activos = [d for d in drones_activos_para_calculo_fuerzas if d.id != dron_actual.id]
                dron_actual.calcular_fuerzas(
                    otros_drones_activos, 
                    obstaculos_activos_para_fuerzas,
                    self.grilla_cobertura,
                    config.TAMANO_CELDA_COBERTURA,
                    self.num_celdas_x,
                    self.num_celdas_y
                )
            else:
                 dron_actual.fuerza_actual = np.array([0.0,0.0])

        # 2. Aplicar CBFs para ajustar velocidades (ANTES de la integración)
        if config.CBF_ACTIVADO:
            self._aplicar_cbfs()

        # 3. Actualizar estado (RK4)
        for dron_actual in self.drones:
            # Los drones inactivos no se moverán por la lógica en actualizar_estado_simple
            # y su fuerza_actual es cero, por lo que _get_derivadas_estado dará accel cero.
            self._integrador_rk4_paso(dron_actual, dt)
            
        # 4. Detección de Colisiones y Manejo de Fallos (DESPUÉS de mover)
        self._detectar_y_manejar_colisiones()

        # 5. Actualizar grilla de cobertura
        self.actualizar_grilla_cobertura()


    def dibujar_elementos(self):
        # ... (dibujado de grilla, obstáculos, drones como antes) ...
        self.pantalla.fill(config.GRIS_CLARO)
        for x_idx in range(self.num_celdas_x):
            for y_idx in range(self.num_celdas_y):
                rect = pygame.Rect(x_idx * config.TAMANO_CELDA_COBERTURA, y_idx * config.TAMANO_CELDA_COBERTURA,
                                   config.TAMANO_CELDA_COBERTURA, config.TAMANO_CELDA_COBERTURA)
                color_celda = config.COLOR_CELDA_CUBIERTA if self.grilla_cobertura[x_idx, y_idx] == 1 \
                              else config.COLOR_CELDA_NO_CUBIERTA
                pygame.draw.rect(self.pantalla, color_celda, rect)
        
        for obs in self.obstaculos: obs.dibujar(self.pantalla)
        for dron in self.drones: dron.dibujar(self.pantalla)
            
        # Métricas actualizadas
        drones_activos_count = sum(1 for d in self.drones if d.esta_activo)
        drones_inactivos_count = len(self.drones) - drones_activos_count

        metric_texts = [
            f"Tiempo: {self.tiempo_simulacion_total:.2f}s",
            f"Cobertura: {self.porcentaje_cobertura:.2f}%",
            f"Drones Activos: {drones_activos_count}",
            f"Drones Inactivos: {drones_inactivos_count}",
            f"Colisiones Críticas: {self.colisiones_criticas_contador}",
            f"Activaciones CBF: {cbf_module.get_cbf_activation_count()}"
        ]
        
        y_offset = 10
        for text_line in metric_texts:
            surface = self.font_metrics.render(text_line, True, config.NEGRO)
            self.pantalla.blit(surface, (10, y_offset))
            y_offset += 20
        
        if not self.corriendo_simulacion:
            pause_text = self.font_metrics.render("PAUSADO (Espacio para reanudar)", True, config.ROJO)
            text_rect = pause_text.get_rect(center=(config.ANCHO_PANTALLA // 2, config.ALTO_PANTALLA // 2))
            self.pantalla.blit(pause_text, text_rect)

        pygame.display.flip()

    def bucle_principal(self):
        self.corriendo_simulacion = True # Iniciar corriendo por defecto
        while self.game_loop_activo: # Cambiado de self.corriendo a self.game_loop_activo
            self.manejar_eventos()
            self.paso_simulacion() # Solo se ejecuta si self.corriendo_simulacion es True
            self.dibujar_elementos()
            self.reloj.tick(config.FPS)
        
        pygame.quit()