# drone_simulation/drone.py
import pygame
import numpy as np
import random # Usado para la velocidad inicial si no se pasa un RNG, o como fallback
from . import config

class Drone:
    _id_counter = -1 # Empezar en -1 para que el primer ID sea 0

    def __init__(self, x, y, radio, color, id_drone=None): # El ID ahora se puede omitir y se autogenera
        Drone._id_counter +=1
        self.id = id_drone if id_drone is not None else Drone._id_counter
        
        self.posicion = np.array([float(x), float(y)])
        # La velocidad inicial podría depender de un RNG pasado o ser aleatoria por defecto
        # Para mantenerlo simple aquí, usamos el 'random' global de Python para la velocidad inicial,
        # ya que la creación de drones en Simulation ya usa sus propios RNGs para las posiciones.
        # Si se quisiera un control total, Simulation debería pasar un RNG a __init__ o establecer la velocidad.
        self.velocidad = np.array([random.uniform(-10,10), random.uniform(-10,10)], dtype=float)
        self.aceleracion = np.array([0.0, 0.0])
        self.fuerza_actual = np.array([0.0,0.0])
        
        self.radio = radio
        self.color_original = color
        self.color = color
        self.masa = config.MASA_DRONE
        self.max_velocidad = config.MAX_VELOCIDAD
        self.max_fuerza = config.MAX_FUERZA
        self.sensor_range = config.SENSOR_RANGE_DRONE
        self.radio_busqueda_frontera = config.RADIO_BUSQUEDA_FRONTERA_DRONE
        
        self.esta_activo = True

    def _encontrar_punto_frontera(self, grilla_cobertura, tamano_celda, num_celdas_x, num_celdas_y, rng_decision):
        if not self.esta_activo:
            return None

        mejor_punto = None
        dist_min_sq = float('inf')
        dron_celda_x = int(self.posicion[0] // tamano_celda)
        dron_celda_y = int(self.posicion[1] // tamano_celda)
        radio_busqueda_celdas = int(self.radio_busqueda_frontera // tamano_celda)

        for dx in range(-radio_busqueda_celdas, radio_busqueda_celdas + 1):
            for dy in range(-radio_busqueda_celdas, radio_busqueda_celdas + 1):
                if dx == 0 and dy == 0: 
                    continue
                
                celda_x_actual = dron_celda_x + dx
                celda_y_actual = dron_celda_y + dy

                if 0 <= celda_x_actual < num_celdas_x and \
                   0 <= celda_y_actual < num_celdas_y:
                    if grilla_cobertura[celda_x_actual, celda_y_actual] == 0:
                        punto_celda_actual = np.array([
                            (celda_x_actual + 0.5) * tamano_celda,
                            (celda_y_actual + 0.5) * tamano_celda
                        ])
                        dist_sq = np.sum((punto_celda_actual - self.posicion)**2)
                        if dist_sq < dist_min_sq:
                            dist_min_sq = dist_sq
                            mejor_punto = punto_celda_actual
        
        if mejor_punto is None:
             # Usar el RNG pasado para el objetivo aleatorio
             mejor_punto = np.array([rng_decision.next_float() * config.ANCHO_PANTALLA, 
                                     rng_decision.next_float() * config.ALTO_PANTALLA])
        
        if mejor_punto is not None:
            safety_margin_border = config.RADIO_DRONE * 2 
            mejor_punto[0] = np.clip(mejor_punto[0], safety_margin_border, config.ANCHO_PANTALLA - safety_margin_border)
            mejor_punto[1] = np.clip(mejor_punto[1], safety_margin_border, config.ALTO_PANTALLA - safety_margin_border)
            
        return mejor_punto

    def calcular_fuerzas(self, otros_drones, obstaculos, grilla_cobertura, tamano_celda, num_cx, num_cy, rng_decision_dron):
        if not self.esta_activo:
            self.fuerza_actual = np.array([0.0, 0.0])
            return

        fuerza_total = np.array([0.0, 0.0])

        # --- 1. Fuerza de Atracción a la Frontera ---
        punto_frontera = self._encontrar_punto_frontera(grilla_cobertura, tamano_celda, num_cx, num_cy, rng_decision_dron)
        if punto_frontera is not None:
            vec_hacia_frontera = punto_frontera - self.posicion
            dist_frontera = np.linalg.norm(vec_hacia_frontera)
            if dist_frontera > 0:
                magnitud_fuerza_frontera_escalada = config.K_FRONTIER_ATTRACTION * 25 
                fuerza_frontera = (vec_hacia_frontera / dist_frontera) * magnitud_fuerza_frontera_escalada
                fuerza_total += fuerza_frontera
        
        # --- 2, 3, 4. Interacciones con otros Drones (Cohesión, Alineación, Separación) ---
        fuerza_cohesion_acumulada = np.array([0.0, 0.0])
        fuerza_separacion_acumulada = np.array([0.0, 0.0])
        fuerza_alineacion_acumulada = np.array([0.0, 0.0])
        vecinos_visibles_contador = 0

        for otro_dron in otros_drones: # Asumimos que 'otros_drones' ya excluye a self
            if not otro_dron.esta_activo: 
                continue
            dist_vector = otro_dron.posicion - self.posicion
            distancia = np.linalg.norm(dist_vector)
            if distancia > 0 and distancia < self.sensor_range:
                vecinos_visibles_contador += 1
                fuerza_cohesion_acumulada += dist_vector
                fuerza_alineacion_acumulada += otro_dron.velocidad
                if distancia < config.DISTANCIA_SEPARACION_MIN:
                    magnitud_separacion = 1.0 / (distancia**2 + config.EPSILON_FUERZA)
                    fuerza_separacion_acumulada += -1.0 * dist_vector * magnitud_separacion # Vector (yo - otro) * magnitud
        
        if vecinos_visibles_contador > 0:
            # Cohesión
            fuerza_total += config.K_COHESION * (fuerza_cohesion_acumulada / vecinos_visibles_contador)
            # Alineación
            fuerza_total += config.K_ALIGNMENT * ((fuerza_alineacion_acumulada / vecinos_visibles_contador) - self.velocidad)
            # Separación
            fuerza_total += config.K_SEPARATION * fuerza_separacion_acumulada
        
        # --- 5. Fuerza de Repulsión de Obstáculos ---
        fuerza_repulsion_obstaculos = np.array([0.0, 0.0])
        for obs in obstaculos: # Asumimos que 'obstaculos' pasados ya son los activos
            dist_vector_obs = self.posicion - obs.posicion
            distancia_obs = np.linalg.norm(dist_vector_obs)
            distancia_efectiva_superficie = distancia_obs - obs.radio - self.radio
            if distancia_efectiva_superficie < config.DISTANCIA_REACCION_OBSTACULO and distancia_obs > 0:
                fuerza_obs_individual = config.K_OBSTACLE_REPULSION * dist_vector_obs / (distancia_obs**2 + config.EPSILON_FUERZA)
                fuerza_repulsion_obstaculos += fuerza_obs_individual
        fuerza_total += fuerza_repulsion_obstaculos

        # --- 6. Fuerza de Repulsión de Bordes ---
        fuerza_repulsion_bordes = np.array([0.0, 0.0])
        if self.posicion[0] < config.DISTANCIA_REACCION_BORDE:
            dist_al_borde = self.posicion[0]
            fuerza_repulsion_bordes[0] += config.K_BORDE_REPULSION / (dist_al_borde + config.EPSILON_FUERZA)
        if self.posicion[0] > config.ANCHO_PANTALLA - config.DISTANCIA_REACCION_BORDE:
            dist_al_borde = config.ANCHO_PANTALLA - self.posicion[0]
            fuerza_repulsion_bordes[0] -= config.K_BORDE_REPULSION / (dist_al_borde + config.EPSILON_FUERZA)
        if self.posicion[1] < config.DISTANCIA_REACCION_BORDE:
            dist_al_borde = self.posicion[1]
            fuerza_repulsion_bordes[1] += config.K_BORDE_REPULSION / (dist_al_borde + config.EPSILON_FUERZA)
        if self.posicion[1] > config.ALTO_PANTALLA - config.DISTANCIA_REACCION_BORDE:
            dist_al_borde = config.ALTO_PANTALLA - self.posicion[1]
            fuerza_repulsion_bordes[1] -= config.K_BORDE_REPULSION / (dist_al_borde + config.EPSILON_FUERZA)
        fuerza_total += fuerza_repulsion_bordes
        
        # Limitar la fuerza total
        norma_fuerza = np.linalg.norm(fuerza_total)
        if norma_fuerza > self.max_fuerza:
            fuerza_total = (fuerza_total / norma_fuerza) * self.max_fuerza
        
        self.fuerza_actual = fuerza_total

    def actualizar_estado_simple(self, nueva_posicion, nueva_velocidad):
        if not self.esta_activo:
            self.velocidad = np.array([0.0, 0.0]) 
            self.aceleracion = np.array([0.0, 0.0])
            return

        self.posicion = nueva_posicion
        self.velocidad = nueva_velocidad
        
        norma_velocidad = np.linalg.norm(self.velocidad)
        if norma_velocidad > self.max_velocidad:
            self.velocidad = (self.velocidad / norma_velocidad) * self.max_velocidad
        
        # Lógica de rebote en bordes
        if self.posicion[0] - self.radio < 0:
            self.posicion[0] = self.radio
            self.velocidad[0] *= -0.5 
        elif self.posicion[0] + self.radio > config.ANCHO_PANTALLA:
            self.posicion[0] = config.ANCHO_PANTALLA - self.radio
            self.velocidad[0] *= -0.5
        
        if self.posicion[1] - self.radio < 0:
            self.posicion[1] = self.radio
            self.velocidad[1] *= -0.5
        elif self.posicion[1] + self.radio > config.ALTO_PANTALLA:
            self.posicion[1] = config.ALTO_PANTALLA - self.radio
            self.velocidad[1] *= -0.5

    def manejar_colision(self, tipo_colision="obstaculo", rng_decision=None):
        if not self.esta_activo: 
            return
        
        prob_fallo = 0.0
        if tipo_colision == "obstaculo": 
            prob_fallo = config.PROBABILIDAD_FALLO_POR_COLISION_OBSTACULO
        elif tipo_colision == "dron": 
            prob_fallo = config.PROBABILIDAD_FALLO_POR_COLISION_DRON
        
        decision_fallo = False
        if rng_decision:
            if rng_decision.next_float() < prob_fallo:
                decision_fallo = True
        else: # Fallback si no se pasa el RNG (no ideal)
            print("Advertencia: RNG no proporcionado a manejar_colision, usando random.random() global.")
            if random.random() < prob_fallo:
                decision_fallo = True

        if decision_fallo:
            self.esta_activo = False
            self.color = config.COLOR_DRON_INACTIVO
            self.velocidad = np.array([0.0, 0.0])
            self.fuerza_actual = np.array([0.0, 0.0])
            self.aceleracion = np.array([0.0,0.0])
            print(f"Dron {self.id} ha fallado debido a colisión con {tipo_colision}.")

    def dibujar(self, pantalla):
        pygame.draw.circle(pantalla, self.color, 
                           (int(self.posicion[0]), int(self.posicion[1])), 
                           self.radio)
        
        if self.esta_activo: # Solo dibujar ID si está activo
            # Considerar inicializar la fuente una vez en Simulation y pasarla o acceder a ella
            font = pygame.font.SysFont(None, 18) 
            texto_id = font.render(str(self.id), True, config.NEGRO)
            pos_texto_x = self.posicion[0] - texto_id.get_width() / 2
            pos_texto_y = self.posicion[1] - self.radio - texto_id.get_height() - 2
            pantalla.blit(texto_id, (pos_texto_x, pos_texto_y))