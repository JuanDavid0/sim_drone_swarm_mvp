# drone_simulation/drone.py
import pygame
import numpy as np
import random # Ya no es necesario si la velocidad inicial la pone Simulation con su RNG

class Drone:
    _id_counter = -1 

    def __init__(self, x, y, radio_param, color, config_obj, id_drone=None): # AÑADIR config_obj, cambiar nombre radio
        Drone._id_counter +=1
        self.id = id_drone if id_drone is not None else Drone._id_counter
        
        self.config_propia = config_obj 
        
        self.posicion = np.array([float(x), float(y)])
        self.velocidad = np.array([0.0, 0.0], dtype=float) # Simulation asignará velocidad inicial
        self.aceleracion = np.array([0.0, 0.0])
        self.fuerza_actual = np.array([0.0,0.0])
        
        self.radio = radio_param # Usar el parámetro pasado
        self.color_original = color
        self.color = color
        self.masa = self.config_propia.MASA_DRONE
        self.max_velocidad = self.config_propia.MAX_VELOCIDAD
        self.max_fuerza = self.config_propia.MAX_FUERZA
        self.sensor_range = self.config_propia.SENSOR_RANGE_DRONE
        self.radio_busqueda_frontera = self.config_propia.RADIO_BUSQUEDA_FRONTERA_DRONE
        self.esta_activo = True

    def _encontrar_punto_frontera(self, grilla_cobertura, tamano_celda, num_celdas_x, num_celdas_y, rng_decision):
        # Todas las referencias a config.PARAM deben ser self.config_propia.PARAM
        if not self.esta_activo: return None
        mejor_punto = None; dist_min_sq = float('inf')
        dron_celda_x = int(self.posicion[0] // tamano_celda)
        dron_celda_y = int(self.posicion[1] // tamano_celda)
        radio_busqueda_celdas = int(self.radio_busqueda_frontera // tamano_celda)
        for dx in range(-radio_busqueda_celdas, radio_busqueda_celdas + 1):
            for dy in range(-radio_busqueda_celdas, radio_busqueda_celdas + 1):
                if dx == 0 and dy == 0: continue
                celda_x_actual = dron_celda_x + dx; celda_y_actual = dron_celda_y + dy
                if 0 <= celda_x_actual < num_celdas_x and 0 <= celda_y_actual < num_celdas_y:
                    if grilla_cobertura[celda_x_actual, celda_y_actual] == 0:
                        punto_celda_actual = np.array([(celda_x_actual + 0.5) * tamano_celda, (celda_y_actual + 0.5) * tamano_celda])
                        dist_sq = np.sum((punto_celda_actual - self.posicion)**2)
                        if dist_sq < dist_min_sq: dist_min_sq = dist_sq; mejor_punto = punto_celda_actual
        if mejor_punto is None:
             mejor_punto = np.array([rng_decision.next_float() * self.config_propia.ANCHO_PANTALLA, 
                                     rng_decision.next_float() * self.config_propia.ALTO_PANTALLA])
        if mejor_punto is not None:
            safety_margin_border = self.radio * 2 # Usar self.radio que ya usa config_propia
            mejor_punto[0] = np.clip(mejor_punto[0], safety_margin_border, self.config_propia.ANCHO_PANTALLA - safety_margin_border)
            mejor_punto[1] = np.clip(mejor_punto[1], safety_margin_border, self.config_propia.ALTO_PANTALLA - safety_margin_border)
        return mejor_punto

    def calcular_fuerzas(self, otros_drones, obstaculos, grilla_cobertura, tamano_celda, num_cx, num_cy, rng_decision_dron):
        # TODAS las referencias a config.ALGO deben ser self.config_propia.ALGO
        # (El código que te di antes para este método ya hacía esto, verifica que sea así)
        if not self.esta_activo: self.fuerza_actual = np.array([0.0, 0.0]); return
        fuerza_total = np.array([0.0, 0.0])
        punto_frontera = self._encontrar_punto_frontera(grilla_cobertura, tamano_celda, num_cx, num_cy, rng_decision_dron)
        if punto_frontera is not None:
            vec_hacia_frontera = punto_frontera - self.posicion; dist_frontera = np.linalg.norm(vec_hacia_frontera)
            if dist_frontera > 0:
                magnitud_fuerza_frontera_escalada = self.config_propia.K_FRONTIER_ATTRACTION * 25 
                fuerza_total += (vec_hacia_frontera / dist_frontera) * magnitud_fuerza_frontera_escalada
        f_coh_acc=np.array([0.0,0.0]); f_sep_acc=np.array([0.0,0.0]); f_ali_acc=np.array([0.0,0.0]); v_count=0
        for otro in otros_drones:
            if not otro.esta_activo: continue
            dist_vec = otro.posicion - self.posicion; dist = np.linalg.norm(dist_vec)
            if 0 < dist < self.sensor_range:
                v_count+=1; f_coh_acc+=dist_vec; f_ali_acc+=otro.velocidad
                if dist < self.config_propia.DISTANCIA_SEPARACION_MIN:
                    f_sep_acc += -1.0 * dist_vec * (1.0 / (dist**2 + self.config_propia.EPSILON_FUERZA))
        if v_count > 0:
            fuerza_total += self.config_propia.K_COHESION * (f_coh_acc / v_count)
            fuerza_total += self.config_propia.K_ALIGNMENT * ((f_ali_acc / v_count) - self.velocidad)
            fuerza_total += self.config_propia.K_SEPARATION * f_sep_acc
        f_obs_rep=np.array([0.0,0.0])
        for obs in obstaculos:
            dist_vec_obs = self.posicion - obs.posicion; dist_obs = np.linalg.norm(dist_vec_obs)
            dist_eff_surf = dist_obs - obs.radio - self.radio
            if dist_eff_surf < self.config_propia.DISTANCIA_REACCION_OBSTACULO and dist_obs > 0:
                f_obs_rep += self.config_propia.K_OBSTACLE_REPULSION * dist_vec_obs / (dist_obs**2 + self.config_propia.EPSILON_FUERZA)
        fuerza_total += f_obs_rep
        f_brd_rep=np.array([0.0,0.0])
        if self.posicion[0] < self.config_propia.DISTANCIA_REACCION_BORDE: f_brd_rep[0] += self.config_propia.K_BORDE_REPULSION / (self.posicion[0] + self.config_propia.EPSILON_FUERZA)
        if self.posicion[0] > self.config_propia.ANCHO_PANTALLA - self.config_propia.DISTANCIA_REACCION_BORDE: f_brd_rep[0] -= self.config_propia.K_BORDE_REPULSION / (self.config_propia.ANCHO_PANTALLA - self.posicion[0] + self.config_propia.EPSILON_FUERZA)
        if self.posicion[1] < self.config_propia.DISTANCIA_REACCION_BORDE: f_brd_rep[1] += self.config_propia.K_BORDE_REPULSION / (self.posicion[1] + self.config_propia.EPSILON_FUERZA)
        if self.posicion[1] > self.config_propia.ALTO_PANTALLA - self.config_propia.DISTANCIA_REACCION_BORDE: f_brd_rep[1] -= self.config_propia.K_BORDE_REPULSION / (self.config_propia.ALTO_PANTALLA - self.posicion[1] + self.config_propia.EPSILON_FUERZA)
        fuerza_total += f_brd_rep
        norma_f = np.linalg.norm(fuerza_total)
        if norma_f > self.max_fuerza: fuerza_total = (fuerza_total / norma_f) * self.max_fuerza
        self.fuerza_actual = fuerza_total

    def actualizar_estado_simple(self, nueva_posicion, nueva_velocidad): # Ya no necesita config_obj_pasado
        if not self.esta_activo:
            self.velocidad = np.array([0.0,0.0]); self.aceleracion = np.array([0.0,0.0]); return

        self.posicion = nueva_posicion
        self.velocidad = nueva_velocidad
        
        norma_velocidad = np.linalg.norm(self.velocidad)
        if norma_velocidad > self.max_velocidad:
            self.velocidad = (self.velocidad / norma_velocidad) * self.max_velocidad
        
        if self.posicion[0] - self.radio < 0:
            self.posicion[0] = self.radio; self.velocidad[0] *= -0.5 
        elif self.posicion[0] + self.radio > self.config_propia.ANCHO_PANTALLA:
            self.posicion[0] = self.config_propia.ANCHO_PANTALLA - self.radio; self.velocidad[0] *= -0.5
        if self.posicion[1] - self.radio < 0:
            self.posicion[1] = self.radio; self.velocidad[1] *= -0.5
        elif self.posicion[1] + self.radio > self.config_propia.ALTO_PANTALLA:
            self.posicion[1] = self.config_propia.ALTO_PANTALLA - self.radio; self.velocidad[1] *= -0.5

    def manejar_colision(self, tipo_colision="obstaculo", rng_decision=None): # config_obj_pasado ya no es necesario
        if not self.esta_activo: return
        
        prob_fallo = 0.0
        cfg_to_use = self.config_propia

        if tipo_colision == "obstaculo": prob_fallo = cfg_to_use.PROBABILIDAD_FALLO_POR_COLISION_OBSTACULO
        elif tipo_colision == "dron": prob_fallo = cfg_to_use.PROBABILIDAD_FALLO_POR_COLISION_DRON
        
        decision_fallo = False
        if rng_decision:
            if rng_decision.next_float() < prob_fallo: decision_fallo = True
        else: 
            # Usar el módulo random global como último recurso si no se pasa rng_decision
            # Esto es solo un fallback, idealmente siempre se pasa rng_decision
            if random.random() < prob_fallo: decision_fallo = True

        if decision_fallo:
            self.esta_activo = False; self.color = cfg_to_use.COLOR_DRON_INACTIVO
            self.velocidad = np.array([0.0,0.0]); self.fuerza_actual = np.array([0.0,0.0]); self.aceleracion = np.array([0.0,0.0])
            if cfg_to_use.VERBOSE: print(f"Dron {self.id} ha fallado debido a colisión con {tipo_colision}.")

    def dibujar(self, pantalla):
        pygame.draw.circle(pantalla, self.color, (int(self.posicion[0]), int(self.posicion[1])), int(self.radio))
        if self.esta_activo:
            font = pygame.font.SysFont(None, 18) 
            texto_id = font.render(str(self.id), True, self.config_propia.NEGRO)
            pos_texto_x = self.posicion[0] - texto_id.get_width() / 2
            pos_texto_y = self.posicion[1] - self.radio - texto_id.get_height() - 2
            pantalla.blit(texto_id, (pos_texto_x, pos_texto_y))