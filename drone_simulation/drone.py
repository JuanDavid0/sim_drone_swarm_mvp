# drone_simulation/drone.py
import pygame
import numpy as np
import random # Usado para velocidad inicial por defecto si Simulation no la provee o como fallback en manejar_colision

# Ya no se importa el config global aquí, se recibe en __init__

class Drone:
    _id_counter = -1 # Contador de clase para asignar IDs únicos a los drones

    def __init__(self, x, y, radio_param, color, config_obj, id_drone=None):
        
        Drone._id_counter +=1
        self.id = id_drone if id_drone is not None else Drone._id_counter
        
        self.config_propia = config_obj
        
        self.posicion = np.array([float(x), float(y)]) # Vector de posición [x, y]
        self.velocidad = np.array([0.0, 0.0], dtype=float) # Vector de velocidad [vx, vy], Simulation asignará la inicial
        self.aceleracion = np.array([0.0, 0.0]) # Vector de aceleración [ax, ay]
        self.fuerza_actual = np.array([0.0,0.0]) # Fuerza neta actual que actúa sobre el dron
        
        self.radio = radio_param
        self.color_original = color 
        self.color = color         

        self.masa = self.config_propia.MASA_DRONE
        self.max_velocidad = self.config_propia.MAX_VELOCIDAD
        self.max_fuerza = self.config_propia.MAX_FUERZA
        self.sensor_range = self.config_propia.SENSOR_RANGE_DRONE 
        self.radio_busqueda_frontera = self.config_propia.RADIO_BUSQUEDA_FRONTERA_DRONE
        
        self.esta_activo = True

    def _encontrar_punto_frontera(self, grilla_cobertura, tamano_celda, num_celdas_x, num_celdas_y, rng_decision):
        """
        Si no encuentra ninguna, elige un punto aleatorio en el mapa.
        Este método implementa la lógica de atracción a la frontera K_o(r_frontier,i - r_i) 
        """
        if not self.esta_activo: # Los drones inactivos no exploran
            return None

        mejor_punto = None # El punto frontera a retornar
        dist_min_sq = float('inf') # Usamos distancia al cuadrado para evitar raíces cuadradas innecesarias

        # Convertir la posición actual del dron a índices de la grilla
        dron_celda_x = int(self.posicion[0] // tamano_celda)
        dron_celda_y = int(self.posicion[1] // tamano_celda)

        # Calcular cuántas celdas buscar en cada dirección basado en el radio de búsqueda
        radio_busqueda_celdas = int(self.radio_busqueda_frontera // tamano_celda)

        # Iterar sobre las celdas vecinas dentro del radio de búsqueda
        for dx in range(-radio_busqueda_celdas, radio_busqueda_celdas + 1):
            for dy in range(-radio_busqueda_celdas, radio_busqueda_celdas + 1):
                if dx == 0 and dy == 0: # No considerar la celda donde ya está el dron
                    continue
                
                celda_x_actual = dron_celda_x + dx
                celda_y_actual = dron_celda_y + dy

                # Verificar que la celda esté dentro de los límites de la grilla
                if 0 <= celda_x_actual < num_celdas_x and \
                   0 <= celda_y_actual < num_celdas_y:
                    # Verificar si la celda no está cubierta
                    if grilla_cobertura[celda_x_actual, celda_y_actual] == 0:
                        # Calcular el centro de esta celda no cubierta
                        punto_celda_actual = np.array([
                            (celda_x_actual + 0.5) * tamano_celda,
                            (celda_y_actual + 0.5) * tamano_celda
                        ])
                        # Calcular distancia (al cuadrado) a esta celda
                        dist_sq = np.sum((punto_celda_actual - self.posicion)**2)
                        if dist_sq < dist_min_sq: # Si es la más cercana hasta ahora
                            dist_min_sq = dist_sq
                            mejor_punto = punto_celda_actual
        
        # Si no se encontró ninguna celda no cubierta en el radio de búsqueda
        if mejor_punto is None:
             # Como último recurso, dirigirse a un punto aleatorio en todo el mapa para seguir explorando
             mejor_punto = np.array([rng_decision.next_float() * self.config_propia.ANCHO_PANTALLA, 
                                     rng_decision.next_float() * self.config_propia.ALTO_PANTALLA])
        
        # Asegurar que el punto frontera elegido no esté demasiado pegado a los bordes físicos del mapa
        if mejor_punto is not None:
            safety_margin_border = self.radio * 2 
            mejor_punto[0] = np.clip(mejor_punto[0], safety_margin_border, self.config_propia.ANCHO_PANTALLA - safety_margin_border)
            mejor_punto[1] = np.clip(mejor_punto[1], safety_margin_border, self.config_propia.ALTO_PANTALLA - safety_margin_border)

        return mejor_punto

    def calcular_fuerzas(self, otros_drones, obstaculos, grilla_cobertura, tamano_celda, num_cx, num_cy, rng_decision_dron):
        """
        Calcula la fuerza neta que actúa sobre este dron.
        Ecuación general de fuerzas:
        F_i = sum(F_interacciones_drones) + F_frontera + F_obstaculos + F_bordes
        donde las interacciones entre drones (F_cohesion, F_separacion, F_alineacion) 
        son análogas a las fuerzas en el problema de N-cuerpos.
        """
        if not self.esta_activo: # Los drones inactivos no calculan ni aplican fuerzas
            self.fuerza_actual = np.array([0.0, 0.0])
            return

        fuerza_total = np.array([0.0, 0.0])

        # --- 1. Fuerza de Atracción a la Frontera (K_o) ---
        # Dirige al dron hacia áreas no exploradas.
        punto_frontera = self._encontrar_punto_frontera(grilla_cobertura, tamano_celda, num_cx, num_cy, rng_decision_dron)
        if punto_frontera is not None:
            vec_hacia_frontera = punto_frontera - self.posicion # Vector (r_frontier,i - r_i)
            dist_frontera = np.linalg.norm(vec_hacia_frontera)
            if dist_frontera > 0: # Evitar división por cero
                magnitud_fuerza_frontera_escalada = self.config_propia.K_FRONTIER_ATTRACTION
                fuerza_frontera = (vec_hacia_frontera / dist_frontera) * magnitud_fuerza_frontera_escalada # Fuerza = K_o * dir_frontera
                fuerza_total += fuerza_frontera
        
        # Interacciones con otros Drones (Cohesión, Alineación, Separación) ---
        # Estas fuerzas modelan el comportamiento de enjambre (flocking).
        fuerza_cohesion_acumulada = np.array([0.0, 0.0])
        fuerza_separacion_acumulada = np.array([0.0, 0.0])
        fuerza_alineacion_acumulada = np.array([0.0, 0.0])
        vecinos_visibles_contador = 0

        for otro_dron in otros_drones:
            if not otro_dron.esta_activo: 
                continue
            
            dist_vector = otro_dron.posicion - self.posicion # Vector (r_j - r_i)
            distancia = np.linalg.norm(dist_vector)

            # Considerar solo drones dentro del rango del sensor
            if 0 < distancia < self.sensor_range:
                vecinos_visibles_contador += 1
                
                # Acumular para Cohesión: vector hacia el otro dron (r_j - r_i)
                fuerza_cohesion_acumulada += dist_vector
    
                # Acumular para Alineación: velocidad del otro dron (v_j)
                fuerza_alineacion_acumulada += otro_dron.velocidad
                
                # Calcular Separación si está demasiado cerca
                # F_rep = -K_r * (r_j - r_i) / (||r_j - r_i||^2 + epsilon)
                # El signo negativo y (r_j - r_i) resulta en una fuerza en dirección (r_i - r_j)
                if distancia > 0:
                    # Fuerza repulsiva, inversamente proporcional al cuadrado de la distancia (o similar)
                    # El vector -(r_j - r_i) = (r_i - r_j) apunta desde el otro dron hacia el dron actual
                    fuerza_separacion_acumulada += (-1.0 * dist_vector / (distancia**2 + self.config_propia.EPSILON_FUERZA))
        
        if vecinos_visibles_contador > 0:
            # Cohesión: Moverse hacia el centroide promedio de los vecinos.
            # F_coh_i = K_c * ( (1/N) * sum(r_j) - r_i ) = K_c * ( sum(r_j - r_i) / N )
            fuerza_total += self.config_propia.K_COHESION * (fuerza_cohesion_acumulada / vecinos_visibles_contador)
            
            # Alineación: Intentar igualar la velocidad promedio de los vecinos.
            # F_ali_i = K_alignment * ( (1/N) * sum(v_j) - v_i )
            velocidad_promedio_vecinos = fuerza_alineacion_acumulada / vecinos_visibles_contador
            fuerza_total += self.config_propia.K_ALIGNMENT * (velocidad_promedio_vecinos - self.velocidad)
            
            # Separación: Aplicar la fuerza de separación acumulada (ya tiene K_r implícito en su cálculo)
            # La K_SEPARATION de config multiplica la suma de componentes 1/dist^2.
            fuerza_total += self.config_propia.K_SEPARATION * fuerza_separacion_acumulada
        
        # --- 5. Fuerza de Repulsión de Obstáculos ---
        # F_i,obs = K_o_obs * (r_i - r_obs) / ||r_i - r_obs||^2 (+ epsilon)
        fuerza_repulsion_obstaculos = np.array([0.0, 0.0])
        for obs in obstaculos: # 'obstaculos' ya son los activos
            dist_vector_obs = self.posicion - obs.posicion # Vector (r_i - r_obs)
            distancia_obs = np.linalg.norm(dist_vector_obs)
            # distancia_efectiva_superficie se usa para decidir si reaccionar
            distancia_efectiva_superficie = distancia_obs - obs.radio - self.radio 
            if distancia_efectiva_superficie < self.config_propia.DISTANCIA_REACCION_OBSTACULO and distancia_obs > 0:
                # La fuerza empuja en la dirección (r_i - r_obs), alejando el dron del centro del obstáculo
                fuerza_obs_individual = self.config_propia.K_OBSTACLE_REPULSION * dist_vector_obs / (distancia_obs**2 + self.config_propia.EPSILON_FUERZA)
                fuerza_repulsion_obstaculos += fuerza_obs_individual
        fuerza_total += fuerza_repulsion_obstaculos

        # --- 6. Fuerza de Repulsión de Bordes ---
        # Fuerza artificial para evitar que los drones se salgan de la pantalla.
        fuerza_repulsion_bordes = np.array([0.0, 0.0])
        if self.posicion[0] < self.config_propia.DISTANCIA_REACCION_BORDE: 
            dist_al_borde = self.posicion[0]
            fuerza_repulsion_bordes[0] += self.config_propia.K_BORDE_REPULSION / (dist_al_borde + self.config_propia.EPSILON_FUERZA)
        if self.posicion[0] > self.config_propia.ANCHO_PANTALLA - self.config_propia.DISTANCIA_REACCION_BORDE: 
            dist_al_borde = self.config_propia.ANCHO_PANTALLA - self.posicion[0]
            fuerza_repulsion_bordes[0] -= self.config_propia.K_BORDE_REPULSION / (dist_al_borde + self.config_propia.EPSILON_FUERZA)
        if self.posicion[1] < self.config_propia.DISTANCIA_REACCION_BORDE: 
            dist_al_borde = self.posicion[1]
            fuerza_repulsion_bordes[1] += self.config_propia.K_BORDE_REPULSION / (dist_al_borde + self.config_propia.EPSILON_FUERZA)
        if self.posicion[1] > self.config_propia.ALTO_PANTALLA - self.config_propia.DISTANCIA_REACCION_BORDE: 
            dist_al_borde = self.config_propia.ALTO_PANTALLA - self.posicion[1]
            fuerza_repulsion_bordes[1] -= self.config_propia.K_BORDE_REPULSION / (dist_al_borde + self.config_propia.EPSILON_FUERZA)
        fuerza_total += fuerza_repulsion_bordes
        
        # Limitar la magnitud de la fuerza total para evitar aceleraciones extremas.
        norma_fuerza = np.linalg.norm(fuerza_total)
        if norma_fuerza > self.max_fuerza: # self.max_fuerza es de config_propia
            fuerza_total = (fuerza_total / norma_fuerza) * self.max_fuerza
        
        self.fuerza_actual = fuerza_total # Almacenar para el integrador RK4

    def actualizar_estado_simple(self, nueva_posicion, nueva_velocidad):
        """
        Actualiza la posición y velocidad del dron con los nuevos valores calculados
        por el integrador. También aplica límites de velocidad y la lógica de rebote en bordes.
        Esta es la aplicación de dr_i/dt = v_i, después de que v_i ha sido actualizada.
        """
        if not self.esta_activo: # Drones inactivos no se actualizan (excepto para ser empujados, no implementado)
            self.velocidad = np.array([0.0,0.0])
            self.aceleracion = np.array([0.0,0.0])
            return

        self.posicion = nueva_posicion
        self.velocidad = nueva_velocidad
        
        # Limitar la velocidad máxima
        norma_velocidad = np.linalg.norm(self.velocidad)
        if norma_velocidad > self.max_velocidad: # self.max_velocidad es de config_propia
            self.velocidad = (self.velocidad / norma_velocidad) * self.max_velocidad
        
        # Lógica de rebote en los bordes físicos de la pantalla
        if self.posicion[0] - self.radio < 0:
            self.posicion[0] = self.radio # Corregir posición para no quedar fuera
            self.velocidad[0] *= -0.5 # Invertir y amortiguar velocidad en X
        elif self.posicion[0] + self.radio > self.config_propia.ANCHO_PANTALLA:
            self.posicion[0] = self.config_propia.ANCHO_PANTALLA - self.radio
            self.velocidad[0] *= -0.5
        
        if self.posicion[1] - self.radio < 0:
            self.posicion[1] = self.radio
            self.velocidad[1] *= -0.5 # Invertir y amortiguar velocidad en Y
        elif self.posicion[1] + self.radio > self.config_propia.ALTO_PANTALLA:
            self.posicion[1] = self.config_propia.ALTO_PANTALLA - self.radio
            self.velocidad[1] *= -0.5

    def manejar_colision(self, tipo_colision="obstaculo", rng_decision=None):
        """
        Gestiona una colisión detectada. Basado en una probabilidad (usando rng_decision),
        el dron puede volverse inactivo.
        Esto se relaciona con "detección de colisiones que provoquen la inactivación de agentes".
        """
        if not self.esta_activo: # Si ya está inactivo, no hacer nada
            return
        
        prob_fallo = 0.0
        cfg_usar = self.config_propia # Usar la configuración propia del dron

        if tipo_colision == "obstaculo": 
            prob_fallo = cfg_usar.PROBABILIDAD_FALLO_POR_COLISION_OBSTACULO
        elif tipo_colision == "dron": 
            prob_fallo = cfg_usar.PROBABILIDAD_FALLO_POR_COLISION_DRON
        
        decision_fallo = False
        if rng_decision: # Usar el RNG provisto si está disponible
            if rng_decision.next_float() < prob_fallo:
                decision_fallo = True
        else: 
            if hasattr(cfg_usar, 'VERBOSE') and cfg_usar.VERBOSE:
                print("Advertencia: RNG no proporcionado a manejar_colision, usando random.random() global.")
            if random.random() < prob_fallo:
                decision_fallo = True

        if decision_fallo:
            self.esta_activo = False
            self.color = cfg_usar.COLOR_DRON_INACTIVO # Cambiar color para indicar inactividad
            self.velocidad = np.array([0.0,0.0])    # Detener el dron
            self.fuerza_actual = np.array([0.0,0.0]) # Sin fuerza
            self.aceleracion = np.array([0.0,0.0]) # Sin aceleración
            if hasattr(cfg_usar, 'VERBOSE') and cfg_usar.VERBOSE: 
                print(f"Dron {self.id} ha fallado debido a colisión con {tipo_colision}.")

    def dibujar(self, pantalla):
        """
        Dibuja el dron en la pantalla de Pygame.
        Los drones activos se muestran con su color original y su ID.
        Los drones inactivos se muestran con el color de dron inactivo.
        """
        pygame.draw.circle(pantalla, self.color, 
                           (int(self.posicion[0]), int(self.posicion[1])), 
                           int(self.radio))
        
        if self.esta_activo: # Solo dibujar ID si está activo
            # Considerar inicializar la fuente una vez en Simulation y pasarla para optimizar
            font = pygame.font.SysFont(None, 18) 
            texto_id = font.render(str(self.id), True, self.config_propia.NEGRO) # Usa color de config_propia
            # Centrar el texto del ID sobre el dron
            pos_texto_x = self.posicion[0] - texto_id.get_width() / 2
            pos_texto_y = self.posicion[1] - self.radio - texto_id.get_height() - 2 # Un poco encima del círculo
            pantalla.blit(texto_id, (pos_texto_x, pos_texto_y))