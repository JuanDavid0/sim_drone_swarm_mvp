# engine.py
import numpy as np
from .drone import Drone
from .obstaculo import Obstaculo
from .cbf import aplicar_cbf_simplificada, reset_cbf_activation_count # Asumiendo que cbf.py también usa el config pasado

class SimulationEngine:
    """
    Motor principal de la simulación. Gestiona el estado, los agentes (drones),
    los obstáculos, las interacciones y la progresión temporal.
    """
    def __init__(self, config, rngs):
        """
        Constructor de SimulationEngine.
        Args:
            config: Objeto de configuración con todos los parámetros de la simulación.
            rngs: Una tupla o lista de instancias de generadores de números aleatorios
                  (rng_entorno, rng_drones, rng_obstaculos).
        """
        self.config = config # Almacena la configuración editable/actual
        self.rng_entorno, self.rng_drones, self.rng_obst = rngs # Desempaqueta y almacena los RNGs

        self._init_state() # Llama al método para inicializar/resetear el estado de la simulación

    def _init_state(self):
        """
        Inicializa o resetea el estado de la simulación a sus valores por defecto.
        Esto incluye contadores, listas de agentes, la grilla de cobertura, etc.
        """
        reset_cbf_activation_count() # Resetea el contador global de activaciones CBF
        Drone._id_counter = -1 # Resetea el contador de ID para nuevos drones
        if hasattr(Obstaculo, '_id_counter'): # Si Obstaculo también tiene un contador de ID
            Obstaculo._id_counter = -1

        self.drones = [] # Lista para almacenar los objetos Drone
        self.obstaculos = [] # Lista para almacenar los objetos Obstaculo

        # Inicialización de la grilla de cobertura
        # Las dimensiones de la grilla se calculan basadas en el tamaño de la pantalla y el tamaño de celda definidos en config.
        # Esto corresponde a la discretización del espacio para medir la cobertura.
        nx = self.config.ANCHO_PANTALLA // self.config.TAMANO_CELDA_COBERTURA
        ny = self.config.ALTO_PANTALLA  // self.config.TAMANO_CELDA_COBERTURA
        self.grilla = np.zeros((nx, ny), dtype=int) # Matriz NumPy, 0=no cubierta, 1=cubierta
        self.total_celdas = nx * ny # Número total de celdas para calcular el porcentaje
        self.coverage = 0.0 # Porcentaje de cobertura inicial

        # Inicialización de contadores de tiempo y colisiones
        self.time = 0.0 # Tiempo total de simulación transcurrido
        self.critical_collisions = 0 # Contador de colisiones que resultan en fallo de dron
        self.time_since_last_obs = 0.0 # Temporizador para la generación periódica de obstáculos

        # Crear los agentes iniciales de la simulación
        self._spawn_drones(self.config.NUM_DRONES_INICIAL) # Crea el número inicial de drones
        self._spawn_initial_obstacles() # Crea el conjunto inicial de obstáculos

    def _spawn_drones(self, count):
        """
        Crea y añade un número 'count' de drones a la simulación.
        Las posiciones y velocidades iniciales son pseudoaleatorias.
        Esto establece las condiciones iniciales para los agentes dron.
        """
        colores = [ # Lista de colores para asignar visualmente a los drones
            self.config.AZUL, self.config.VERDE, self.config.ROJO,
            (255, 165, 0), (128, 0, 128) # Naranja, Morado
        ]
        for _ in range(count):
            # Generar posición inicial aleatoria usando el RNG del entorno
            # Se asegura que el dron aparezca completamente dentro de los límites
            x = (self.rng_entorno.next_float() *
                 (self.config.ANCHO_PANTALLA - 2 * self.config.RADIO_DRONE) # Rango disponible
                 + self.config.RADIO_DRONE) # Offset para el radio
            y = (self.rng_entorno.next_float() *
                 (self.config.ALTO_PANTALLA - 2 * self.config.RADIO_DRONE)
                 + self.config.RADIO_DRONE)
            
            dr_id = Drone._id_counter + 1 # El ID se gestiona en la clase Drone
            
            # Crea la instancia del Drone, pasando el objeto de configuración actual
            dr = Drone(
                x, y,
                self.config.RADIO_DRONE, # Radio del dron desde la config
                colores[dr_id % len(colores)], # Asigna un color cíclicamente
                config_obj=self.config # Pasa el objeto config para que el dron use los parámetros actuales
            )
            # Asigna una velocidad inicial aleatoria usando el RNG de decisiones de drones
            dr.velocidad = np.array([
                self.rng_drones.next_float() * 40 - 20, # Rango de velocidad [-20, 20]
                self.rng_drones.next_float() * 40 - 20
            ], dtype=float)
            self.drones.append(dr)

    def _spawn_initial_obstacles(self):
        """Crea el conjunto inicial de obstáculos al inicio de la simulación."""
        for _ in range(self.config.NUM_OBSTACULOS):
            self._spawn_obstacle(initial=True) # Llama al método genérico de creación de obstáculos

    def _spawn_obstacle(self, initial=False):
        """
        Crea un único obstáculo, que puede ser estático o dinámico.
        Utiliza RNGs para determinar posición, tamaño, y propiedades dinámicas.
        Esto implementa la "generación aleatoria de obstáculos estáticos y dinámicos".
        """
        # No excede el máximo de obstáculos si no es una creación inicial
        if (len(self.obstaculos) >= self.config.MAX_OBSTACULOS_SIMULTANEOS
                and not initial):
            return

        # Posición y tamaño aleatorios usando el RNG del entorno
        margin = self.config.MAX_TAMANO_OBSTACULO + 20 # Margen para evitar bordes
        x = (self.rng_entorno.next_float() *
             (self.config.ANCHO_PANTALLA - 2 * margin) + margin)
        y = (self.rng_entorno.next_float() *
             (self.config.ALTO_PANTALLA - 2 * margin) + margin)
        r = (self.rng_entorno.next_float() * # Radio aleatorio dentro del rango configurado
             (self.config.MAX_TAMANO_OBSTACULO -
              self.config.MIN_TAMANO_OBSTACULO)
             + self.config.MIN_TAMANO_OBSTACULO)

        # Determinar si es dinámico y sus tiempos de vida/reaparición usando el RNG de obstáculos
        dyn = False
        tv = float('inf') # Tiempo de vida, infinito para estáticos
        tr = 0.0          # Tiempo de reaparición
        if self.rng_obst.next_float() < self.config.OBSTACULOS_DINAMICOS_PORCENTAJE:
            dyn = True
            tv = (self.rng_obst.next_float() *
                  (self.config.TIEMPO_VIDA_OBSTACULO_MAX -
                   self.config.TIEMPO_VIDA_OBSTACULO_MIN)
                  + self.config.TIEMPO_VIDA_OBSTACULO_MIN)
            tr = (self.rng_obst.next_float() *
                  (self.config.TIEMPO_RESPAWN_OBSTACULO_MAX -
                   self.config.TIEMPO_RESPAWN_OBSTACULO_MIN)
                  + self.config.TIEMPO_RESPAWN_OBSTACULO_MIN)

        # Crea la instancia del Obstaculo, pasando el objeto de configuración
        obs = Obstaculo(
            x, y, r,
            self.config.NEGRO, # Color del obstáculo
            self.config,       # Objeto de configuración
            es_dinamico=dyn,
            tiempo_vida=tv,
            tiempo_respawn=tr,
            rng_para_dinamica=self.rng_obst # RNG para su lógica interna si es dinámico
        )
        self.obstaculos.append(obs)

    def paso(self):
        """
        Ejecuta un único paso de tiempo (dt) en la simulación.
        Este es el método central que actualiza el estado del mundo.
        Sigue la secuencia: actualizar entorno, calcular fuerzas, aplicar CBF, integrar, detectar colisiones, actualizar cobertura.
        """
        dt = self.config.DELTA_T # Paso de tiempo
        self.time += dt          # Avanzar el tiempo global de la simulación

        # 1) Actualizar el estado de los obstáculos dinámicos (aparecer/desaparecer)
        for obs in self.obstaculos:
            obs.actualizar(dt, self.rng_obst) # El obstáculo usa su RNG para tiempos/tamaños al reactivarse

        # 2) Generar nuevos obstáculos dinámicos periódicamente
        self.time_since_last_obs += dt
        if (self.time_since_last_obs >=
                self.config.GENERAR_NUEVOS_OBSTACULOS_INTERVALO):
            self._spawn_obstacle()
            self.time_since_last_obs = 0.0 # Resetear temporizador

        # 3) Calcular las fuerzas para cada dron activo
        # Se necesitan las listas actuales de obstáculos y drones activos para estos cálculos.
        obsts_active = [o for o in self.obstaculos if o.esta_activo]
        drones_active_for_forces = [d for d in self.drones if d.esta_activo] # Evitar que drones inactivos ejerzan fuerza
        
        for dr in self.drones:
            if dr.esta_activo:
                # Los vecinos son otros drones activos dentro del rango sensorial del dron 'dr'
                neighbors = [d for d in drones_active_for_forces if d.id != dr.id] # Excluye al propio dron
                # El método calcular_fuerzas en Drone implementa la EDO de fuerzas:
                # F_i = F_cohesion + F_separacion + F_alineacion + F_frontera + F_obstaculos + F_bordes
                #
                dr.calcular_fuerzas(
                    neighbors, obsts_active,
                    self.grilla, self.config.TAMANO_CELDA_COBERTURA,
                    self.grilla.shape[0], self.grilla.shape[1], # Pasar dimensiones de la grilla (nx, ny)
                    self.rng_drones # RNG para decisiones internas del dron (ej. _encontrar_punto_frontera)
                )
            else:
                # Los drones inactivos no ejercen ni experimentan estas fuerzas de enjambre
                dr.fuerza_actual = np.zeros(2)

        # 4) Aplicar Control Barrier Functions (CBF) si están activadas
        # La CBF ajusta las velocidades para garantizar la seguridad (evitar colisiones)
        # Se basa en h(x) >= 0, donde h es la función barrera.
        if self.config.CBF_ACTIVADO:
            # Iterar sobre drones activos para aplicar CBF
            # Nota: drones_active fue definida antes para el cálculo de fuerzas, podemos reutilizarla
            # o re-filtrarla si algún dron se volvió inactivo justo ahora (poco probable sin movimiento aún)
            current_drones_active_for_cbf = [d for d in self.drones if d.esta_activo]
            for i, d1 in enumerate(current_drones_active_for_cbf):
                # CBF Dron-Dron
                for d2 in current_drones_active_for_cbf[i+1:]: # Evitar auto-comparación y pares duplicados
                    aplicar_cbf_simplificada(
                        d1, d2, self.config.CBF_D_MIN_DRON_DRON, False, # es_obstaculo = False
                        self.config # Pasa el objeto de configuración actual
                    )
                    aplicar_cbf_simplificada( # Aplicación simétrica
                        d2, d1, self.config.CBF_D_MIN_DRON_DRON, False,
                        self.config
                    )
                # CBF Dron-Obstáculo
                for obs in obsts_active: # Usar la lista ya filtrada de obstáculos activos
                    # d_min para CBF con obstáculos debe ser la distancia centro-a-centro segura
                    dist_min_cbf_obs = self.config.RADIO_DRONE + obs.radio + self.config.CBF_D_MIN_DRON_OBSTACULO
                    aplicar_cbf_simplificada(
                        d1, obs, dist_min_cbf_obs, True, # es_obstaculo = True
                        self.config
                    )

        # 5) Integración numérica RK4 para actualizar posición y velocidad
        # Resuelve el sistema de EDOs: dr/dt = v, dv/dt = F/m
        for dr in self.drones:
            # El método _rk4_step solo actúa sobre drones activos (o debería verificar internamente)
            # La lógica en _get_derivadas_estado dentro de _rk4_step ya maneja drones inactivos
            self._rk4_step(dr, dt)

        # 6) Detección y manejo de colisiones (después del movimiento)
        # Esto determina si un dron "falla" y se vuelve inactivo
        self._detect_collisions()

        # 7) Actualizar la grilla de cobertura
        self._update_coverage()

    def _rk4_step(self, dr, dt):
        """
        Implementa un paso del método Runge-Kutta de 4º orden para el dron 'dr'.
        Este método resuelve las EDOs de movimiento:
        dr/dt = v  (velocidad)
        dv/dt = F/m (aceleración, donde F es dr.fuerza_actual)
        """
        if not dr.esta_activo: # Los drones inactivos no se mueven
            dr.velocidad = np.zeros(2) # Asegurar que la velocidad sea cero
            return

        # Función interna para obtener las derivadas [velocidad, aceleración]
        # Usa la dr.fuerza_actual, que fue calculada al inicio del método 'paso'
        # Esto es una aproximación, ya que para un RK4 puro en sistemas acoplados,
        # la fuerza debería recalcularse en cada sub-paso k.
        def deriv(drone_obj, current_pos, current_vel):
            # La fuerza ya está calculada y almacenada en drone_obj.fuerza_actual
            # Si drone_obj.masa es cero, esto daría error. Se asume masa > 0.
            acceleration = drone_obj.fuerza_actual / drone_obj.masa
            return current_vel, acceleration # Retorna (derivada de posición, derivada de velocidad)

        p0, v0 = dr.posicion, dr.velocidad # Estado inicial del paso RK4

        # k1: Evaluar derivadas en el estado inicial (t0, y0)
        k1v_deriv, k1a_deriv = deriv(dr, p0, v0) # (velocidad_k1, aceleracion_k1)
        
        # k2: Evaluar derivadas en el punto medio (t0 + dt/2, y0 + (dt/2)*k1)
        k2v_deriv, k2a_deriv = deriv(dr, p0 + 0.5*dt*k1v_deriv, v0 + 0.5*dt*k1a_deriv)
        
        # k3: Evaluar derivadas en el punto medio (t0 + dt/2, y0 + (dt/2)*k2)
        k3v_deriv, k3a_deriv = deriv(dr, p0 + 0.5*dt*k2v_deriv, v0 + 0.5*dt*k2a_deriv)
        
        # k4: Evaluar derivadas al final del intervalo (t0 + dt, y0 + dt*k3)
        k4v_deriv, k4a_deriv = deriv(dr, p0 + dt*k3v_deriv, v0 + dt*k3a_deriv)

        # Combinar las pendientes para obtener la nueva posición y velocidad
        # y_new = y_old + (dt/6) * (k1 + 2*k2 + 2*k3 + k4)
        newp = p0 + (dt/6.0)*(k1v_deriv + 2*k2v_deriv + 2*k3v_deriv + k4v_deriv)
        newv = v0 + (dt/6.0)*(k1a_deriv + 2*k2a_deriv + 2*k3a_deriv + k4a_deriv)
        
        # Actualizar el estado del dron (incluye límites de velocidad y rebote en bordes)
        dr.actualizar_estado_simple(newp, newv)

    def _detect_collisions(self):
        """
        Detecta colisiones entre drones, y drones con obstáculos.
        Si ocurre una colisión, llama a dr.manejar_colision() que puede inactivar el dron.
        """
        # Colisión dron-obstáculo
        for dr in self.drones:
            if not dr.esta_activo:
                continue
            for obs in self.obstaculos:
                if not obs.esta_activo:
                    continue
                # Distancia entre centros de dron y obstáculo
                d = np.linalg.norm(dr.posicion - obs.posicion)
                # Umbral para considerar colisión (superposición menos un margen)
                umbral = dr.radio + obs.radio - self.config.DISTANCIA_COLISION_DRON_OBSTACULO
                if d < umbral:
                    antes_activo = dr.esta_activo
                    dr.manejar_colision("obstaculo", self.rng_drones) # rng_drones para la probabilidad de fallo
                    if antes_activo and not dr.esta_activo: # Si el dron falló en esta colisión
                        self.critical_collisions += 1

        # Colisión dron-dron (evita doble conteo y auto-colisión)
        n_drones = len(self.drones)
        for i in range(n_drones):
            for j in range(i + 1, n_drones): # Solo pares (i, j) con j > i
                d1, d2 = self.drones[i], self.drones[j]
                if not d1.esta_activo or not d2.esta_activo:
                    continue
                dist = np.linalg.norm(d1.posicion - d2.posicion)
                umbral_dron_dron = d1.radio + d2.radio - self.config.DISTANCIA_COLISION_DRON_DRON
                if dist < umbral_dron_dron:
                    antes_d1_activo, antes_d2_activo = d1.esta_activo, d2.esta_activo
                    d1.manejar_colision("dron", self.rng_drones)
                    d2.manejar_colision("dron", self.rng_drones) # Ambos tienen probabilidad de fallar
                    if (antes_d1_activo and not d1.esta_activo) or \
                       (antes_d2_activo and not d2.esta_activo):
                        self.critical_collisions += 1 # Contar como una colisión crítica si al menos uno falla

    def _update_coverage(self):
        """
        Actualiza la grilla de cobertura basada en las posiciones de los drones activos.
        Calcula el porcentaje de cobertura.
        """
        nx_grilla, ny_grilla = self.grilla.shape # Dimensiones de la grilla
        for dr in self.drones:
            if not dr.esta_activo:
                continue
            # Convertir posición del dron a índices de la grilla
            ix = int(dr.posicion[0] // self.config.TAMANO_CELDA_COBERTURA)
            iy = int(dr.posicion[1] // self.config.TAMANO_CELDA_COBERTURA)
            
            # Asegurar que los índices estén dentro de los límites de la grilla
            if 0 <= ix < nx_grilla and 0 <= iy < ny_grilla:
                if self.grilla[ix, iy] == 0: # Marcar solo si no estaba cubierta
                    self.grilla[ix, iy] = 1
        
        covered_cells = np.sum(self.grilla)
        self.coverage = (covered_cells / self.total_celdas) * 100 if self.total_celdas > 0 else 0