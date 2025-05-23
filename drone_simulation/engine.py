# engine.py
import numpy as np
from .drone import Drone
from .obstaculo import Obstaculo
from .cbf import aplicar_cbf_simplificada, reset_cbf_activation_count

class SimulationEngine:
    def __init__(self, config, rngs):
        self.config = config
        self.rng_entorno, self.rng_drones, self.rng_obst = rngs
        self._init_state()

    def _init_state(self):
        reset_cbf_activation_count()
        Drone._id_counter = -1
        if hasattr(Obstaculo, '_id_counter'):
            Obstaculo._id_counter = -1

        self.drones = []
        self.obstaculos = []

        # Grilla de cobertura
        nx = self.config.ANCHO_PANTALLA // self.config.TAMANO_CELDA_COBERTURA
        ny = self.config.ALTO_PANTALLA  // self.config.TAMANO_CELDA_COBERTURA
        self.grilla = np.zeros((nx, ny), dtype=int)
        self.total_celdas = nx * ny
        self.coverage = 0.0

        # Contadores de tiempo y colisiones
        self.time = 0.0
        self.critical_collisions = 0
        self.time_since_last_obs = 0.0

        # Crear agentes iniciales
        self._spawn_drones(self.config.NUM_DRONES_INICIAL)
        self._spawn_initial_obstacles()

    def _spawn_drones(self, count):
        colores = [
            self.config.AZUL, self.config.VERDE, self.config.ROJO,
            (255, 165, 0), (128, 0, 128)
        ]
        for _ in range(count):
            x = (self.rng_entorno.next_float() *
                 (self.config.ANCHO_PANTALLA - 2 * self.config.RADIO_DRONE)
                 + self.config.RADIO_DRONE)
            y = (self.rng_entorno.next_float() *
                 (self.config.ALTO_PANTALLA - 2 * self.config.RADIO_DRONE)
                 + self.config.RADIO_DRONE)
            dr_id = Drone._id_counter + 1
            dr = Drone(
                x, y,
                self.config.RADIO_DRONE,
                colores[dr_id % len(colores)],
                config_obj=self.config
            )
            dr.velocidad = np.array([
                self.rng_drones.next_float() * 40 - 20,
                self.rng_drones.next_float() * 40 - 20
            ], dtype=float)
            self.drones.append(dr)

    def _spawn_initial_obstacles(self):
        for _ in range(self.config.NUM_OBSTACULOS):
            self._spawn_obstacle(initial=True)

    def _spawn_obstacle(self, initial=False):
        if (len(self.obstaculos) >= self.config.MAX_OBSTACULOS_SIMULTANEOS
                and not initial):
            return

        margin = self.config.MAX_TAMANO_OBSTACULO + 20
        x = (self.rng_entorno.next_float() *
             (self.config.ANCHO_PANTALLA - 2 * margin) + margin)
        y = (self.rng_entorno.next_float() *
             (self.config.ALTO_PANTALLA - 2 * margin) + margin)
        r = (self.rng_entorno.next_float() *
             (self.config.MAX_TAMANO_OBSTACULO -
              self.config.MIN_TAMANO_OBSTACULO)
             + self.config.MIN_TAMANO_OBSTACULO)

        dyn = False
        tv = float('inf')
        tr = 0.0
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

        obs = Obstaculo(
            x, y, r,
            self.config.NEGRO,
            self.config,
            es_dinamico=dyn,
            tiempo_vida=tv,
            tiempo_respawn=tr,
            rng_para_dinamica=self.rng_obst
        )
        self.obstaculos.append(obs)

    def paso(self):
        dt = self.config.DELTA_T
        self.time += dt

        # 1) Actualizar obstáculos dinámicos
        for obs in self.obstaculos:
            obs.actualizar(dt, self.rng_obst)

        # 2) Generar nuevos obstáculos si toca
        self.time_since_last_obs += dt
        if (self.time_since_last_obs >=
                self.config.GENERAR_NUEVOS_OBSTACULOS_INTERVALO):
            self._spawn_obstacle()
            self.time_since_last_obs = 0.0

        # 3) Calcular fuerzas de cada dron
        obsts_active = [o for o in self.obstaculos if o.esta_activo]
        drones_active = [d for d in self.drones if d.esta_activo]
        for dr in self.drones:
            if dr.esta_activo:
                neighbors = [d for d in drones_active if d.id != dr.id]
                dr.calcular_fuerzas(
                    neighbors, obsts_active,
                    self.grilla, self.config.TAMANO_CELDA_COBERTURA,
                    *self.grilla.shape, self.rng_drones
                )
            else:
                dr.fuerza_actual = np.zeros(2)

        # 4) Aplicar Control Barrier Functions (si está activado)
        if self.config.CBF_ACTIVADO:
            for i, d1 in enumerate(drones_active):
                for d2 in drones_active[i+1:]:
                    aplicar_cbf_simplificada(
                        d1, d2, self.config.CBF_D_MIN_DRON_DRON, False,
                        self.config
                    )
                    aplicar_cbf_simplificada(
                        d2, d1, self.config.CBF_D_MIN_DRON_DRON, False,
                        self.config
                    )
                for obs in obsts_active:
                    dist_min = self.config.RADIO_DRONE + obs.radio + 5.0
                    aplicar_cbf_simplificada(
                        d1, obs, dist_min, True, self.config
                    )

        # 5) Integración numérica RK4
        for dr in self.drones:
            if dr.esta_activo:
                self._rk4_step(dr, dt)

        # 6) Detección y manejo de colisiones
        self._detect_collisions()

        # 7) Actualizar cobertura
        self._update_coverage()

    def _rk4_step(self, dr, dt):
        # Obtener derivadas
        def deriv(drone, pos, vel):
            a = drone.fuerza_actual / drone.masa
            return vel, a

        p0, v0 = dr.posicion, dr.velocidad
        k1v, k1a = deriv(dr, p0, v0)
        k2v, k2a = deriv(dr, p0 + 0.5*dt*k1v, v0 + 0.5*dt*k1a)
        k3v, k3a = deriv(dr, p0 + 0.5*dt*k2v, v0 + 0.5*dt*k2a)
        k4v, k4a = deriv(dr, p0 + dt*k3v, v0 + dt*k3a)

        newp = p0 + (dt/6)*(k1v + 2*k2v + 2*k3v + k4v)
        newv = v0 + (dt/6)*(k1a + 2*k2a + 2*k3a + k4a)
        dr.actualizar_estado_simple(newp, newv)

    def _detect_collisions(self):
        # Colisión dron-obstáculo
        for dr in self.drones:
            if not dr.esta_activo:
                continue
            for obs in self.obstaculos:
                if not obs.esta_activo:
                    continue
                d = np.linalg.norm(dr.posicion - obs.posicion)
                umbral = dr.radio + obs.radio - self.config.DISTANCIA_COLISION_DRON_OBSTACULO
                if d < umbral:
                    antes = dr.esta_activo
                    dr.manejar_colision("obstaculo", self.rng_drones)
                    if antes and not dr.esta_activo:
                        self.critical_collisions += 1

        # Colisión dron-dron
        n = len(self.drones)
        for i in range(n):
            for j in range(i+1, n):
                d1, d2 = self.drones[i], self.drones[j]
                if not d1.esta_activo or not d2.esta_activo:
                    continue
                dist = np.linalg.norm(d1.posicion - d2.posicion)
                umbral = (d1.radio + d2.radio -
                          self.config.DISTANCIA_COLISION_DRON_DRON)
                if dist < umbral:
                    b1, b2 = d1.esta_activo, d2.esta_activo
                    d1.manejar_colision("dron", self.rng_drones)
                    d2.manejar_colision("dron", self.rng_drones)
                    if (b1 and not d1.esta_activo) or (b2 and not d2.esta_activo):
                        self.critical_collisions += 1

    def _update_coverage(self):
        nx, ny = self.grilla.shape
        for dr in self.drones:
            if not dr.esta_activo:
                continue
            ix = int(dr.posicion[0] // self.config.TAMANO_CELDA_COBERTURA)
            iy = int(dr.posicion[1] // self.config.TAMANO_CELDA_COBERTURA)
            if 0 <= ix < nx and 0 <= iy < ny and self.grilla[ix, iy] == 0:
                self.grilla[ix, iy] = 1
        covered = np.sum(self.grilla)
        self.coverage = (covered / self.total_celdas) * 100 if self.total_celdas > 0 else 0
