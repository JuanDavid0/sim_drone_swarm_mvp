# drone_simulation/simulation.py
import pygame
import numpy as np
from . import config
from .drone import Drone
from .obstaculo import Obstaculo
from . import cbf as cbf_module
from .rng import LCG, MiddleSquareRNG # <--- IMPORTAR NUESTROS RNGs

class Simulation:
    def __init__(self):
        pygame.init()
        pygame.font.init()
        self.pantalla = pygame.display.set_mode((config.ANCHO_PANTALLA, config.ALTO_PANTALLA))
        pygame.display.set_caption("Simulación Enjambre - MVP Etapa 7: RNGs Personalizados")
        self.reloj = pygame.time.Clock()
        
        self.corriendo_simulacion = False
        self.game_loop_activo = True

        # --- Inicialización de RNGs ---
        self.rng_entorno = LCG(
            seed=config.GCL_SEED_ENTORNO,
            multiplier=config.GCL_MULTIPLIER_A,
            increment=config.GCL_INCREMENT_C,
            modulus=config.GCL_MODULUS_M
        )
        self.rng_drones_decisiones = MiddleSquareRNG(
            seed=config.MIDDLE_SQUARE_SEED_DRONES,
            num_digits=config.N_DIGITS_MIDDLE_SQUARE
        )
        # (Opcional) Si quieres usar el GCL secundario para obstáculos dinámicos:
        self.rng_obstaculos_dinamicos = LCG(
            seed=config.GCL_SEED_OBSTACULOS_DYN,
            multiplier=config.GCL_MULTIPLIER_A_OBS,
            increment=config.GCL_INCREMENT_C_OBS,
            modulus=config.GCL_MODULUS_M_OBS
        )
        print(f"RNG Entorno (LCG) inicializado con semilla: {self.rng_entorno.initial_seed}")
        print(f"RNG Drones (MiddleSquare) inicializado con semilla: {self.rng_drones_decisiones.initial_seed}")
        print(f"RNG Obst. Dinámicos (LCG) inicializado con semilla: {self.rng_obstaculos_dinamicos.initial_seed}")


        self.drones = []
        self.obstaculos = []
        
        self.num_celdas_x = config.ANCHO_PANTALLA // config.TAMANO_CELDA_COBERTURA
        self.num_celdas_y = config.ALTO_PANTALLA // config.TAMANO_CELDA_COBERTURA
        self.grilla_cobertura = np.zeros((self.num_celdas_x, self.num_celdas_y), dtype=int)
        self.celdas_totales_cobertura = self.num_celdas_x * self.num_celdas_y
        self.porcentaje_cobertura = 0.0
        
        self.font_metrics = pygame.font.SysFont(None, 24)
        self.tiempo_simulacion_total = 0.0
        self.colisiones_criticas_contador = 0
        
        self._reset_simulation_state()

    def _reset_simulation_state(self):
        print("Reseteando simulación...")
        # Resetear semillas de RNGs si es necesario o se desea un nuevo comportamiento
        # o mantenerlas para reproducibilidad si la semilla inicial de config no cambió.
        self.rng_entorno.set_seed(config.GCL_SEED_ENTORNO) # Usa la semilla de config (que podría ser None)
        self.rng_drones_decisiones.set_seed(config.MIDDLE_SQUARE_SEED_DRONES)
        self.rng_obstaculos_dinamicos.set_seed(config.GCL_SEED_OBSTACULOS_DYN)
        
        print(f"RNG Entorno (LCG) reseteado con semilla: {self.rng_entorno.initial_seed}")
        print(f"RNG Drones (MiddleSquare) reseteado con semilla: {self.rng_drones_decisiones.initial_seed}")
        print(f"RNG Obst. Dinámicos (LCG) reseteado con semilla: {self.rng_obstaculos_dinamicos.initial_seed}")

        Drone._id_counter = 0
        # Obstaculo._id_counter = 0 # Si tienes un contador similar en Obstaculo

        self.drones = []
        self.obstaculos = []
        self.grilla_cobertura = np.zeros((self.num_celdas_x, self.num_celdas_y), dtype=int)
        self.porcentaje_cobertura = 0.0
        self.tiempo_simulacion_total = 0.0
        self.colisiones_criticas_contador = 0
        cbf_module.reset_cbf_activation_count()
        
        self._crear_drones_iniciales()
        self._crear_obstaculos_iniciales()
        self.tiempo_desde_ultimo_obstaculo_generado = 0.0
        self.corriendo_simulacion = False
        print("Simulación reseteada.")

    def _crear_drones_iniciales(self, cantidad=None):
        if cantidad is None:
            cantidad = config.NUM_DRONES_INICIAL
        
        colores = [config.AZUL, config.VERDE, config.ROJO, (255, 165, 0), (128,0,128)]
        for _ in range(cantidad):
            # Usar RNG_Entorno para posiciones iniciales
            x = self.rng_entorno.next_float() * (config.ANCHO_PANTALLA - 2 * config.RADIO_DRONE) + config.RADIO_DRONE
            y = self.rng_entorno.next_float() * (config.ALTO_PANTALLA - 2 * config.RADIO_DRONE) + config.RADIO_DRONE
            
            color_dron = colores[Drone._id_counter % len(colores)]
            dron = Drone(x, y, config.RADIO_DRONE, color_dron) # El ID se auto-incrementa en Drone
            
            # Usar RNG_Drones_Decisiones para velocidad inicial
            vel_x = self.rng_drones_decisiones.next_float() * 40 - 20 # Rango de -20 a 20
            vel_y = self.rng_drones_decisiones.next_float() * 40 - 20
            dron.velocidad = np.array([vel_x, vel_y], dtype=float)
            self.drones.append(dron)
        print(f"Creados/Añadidos {cantidad} drones. Total: {len(self.drones)}")

    def _quitar_dron(self):
        if self.drones:
            # Usar RNG_Entorno para decidir cuál quitar (si es aleatorio)
            indice_a_quitar = self.rng_entorno.next_int(0, len(self.drones) - 1)
            dron_quitado = self.drones.pop(indice_a_quitar)
            print(f"Dron {dron_quitado.id} quitado. Total: {len(self.drones)}")
        else:
            print("No hay drones para quitar.")

    def _crear_obstaculos_iniciales(self):
        self.obstaculos = []
        for _ in range(config.NUM_OBSTACULOS):
            self._generar_un_obstaculo(es_inicial=True)

    def _generar_un_obstaculo(self, es_inicial=False):
        if len(self.obstaculos) >= config.MAX_OBSTACULOS_SIMULTANEOS and not es_inicial:
            return

        # Usar RNG_Entorno para posición y tamaño general
        margin = config.MAX_TAMANO_OBSTACULO + 20
        x = self.rng_entorno.next_float() * (config.ANCHO_PANTALLA - 2 * margin) + margin
        y = self.rng_entorno.next_float() * (config.ALTO_PANTALLA - 2 * margin) + margin
        radio_obs = self.rng_entorno.next_float() * \
                    (config.MAX_TAMANO_OBSTACULO - config.MIN_TAMANO_OBSTACULO) + config.MIN_TAMANO_OBSTACULO
        
        es_dinamico = False
        tiempo_vida = float('inf')
        tiempo_respawn = 0
        
        # Usar RNG_Obstaculos_Dinamicos para decidir si es dinámico y sus tiempos
        if self.rng_obstaculos_dinamicos.next_float() < config.OBSTACULOS_DINAMICOS_PORCENTAJE:
            es_dinamico = True
            tiempo_vida = self.rng_obstaculos_dinamicos.next_float() * \
                          (config.TIEMPO_VIDA_OBSTACULO_MAX - config.TIEMPO_VIDA_OBSTACULO_MIN) + \
                          config.TIEMPO_VIDA_OBSTACULO_MIN
            tiempo_respawn = self.rng_obstaculos_dinamicos.next_float() * \
                             (config.TIEMPO_RESPAWN_OBSTACULO_MAX - config.TIEMPO_RESPAWN_OBSTACULO_MIN) + \
                             config.TIEMPO_RESPAWN_OBSTACULO_MIN
        
        self.obstaculos.append(Obstaculo(x, y, radio_obs, config.NEGRO, es_dinamico, tiempo_vida, tiempo_respawn, self.rng_obstaculos_dinamicos)) # Pasar RNG al obstáculo si lo usa internamente
    
    # ... ( _get_derivadas_estado, _integrador_rk4_paso, actualizar_grilla_cobertura sin cambios en RNG) ...
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
        # Detección de colisión Dron-Obstáculo
        for dron in self.drones:
            if not dron.esta_activo: continue
            for obs in self.obstaculos:
                if not obs.esta_activo: continue
                distancia_centros = np.linalg.norm(dron.posicion - obs.posicion)
                if distancia_centros < (dron.radio + obs.radio - config.DISTANCIA_COLISION_DRON_OBSTACULO):
                    estado_previo_activo = dron.esta_activo
                    # Usar RNG_Drones_Decisiones para la probabilidad de fallo
                    dron.manejar_colision("obstaculo", self.rng_drones_decisiones) 
                    if estado_previo_activo and not dron.esta_activo: self.colisiones_criticas_contador +=1

        # Detección de colisión Dron-Dron
        for i in range(len(self.drones)):
            for j in range(i + 1, len(self.drones)):
                dron1 = self.drones[i]; dron2 = self.drones[j]
                if not dron1.esta_activo or not dron2.esta_activo: continue
                distancia_centros = np.linalg.norm(dron1.posicion - dron2.posicion)
                if distancia_centros < (dron1.radio + dron2.radio - config.DISTANCIA_COLISION_DRON_DRON):
                    e_p_d1 = dron1.esta_activo; e_p_d2 = dron2.esta_activo
                    # Usar RNG_Drones_Decisiones para la probabilidad de fallo
                    dron1.manejar_colision("dron", self.rng_drones_decisiones) 
                    dron2.manejar_colision("dron", self.rng_drones_decisiones)
                    if (e_p_d1 and not dron1.esta_activo) or \
                       (e_p_d2 and not dron2.esta_activo):
                       self.colisiones_criticas_contador +=1
    
    def _aplicar_cbfs(self):
        # ... (sin cambios en RNG, la lógica CBF en sí no es aleatoria) ...
        if not config.CBF_ACTIVADO: return
        drones_activos = [d for d in self.drones if d.esta_activo]
        obstaculos_activos = [o for o in self.obstaculos if o.esta_activo]
        for i, dron1 in enumerate(drones_activos):
            for j in range(i + 1, len(drones_activos)):
                dron2 = drones_activos[j]
                cbf_module.aplicar_cbf_simplificada(dron1, dron2, config.CBF_D_MIN_DRON_DRON, es_obstaculo=False)
                cbf_module.aplicar_cbf_simplificada(dron2, dron1, config.CBF_D_MIN_DRON_DRON, es_obstaculo=False)
            for obs in obstaculos_activos:
                d_min_c_obs = config.CBF_D_MIN_DRON_OBSTACULO 
                cbf_module.aplicar_cbf_simplificada(dron1, obs, d_min_c_obs, es_obstaculo=True)

    def manejar_eventos(self):
        for evento in pygame.event.get():
            if evento.type == pygame.QUIT:
                self.game_loop_activo = False
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
                if evento.key == config.TECLA_EJECUTAR_RNG_TESTS: # Nueva tecla
                    self.ejecutar_y_mostrar_pruebas_rng()


    def ejecutar_y_mostrar_pruebas_rng(self):
        """Ejecuta pruebas en los RNGs y muestra resultados en consola."""
        print("\n--- Ejecutando Pruebas de Calidad RNG ---")
        from .rng_validator import perform_rng_quality_tests # Importar aquí para evitar import circular si estuviera en otro sitio

        # Probar el GCL del Entorno
        print("\n--- Pruebas para GCL_ENTORNO ---")
        # Crear una instancia nueva para la prueba para no alterar el estado del de la simulación
        gcl_test_entorno = LCG(seed=self.rng_entorno.initial_seed, # Usa la misma semilla inicial
                               multiplier=config.GCL_MULTIPLIER_A,
                               increment=config.GCL_INCREMENT_C,
                               modulus=config.GCL_MODULUS_M)
        resultados_gcl_e = perform_rng_quality_tests(gcl_test_entorno, config.RNG_TEST_NUM_SAMPLES)
        for test_name, result in resultados_gcl_e.items():
            print(f"  {test_name}: {result}")

        # Probar el MiddleSquareRNG de los Drones
        print("\n--- Pruebas para MIDDLE_SQUARE_DRONES ---")
        ms_test_drones = MiddleSquareRNG(seed=self.rng_drones_decisiones.initial_seed,
                                         num_digits=config.N_DIGITS_MIDDLE_SQUARE)
        resultados_ms_d = perform_rng_quality_tests(ms_test_drones, config.RNG_TEST_NUM_SAMPLES)
        for test_name, result in resultados_ms_d.items():
            print(f"  {test_name}: {result}")
        
        # Probar el GCL de Obstáculos Dinámicos
        print("\n--- Pruebas para GCL_OBSTACULOS_DINAMICOS ---")
        gcl_test_obs = LCG(seed=self.rng_obstaculos_dinamicos.initial_seed,
                           multiplier=config.GCL_MULTIPLIER_A_OBS,
                           increment=config.GCL_INCREMENT_C_OBS,
                           modulus=config.GCL_MODULUS_M_OBS)
        resultados_gcl_o = perform_rng_quality_tests(gcl_test_obs, config.RNG_TEST_NUM_SAMPLES)
        for test_name, result in resultados_gcl_o.items():
            print(f"  {test_name}: {result}")
        print("---------------------------------------\n")


    def paso_simulacion(self):
        # ... (Lógica de paso_simulacion como la tenías, asegurándote de que los RNGs se usen donde sea necesario) ...
        if not self.corriendo_simulacion: return
        dt = config.DELTA_T; self.tiempo_simulacion_total += dt
        for obs in self.obstaculos: obs.actualizar(dt, self.rng_obstaculos_dinamicos) # Pasar RNG si obs lo usa para respawn aleatorio
        self.tiempo_desde_ultimo_obstaculo_generado += dt
        if self.tiempo_desde_ultimo_obstaculo_generado >= config.GENERAR_NUEVOS_OBSTACULOS_INTERVALO:
            self._generar_un_obstaculo(); self.tiempo_desde_ultimo_obstaculo_generado = 0.0
        obstaculos_activos_para_fuerzas = [obs for obs in self.obstaculos if obs.esta_activo]
        drones_activos_para_calculo_fuerzas = [d for d in self.drones if d.esta_activo]
        for dron_actual in self.drones:
            if dron_actual.esta_activo:
                otros_drones_activos = [d for d in drones_activos_para_calculo_fuerzas if d.id != dron_actual.id]
                # Pasar el RNG de decisiones del dron a _encontrar_punto_frontera si usa aleatoriedad
                dron_actual.calcular_fuerzas(otros_drones_activos, obstaculos_activos_para_fuerzas,
                                             self.grilla_cobertura, config.TAMANO_CELDA_COBERTURA,
                                             self.num_celdas_x, self.num_celdas_y, self.rng_drones_decisiones)
            else: dron_actual.fuerza_actual = np.array([0.0,0.0])
        if config.CBF_ACTIVADO: self._aplicar_cbfs()
        for dron_actual in self.drones: self._integrador_rk4_paso(dron_actual, dt)
        self._detectar_y_manejar_colisiones()
        self.actualizar_grilla_cobertura()


    def dibujar_elementos(self):
        # ... (sin cambios significativos, solo asegúrate de mostrar el contador CBF) ...
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
        drones_activos_count = sum(1 for d in self.drones if d.esta_activo)
        drones_inactivos_count = len(self.drones) - drones_activos_count
        metric_texts = [
            f"Tiempo: {self.tiempo_simulacion_total:.2f}s", f"Cobertura: {self.porcentaje_cobertura:.2f}%",
            f"Drones Activos: {drones_activos_count}", f"Drones Inactivos: {drones_inactivos_count}",
            f"Colisiones Críticas: {self.colisiones_criticas_contador}",
            f"Activaciones CBF: {cbf_module.get_cbf_activation_count()}"
        ]
        y_offset = 10
        for text_line in metric_texts:
            surface = self.font_metrics.render(text_line, True, config.NEGRO)
            self.pantalla.blit(surface, (10, y_offset)); y_offset += 20
        if not self.corriendo_simulacion:
            pause_text = self.font_metrics.render("PAUSADO (Espacio para reanudar, R para reset, T para test RNG)", True, config.ROJO)
            text_rect = pause_text.get_rect(center=(config.ANCHO_PANTALLA // 2, config.ALTO_PANTALLA // 2))
            self.pantalla.blit(pause_text, text_rect)
        pygame.display.flip()

    def bucle_principal(self):
        # ... (sin cambios) ...
        self.corriendo_simulacion = True 
        while self.game_loop_activo:
            self.manejar_eventos()
            self.paso_simulacion()
            self.dibujar_elementos()
            self.reloj.tick(config.FPS)
        pygame.quit()