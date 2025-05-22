# drone_simulation/simulation.py
import pygame
import numpy as np
from . import config as defaultConfig_module # Para valores iniciales
from .drone import Drone
from .obstaculo import Obstaculo
from . import cbf as cbf_module
from .rng import LCG, MiddleSquareRNG
from .rng_validator import perform_rng_quality_tests_from_scratch
import types # Para SimpleNamespace

class Simulation:
    def __init__(self):
        # Crear una copia editable de la configuración
        self.config_actual = types.SimpleNamespace()
        for attr in dir(defaultConfig_module):
            if not attr.startswith("__"): # Copiar solo atributos públicos
                setattr(self.config_actual, attr, getattr(defaultConfig_module, attr))

        pygame.init()
        pygame.font.init()
        self.pantalla = pygame.display.set_mode((self.config_actual.ANCHO_PANTALLA, self.config_actual.ALTO_PANTALLA))
        pygame.display.set_caption(f"Sim Enjambre - Pygame (Ctrl Params)") # Título más corto
        self.reloj = pygame.time.Clock()
        
        self.corriendo_simulacion = False 
        self.game_loop_activo = True

        # --- Inicialización de RNGs (usando self.config_actual) ---
        self.rng_entorno = LCG(
            seed=self.config_actual.GCL_SEED_ENTORNO,
            multiplier=self.config_actual.GCL_MULTIPLIER_A,
            increment=self.config_actual.GCL_INCREMENT_C,
            modulus=self.config_actual.GCL_MODULUS_M
        )
        self.rng_drones_decisiones = MiddleSquareRNG(
            seed=self.config_actual.MIDDLE_SQUARE_SEED_DRONES,
            num_digits=self.config_actual.N_DIGITS_MIDDLE_SQUARE
        )
        self.rng_obstaculos_dinamicos = LCG(
            seed=self.config_actual.GCL_SEED_OBSTACULOS_DYN,
            multiplier=self.config_actual.GCL_MULTIPLIER_A_OBS,
            increment=self.config_actual.GCL_INCREMENT_C_OBS,
            modulus=self.config_actual.GCL_MODULUS_M_OBS
        )
        
        if self.config_actual.VERBOSE:
            print(f"Simulación Instanciada.")
            print(f"  RNG Entorno (LCG) con semilla inicial: {self.rng_entorno.initial_seed}")
            print(f"  RNG Drones (MiddleSquare) con semilla inicial: {self.rng_drones_decisiones.initial_seed}")
            print(f"  RNG Obst. Dinámicos (LCG) con semilla inicial: {self.rng_obstaculos_dinamicos.initial_seed}")

        self.drones = []
        self.obstaculos = []
        
        self.num_celdas_x = self.config_actual.ANCHO_PANTALLA // self.config_actual.TAMANO_CELDA_COBERTURA
        self.num_celdas_y = self.config_actual.ALTO_PANTALLA // self.config_actual.TAMANO_CELDA_COBERTURA
        self.grilla_cobertura = np.zeros((self.num_celdas_x, self.num_celdas_y), dtype=int)
        self.celdas_totales_cobertura = self.num_celdas_x * self.num_celdas_y
        self.porcentaje_cobertura = 0.0
        
        self.font_metrics = pygame.font.SysFont(None, 22) # Ligeramente más pequeño para más métricas
        self.font_params = pygame.font.SysFont("Consolas", 15) 
        self.font_instructions = pygame.font.SysFont(None, 18)
        self.tiempo_simulacion_total = 0.0
        self.colisiones_criticas_contador = 0

        # --- Definición de Parámetros Editables ---
        self.parametros_editables_definicion = {
            "NUM_DRONES_INICIAL": {"tipo": int, "min": 1, "max": 20, "paso": 1, "display_name": "Num Drones"},
            "MASA_DRONE": {"tipo": float, "min": 1.0, "max": 50.0, "paso": 1.0, "display_name": "Masa Dron"},
            "MAX_VELOCIDAD": {"tipo": float, "min": 10.0, "max": 300.0, "paso": 10.0, "display_name": "Max Vel"},
            "K_COHESION": {"tipo": float, "min": 0.0, "max": 3.0, "paso": 0.1, "display_name": "K Cohesión"},
            "K_SEPARATION": {"tipo": float, "min": 0.0, "max": 5000.0, "paso": 100.0, "display_name": "K Separación"},
            "K_ALIGNMENT": {"tipo": float, "min": 0.0, "max": 3.0, "paso": 0.1, "display_name": "K Alineación"},
            "K_FRONTIER_ATTRACTION": {"tipo": float, "min": 0.0, "max": 2.0, "paso": 0.1, "display_name": "K Frontera"},
            "K_OBSTACLE_REPULSION": {"tipo": float, "min": 0.0, "max": 10000.0, "paso": 250.0, "display_name": "K Rep. Obs."},
            "K_BORDE_REPULSION": {"tipo": float, "min": 0.0, "max": 5000.0, "paso": 100.0, "display_name": "K Rep. Borde"},
            "SENSOR_RANGE_DRONE": {"tipo": int, "min": 50, "max": 400, "paso": 10, "display_name": "Rango Sensor"},
            "RADIO_BUSQUEDA_FRONTERA_DRONE": {"tipo": int, "min": 50, "max": 500, "paso": 10, "display_name": "Rad. Bsq. Front."},
            "GCL_SEED_ENTORNO": {"tipo": "semilla", "display_name": "Semilla Entorno"},
            "MIDDLE_SQUARE_SEED_DRONES": {"tipo": "semilla_ms", "display_name": "Semilla Drones (MS)"},
            "N_DIGITS_MIDDLE_SQUARE": {"tipo": int, "min": 2, "max": 8, "paso":1, "display_name": "Digitos MS"},
            "PROBABILIDAD_FALLO_POR_COLISION_OBSTACULO": {"tipo": float, "min":0.0, "max":1.0, "paso":0.05, "display_name": "P(Fallo Obs)"},
            "PROBABILIDAD_FALLO_POR_COLISION_DRON": {"tipo": float, "min":0.0, "max":1.0, "paso":0.05, "display_name": "P(Fallo Dron)"},
        }
        self.nombres_parametros_editables = list(self.parametros_editables_definicion.keys())
        self.parametro_seleccionado_idx = 0
        self.modo_edicion_texto_param = False 
        self.buffer_texto_param = ""

        self._reset_simulation_state(inicial=True) # Pasar el flag 'inicial'
    
    def _reset_simulation_state(self, inicial=False): # <--- AÑADIR inicial=False AQUÍ
        if not inicial and self.config_actual.VERBOSE:
            print("Reseteando estado de la simulación con config_actual...")
        
        self.rng_entorno.set_seed(self.config_actual.GCL_SEED_ENTORNO)
        self.rng_drones_decisiones.set_seed(self.config_actual.MIDDLE_SQUARE_SEED_DRONES)
        self.rng_obstaculos_dinamicos.set_seed(self.config_actual.GCL_SEED_OBSTACULOS_DYN)
        
        if not inicial and self.config_actual.VERBOSE:
            print(f"  RNG Entorno usando semilla: {self.config_actual.GCL_SEED_ENTORNO} -> Efectiva: {self.rng_entorno.initial_seed}")
            print(f"  RNG Drones usando semilla: {self.config_actual.MIDDLE_SQUARE_SEED_DRONES} -> Efectiva: {self.rng_drones_decisiones.initial_seed}")
            print(f"  RNG Obst. Dinámicos usando semilla: {self.config_actual.GCL_SEED_OBSTACULOS_DYN} -> Efectiva: {self.rng_obstaculos_dinamicos.initial_seed}")

        Drone._id_counter = -1 
        if hasattr(Obstaculo, '_id_counter') and Obstaculo._id_counter is not None : 
            Obstaculo._id_counter = -1

        self.drones = []
        self.obstaculos = []
        self.grilla_cobertura.fill(0)
        self.porcentaje_cobertura = 0.0
        self.tiempo_simulacion_total = 0.0
        self.colisiones_criticas_contador = 0
        cbf_module.reset_cbf_activation_count()
        
        self._crear_drones_iniciales(self.config_actual.NUM_DRONES_INICIAL)
        self._crear_obstaculos_iniciales() 
        self.tiempo_desde_ultimo_obstaculo_generado = 0.0
        # No iniciar corriendo al resetear, permitir que el usuario lo haga o que el bucle lo maneje
        self.corriendo_simulacion = False if not inicial else True 
        if not inicial and self.config_actual.VERBOSE: print("Estado de simulación reseteado.")

    def _crear_drones_iniciales(self, cantidad=None):
        if cantidad is None:
            cantidad = self.config_actual.NUM_DRONES_INICIAL # Usar self.config_actual
        
        colores = [self.config_actual.AZUL, self.config_actual.VERDE, self.config_actual.ROJO, (255, 165, 0), (128,0,128)]
        for _ in range(cantidad):
            x = self.rng_entorno.next_float() * (self.config_actual.ANCHO_PANTALLA - 2 * self.config_actual.RADIO_DRONE) + self.config_actual.RADIO_DRONE
            y = self.rng_entorno.next_float() * (self.config_actual.ALTO_PANTALLA - 2 * self.config_actual.RADIO_DRONE) + self.config_actual.RADIO_DRONE
            color_idx = Drone._id_counter + 1 
            color_dron = colores[color_idx % len(colores)]
            # Pasar self.config_actual al constructor de Drone
            dron = Drone(x, y, self.config_actual.RADIO_DRONE, color_dron, config_obj=self.config_actual) 
            vel_x = self.rng_drones_decisiones.next_float() * 40 - 20 
            vel_y = self.rng_drones_decisiones.next_float() * 40 - 20
            dron.velocidad = np.array([vel_x, vel_y], dtype=float)
            self.drones.append(dron)
        if self.config_actual.VERBOSE: print(f"Creados/Añadidos {cantidad} drones. Total actual: {len(self.drones)}")

    def _quitar_dron(self):
        if self.drones:
            indice_a_quitar = self.rng_entorno.next_int(0, len(self.drones) - 1)
            dron_quitado = self.drones.pop(indice_a_quitar)
            if self.config_actual.VERBOSE: print(f"Dron {dron_quitado.id} quitado. Total: {len(self.drones)}")
        elif self.config_actual.VERBOSE: print("No hay drones para quitar.")

    def _crear_obstaculos_iniciales(self):
        self.obstaculos = []
        for _ in range(self.config_actual.NUM_OBSTACULOS): # Usar self.config_actual
            self._generar_un_obstaculo(es_inicial=True)

    def _generar_un_obstaculo(self, es_inicial=False): # Usa self.config_actual
        if len(self.obstaculos) >= self.config_actual.MAX_OBSTACULOS_SIMULTANEOS and not es_inicial:
            return
        margin = self.config_actual.MAX_TAMANO_OBSTACULO + 20
        x = self.rng_entorno.next_float() * (self.config_actual.ANCHO_PANTALLA - 2 * margin) + margin
        y = self.rng_entorno.next_float() * (self.config_actual.ALTO_PANTALLA - 2 * margin) + margin
        radio_obs = self.rng_entorno.next_float() * \
                    (self.config_actual.MAX_TAMANO_OBSTACULO - self.config_actual.MIN_TAMANO_OBSTACULO) + self.config_actual.MIN_TAMANO_OBSTACULO
        es_dinamico = False; tiempo_vida = float('inf'); tiempo_respawn = 0
        if self.rng_obstaculos_dinamicos.next_float() < self.config_actual.OBSTACULOS_DINAMICOS_PORCENTAJE:
            es_dinamico = True
            tiempo_vida = self.rng_obstaculos_dinamicos.next_float() * \
                          (self.config_actual.TIEMPO_VIDA_OBSTACULO_MAX - self.config_actual.TIEMPO_VIDA_OBSTACULO_MIN) + \
                          self.config_actual.TIEMPO_VIDA_OBSTACULO_MIN
            tiempo_respawn = self.rng_obstaculos_dinamicos.next_float() * \
                             (self.config_actual.TIEMPO_RESPAWN_OBSTACULO_MAX - self.config_actual.TIEMPO_RESPAWN_OBSTACULO_MIN) + \
                             self.config_actual.TIEMPO_RESPAWN_OBSTACULO_MIN
        # Pasar self.config_actual al constructor de Obstaculo
        # En simulation.py, dentro de _generar_un_obstaculo
        self.obstaculos.append(Obstaculo(x, # x
                                        y, # y
                                        radio_obs, # radio_param
                                        self.config_actual.NEGRO, # color
                                        self.config_actual, # config_obj <--- ESTE ES EL IMPORTANTE
                                        es_dinamico=es_dinamico, # es_dinamico (ahora como keyword)
                                        tiempo_vida=tiempo_vida, # tiempo_vida (ahora como keyword)
                                        tiempo_respawn=tiempo_respawn, # tiempo_respawn (ahora como keyword)
                                        rng_para_dinamica=self.rng_obstaculos_dinamicos # rng_para_dinamica (ahora como keyword)
                                        ))
    def _get_derivadas_estado(self, dron, temp_pos, temp_vel):
        if not dron.esta_activo: return temp_vel, np.array([0.0, 0.0])
        aceleracion_temporal = dron.fuerza_actual / dron.masa # dron.masa viene de su config_propia
        return temp_vel, aceleracion_temporal

    def _integrador_rk4_paso(self, dron, dt):
        if not dron.esta_activo: return
        y_pos = dron.posicion; y_vel = dron.velocidad
        k1_vel_deriv, k1_accel_deriv = self._get_derivadas_estado(dron, y_pos, y_vel)
        temp_pos_k2 = y_pos + 0.5 * dt * k1_vel_deriv; temp_vel_k2 = y_vel + 0.5 * dt * k1_accel_deriv
        k2_vel_deriv, k2_accel_deriv = self._get_derivadas_estado(dron, temp_pos_k2, temp_vel_k2)
        temp_pos_k3 = y_pos + 0.5 * dt * k2_vel_deriv; temp_vel_k3 = y_vel + 0.5 * dt * k2_accel_deriv
        k3_vel_deriv, k3_accel_deriv = self._get_derivadas_estado(dron, temp_pos_k3, temp_vel_k3)
        temp_pos_k4 = y_pos + dt * k3_vel_deriv; temp_vel_k4 = y_vel + dt * k3_accel_deriv
        k4_vel_deriv, k4_accel_deriv = self._get_derivadas_estado(dron, temp_pos_k4, temp_vel_k4)
        nueva_posicion = y_pos + (dt / 6.0) * (k1_vel_deriv + 2*k2_vel_deriv + 2*k3_vel_deriv + k4_vel_deriv)
        nueva_velocidad = y_vel + (dt / 6.0) * (k1_accel_deriv + 2*k2_accel_deriv + 2*k3_accel_deriv + k4_accel_deriv)
        dron.actualizar_estado_simple(nueva_posicion, nueva_velocidad) # Ya no se pasa config_actual aquí, Drone usa su self.config_propia

    def actualizar_grilla_cobertura(self):
        for dron in self.drones:
            if not dron.esta_activo: continue
            celda_x = int(dron.posicion[0] // self.config_actual.TAMANO_CELDA_COBERTURA)
            celda_y = int(dron.posicion[1] // self.config_actual.TAMANO_CELDA_COBERTURA)
            if 0 <= celda_x < self.num_celdas_x and 0 <= celda_y < self.num_celdas_y:
                if self.grilla_cobertura[celda_x, celda_y] == 0:
                    self.grilla_cobertura[celda_x, celda_y] = 1
        celdas_cubiertas = np.sum(self.grilla_cobertura)
        self.porcentaje_cobertura = (celdas_cubiertas / self.celdas_totales_cobertura) * 100 if self.celdas_totales_cobertura > 0 else 0

    def _detectar_y_manejar_colisiones(self): # Usa self.config_actual
        for dron in self.drones:
            if not dron.esta_activo: continue
            for obs in self.obstaculos:
                if not obs.esta_activo: continue
                distancia_centros = np.linalg.norm(dron.posicion - obs.posicion)
                if distancia_centros < (dron.radio + obs.radio - self.config_actual.DISTANCIA_COLISION_DRON_OBSTACULO):
                    estado_previo_activo = dron.esta_activo
                    dron.manejar_colision("obstaculo", self.rng_drones_decisiones) # Drone usa su config_propia
                    if estado_previo_activo and not dron.esta_activo: self.colisiones_criticas_contador +=1
        for i in range(len(self.drones)):
            for j in range(i + 1, len(self.drones)):
                dron1 = self.drones[i]; dron2 = self.drones[j]
                if not dron1.esta_activo or not dron2.esta_activo: continue
                distancia_centros = np.linalg.norm(dron1.posicion - dron2.posicion)
                if distancia_centros < (dron1.radio + dron2.radio - self.config_actual.DISTANCIA_COLISION_DRON_DRON):
                    e_p_d1 = dron1.esta_activo; e_p_d2 = dron2.esta_activo
                    dron1.manejar_colision("dron", self.rng_drones_decisiones) 
                    dron2.manejar_colision("dron", self.rng_drones_decisiones)
                    if (e_p_d1 and not dron1.esta_activo) or (e_p_d2 and not dron2.esta_activo):
                       self.colisiones_criticas_contador +=1
    
    def _aplicar_cbfs(self): # Usa self.config_actual
        if not self.config_actual.CBF_ACTIVADO: return
        drones_activos = [d for d in self.drones if d.esta_activo]
        obstaculos_activos = [o for o in self.obstaculos if o.esta_activo]
        for i, dron1 in enumerate(drones_activos):
            for j in range(i + 1, len(drones_activos)):
                dron2 = drones_activos[j]
                cbf_module.aplicar_cbf_simplificada(dron1, dron2, self.config_actual.CBF_D_MIN_DRON_DRON, False, self.config_actual)
                cbf_module.aplicar_cbf_simplificada(dron2, dron1, self.config_actual.CBF_D_MIN_DRON_DRON, False, self.config_actual)
            for obs in obstaculos_activos:
                distancia_seguridad_cbf_obs = self.config_actual.RADIO_DRONE + obs.radio + 5.0 
                cbf_module.aplicar_cbf_simplificada(dron1, obs, distancia_seguridad_cbf_obs, True, self.config_actual)

    def paso_simulacion(self): # Usa self.config_actual
        if not self.corriendo_simulacion: return 
        dt = self.config_actual.DELTA_T
        self.tiempo_simulacion_total += dt
        for obs in self.obstaculos: obs.actualizar(dt, self.rng_obstaculos_dinamicos) # Obstaculo usa su config_propia
        self.tiempo_desde_ultimo_obstaculo_generado += dt
        if self.tiempo_desde_ultimo_obstaculo_generado >= self.config_actual.GENERAR_NUEVOS_OBSTACULOS_INTERVALO:
            self._generar_un_obstaculo(); self.tiempo_desde_ultimo_obstaculo_generado = 0.0
        
        obstaculos_activos_para_fuerzas = [obs for obs in self.obstaculos if obs.esta_activo]
        drones_activos_para_calculo_fuerzas = [d for d in self.drones if d.esta_activo]

        for dron_actual in self.drones:
            if dron_actual.esta_activo:
                otros_drones_activos = [d for d in drones_activos_para_calculo_fuerzas if d.id != dron_actual.id]
                dron_actual.calcular_fuerzas( # Drone usa su config_propia
                    otros_drones_activos, obstaculos_activos_para_fuerzas,
                    self.grilla_cobertura, self.config_actual.TAMANO_CELDA_COBERTURA,
                    self.num_celdas_x, self.num_celdas_y,
                    self.rng_drones_decisiones
                )
            else: dron_actual.fuerza_actual = np.array([0.0,0.0])
        
        if self.config_actual.CBF_ACTIVADO: self._aplicar_cbfs()
        for dron_actual in self.drones: self._integrador_rk4_paso(dron_actual, dt)
        self._detectar_y_manejar_colisiones()
        self.actualizar_grilla_cobertura()

    def controlar_eventos_pygame(self, eventos): # Usa self.config_actual
        for evento in eventos:
            if evento.type == pygame.QUIT:
                self.game_loop_activo = False; self.corriendo_simulacion = False
            if evento.type == pygame.KEYDOWN:
                nombre_param_actual = self.nombres_parametros_editables[self.parametro_seleccionado_idx]
                definicion_actual = self.parametros_editables_definicion[nombre_param_actual]

                if self.modo_edicion_texto_param:
                    if evento.key == pygame.K_RETURN:
                        try:
                            val_a_setear = None
                            if self.buffer_texto_param.lower() == "none" or self.buffer_texto_param == "":
                                val_a_setear = None
                            elif definicion_actual["tipo"] == "semilla_ms": 
                                val_a_setear = int(self.buffer_texto_param)
                                n_digits = self.config_actual.N_DIGITS_MIDDLE_SQUARE
                                if not (0 <= val_a_setear < 10**n_digits):
                                    if self.config_actual.VERBOSE: print(f"Advertencia: Semilla MS {val_a_setear} fuera de rango para {n_digits} dígitos.")
                            else: 
                                val_a_setear = int(self.buffer_texto_param)
                            
                            setattr(self.config_actual, nombre_param_actual, val_a_setear)
                            if self.config_actual.VERBOSE: print(f"Parámetro {nombre_param_actual} establecido a: {val_a_setear}")
                        except ValueError:
                            if self.config_actual.VERBOSE: print(f"Error: '{self.buffer_texto_param}' no es una semilla válida para {nombre_param_actual}.")
                        self.modo_edicion_texto_param = False; self.buffer_texto_param = ""
                    elif evento.key == pygame.K_BACKSPACE:
                        self.buffer_texto_param = self.buffer_texto_param[:-1]
                    elif evento.unicode.isdigit() or (self.buffer_texto_param == "" and evento.unicode == "-") or \
                         (self.buffer_texto_param == "" and definicion_actual["tipo"] == "semilla" and evento.unicode.lower() == "n") or \
                         (self.buffer_texto_param.lower() == "n" and definicion_actual["tipo"] == "semilla" and evento.unicode.lower() == "o") or \
                         (self.buffer_texto_param.lower() == "no" and definicion_actual["tipo"] == "semilla" and evento.unicode.lower() == "n") or \
                         (self.buffer_texto_param.lower() == "non" and definicion_actual["tipo"] == "semilla" and evento.unicode.lower() == "e"):
                        self.buffer_texto_param += evento.unicode # Permite escribir "None"
                    elif evento.key == pygame.K_ESCAPE:
                        self.modo_edicion_texto_param = False; self.buffer_texto_param = ""
                else: 
                    if evento.key == self.config_actual.TECLA_PAUSA_REANUDAR:
                        self.corriendo_simulacion = not self.corriendo_simulacion
                        if self.config_actual.VERBOSE: print(f"Simulación {'reanudada' if self.corriendo_simulacion else 'pausada'}.")
                    elif evento.key == self.config_actual.TECLA_RESETEAR:
                        self._reset_simulation_state() 
                        if self.config_actual.VERBOSE: print("Simulación reseteada con parámetros actuales.")
                    elif evento.key == self.config_actual.TECLA_ANADIR_DRON: self._crear_drones_iniciales(cantidad=1)
                    elif evento.key == self.config_actual.TECLA_QUITAR_DRON: self._quitar_dron()
                    elif evento.key == self.config_actual.TECLA_EJECUTAR_RNG_TESTS: self.ejecutar_y_mostrar_pruebas_rng_consola()
                    elif evento.key == pygame.K_UP:
                        self.parametro_seleccionado_idx = (self.parametro_seleccionado_idx - 1) % len(self.nombres_parametros_editables)
                        if self.config_actual.VERBOSE: print(f"Seleccionado: {self.nombres_parametros_editables[self.parametro_seleccionado_idx]}")
                    elif evento.key == pygame.K_DOWN:
                        self.parametro_seleccionado_idx = (self.parametro_seleccionado_idx + 1) % len(self.nombres_parametros_editables)
                        if self.config_actual.VERBOSE: print(f"Seleccionado: {self.nombres_parametros_editables[self.parametro_seleccionado_idx]}")
                    elif evento.key == pygame.K_LEFT or evento.key == pygame.K_MINUS:
                        if definicion_actual["tipo"] != "semilla" and definicion_actual["tipo"] != "semilla_ms":
                            valor_actual = getattr(self.config_actual, nombre_param_actual)
                            nuevo_valor = valor_actual - definicion_actual["paso"]
                            if definicion_actual["tipo"] == int: nuevo_valor = int(round(nuevo_valor))
                            else: nuevo_valor = round(nuevo_valor, 2) 
                            setattr(self.config_actual, nombre_param_actual, max(definicion_actual.get("min", nuevo_valor), nuevo_valor))
                            if self.config_actual.VERBOSE: print(f"{nombre_param_actual} = {getattr(self.config_actual, nombre_param_actual)}")
                    elif evento.key == pygame.K_RIGHT or evento.key == pygame.K_EQUALS or evento.key == pygame.K_PLUS:
                        if definicion_actual["tipo"] != "semilla" and definicion_actual["tipo"] != "semilla_ms":
                            valor_actual = getattr(self.config_actual, nombre_param_actual)
                            nuevo_valor = valor_actual + definicion_actual["paso"]
                            if definicion_actual["tipo"] == int: nuevo_valor = int(round(nuevo_valor))
                            else: nuevo_valor = round(nuevo_valor, 2)
                            setattr(self.config_actual, nombre_param_actual, min(definicion_actual.get("max", nuevo_valor), nuevo_valor))
                            if self.config_actual.VERBOSE: print(f"{nombre_param_actual} = {getattr(self.config_actual, nombre_param_actual)}")
                    elif evento.key == pygame.K_RETURN:
                        if definicion_actual["tipo"] == "semilla" or definicion_actual["tipo"] == "semilla_ms":
                            self.modo_edicion_texto_param = True
                            current_val = getattr(self.config_actual, nombre_param_actual)
                            self.buffer_texto_param = str(current_val) if current_val is not None else ""
                            if self.config_actual.VERBOSE: print(f"Editando {nombre_param_actual}. Buffer: '{self.buffer_texto_param}'. Escriba valor y Enter, o Esc.")
                            
    def dibujar_elementos_pygame(self, pantalla_pygame, font_pygame_metrics, font_pygame_params): # Usa self.config_actual
        pantalla_pygame.fill(self.config_actual.GRIS_CLARO)
        for x_idx in range(self.num_celdas_x):
            for y_idx in range(self.num_celdas_y):
                rect = pygame.Rect(x_idx * self.config_actual.TAMANO_CELDA_COBERTURA, y_idx * self.config_actual.TAMANO_CELDA_COBERTURA,
                                   self.config_actual.TAMANO_CELDA_COBERTURA, self.config_actual.TAMANO_CELDA_COBERTURA)
                color_celda = self.config_actual.COLOR_CELDA_CUBIERTA if self.grilla_cobertura[x_idx, y_idx] == 1 \
                              else self.config_actual.COLOR_CELDA_NO_CUBIERTA
                pygame.draw.rect(pantalla_pygame, color_celda, rect)
        
        for obs in self.obstaculos: obs.dibujar(pantalla_pygame) 
        for dron in self.drones: dron.dibujar(pantalla_pygame)
            
        drones_activos_count = sum(1 for d in self.drones if d.esta_activo)
        drones_inactivos_count = len(self.drones) - drones_activos_count
        metric_texts = [
            f"Tiempo: {self.tiempo_simulacion_total:.2f}s", f"Cobertura: {self.porcentaje_cobertura:.2f}%",
            f"Drones Activos: {drones_activos_count}", f"Drones Inactivos: {drones_inactivos_count}",
            f"Colisiones Críticas: {self.colisiones_criticas_contador}",
            f"Activaciones CBF: {cbf_module.get_cbf_activation_count()}"
        ]
        y_offset_metrics = 10
        for text_line in metric_texts:
            surface = font_pygame_metrics.render(text_line, True, self.config_actual.NEGRO)
            pantalla_pygame.blit(surface, (10, y_offset_metrics)); y_offset_metrics += 20
        
        param_area_y_start = y_offset_metrics + 10 
        param_line_height = 18 
        
        titulo_params_str = "Params (Up/Down, L/R o -/+, Enter:Seed, R:Reset):"
        if self.modo_edicion_texto_param:
            titulo_params_str = f"Editando: {self.nombres_parametros_editables[self.parametro_seleccionado_idx]} (Enter/Esc)"
        
        titulo_params_surface = self.font_instructions.render(titulo_params_str, True, self.config_actual.NEGRO, self.config_actual.BLANCO) # Usar font_instructions
        pantalla_pygame.blit(titulo_params_surface, (10, param_area_y_start))
        param_area_y_start += 20 # Un poco más de espacio

        for i, nombre_param_key in enumerate(self.nombres_parametros_editables):
            definicion_param = self.parametros_editables_definicion[nombre_param_key]
            display_name = definicion_param['display_name']
            valor_actual = getattr(self.config_actual, nombre_param_key, "N/A")
            display_valor_str = ""
            if definicion_param["tipo"] == "semilla" or definicion_param["tipo"] == "semilla_ms":
                display_valor_str = str(valor_actual) if valor_actual is not None else "None"
            elif definicion_param["tipo"] == float: display_valor_str = f"{valor_actual:.1f}"
            else: display_valor_str = str(valor_actual)

            text_str = f"{'>' if i == self.parametro_seleccionado_idx else ' '}{display_name[:18].ljust(18)}: {display_valor_str.rjust(7)}"
            if self.modo_edicion_texto_param and i == self.parametro_seleccionado_idx:
                text_str = f">{display_name[:18].ljust(18)}: {self.buffer_texto_param}_"

            color_texto_param = self.config_actual.ROJO if i == self.parametro_seleccionado_idx else self.config_actual.NEGRO
            surface = font_pygame_params.render(text_str, True, color_texto_param, self.config_actual.BLANCO)
            
            if param_area_y_start + i * param_line_height < self.config_actual.ALTO_PANTALLA - param_line_height:
                 pantalla_pygame.blit(surface, (15, param_area_y_start + i * param_line_height))

        if not self.corriendo_simulacion:
            pause_text_surface = font_pygame_metrics.render("PAUSADO (Espacio)", True, self.config_actual.ROJO)
            text_rect = pause_text_surface.get_rect(center=(self.config_actual.ANCHO_PANTALLA // 2, 20))
            pantalla_pygame.blit(pause_text_surface, text_rect)
        pygame.display.flip()

    def bucle_principal_pygame(self): # Nombre corregido
        # pygame.init() y pygame.font.init() ya están en self.__init__
        # self.pantalla y self.reloj ya están en self
        # Las fuentes también están en self (self.font_metrics, self.font_params)

        self.corriendo_simulacion = True 
        self.game_loop_activo = True
        while self.game_loop_activo:
            eventos_pygame = pygame.event.get()
            self.controlar_eventos_pygame(eventos_pygame)
            if self.corriendo_simulacion:
                self.paso_simulacion()
            self.dibujar_elementos_pygame(self.pantalla, self.font_metrics, self.font_params) # Pasar fuentes
            self.reloj.tick(self.config_actual.FPS)
        pygame.quit()

    def ejecutar_y_mostrar_pruebas_rng_consola(self): # Usa self.config_actual
        # from .rng_validator import perform_rng_quality_tests_from_scratch # Ya está importado arriba
        print("\n--- Ejecutando Pruebas de Calidad RNG (Desde Cero - Consola) ---")
        rngs_para_probar = [
            ("GCL_ENTORNO", self.rng_entorno, 
             {'a': self.config_actual.GCL_MULTIPLIER_A, 'c': self.config_actual.GCL_INCREMENT_C, 'm': self.config_actual.GCL_MODULUS_M}),
            ("MIDDLE_SQUARE_DRONES", self.rng_drones_decisiones, {'digits': self.config_actual.N_DIGITS_MIDDLE_SQUARE}),
            ("GCL_OBSTACULOS_DINAMICOS", self.rng_obstaculos_dinamicos,
             {'a': self.config_actual.GCL_MULTIPLIER_A_OBS, 'c': self.config_actual.GCL_INCREMENT_C_OBS, 'm': self.config_actual.GCL_MODULUS_M_OBS})
        ]
        for nombre, instancia_sim, params_rng in rngs_para_probar:
            print(f"\n--- Pruebas para {nombre} (Semilla Inicial de Sim: {instancia_sim.initial_seed}) ---")
            instancia_test = None
            if isinstance(instancia_sim, LCG):
                instancia_test = LCG(seed=instancia_sim.initial_seed, multiplier=params_rng['a'], increment=params_rng['c'], modulus=params_rng['m'])
            elif isinstance(instancia_sim, MiddleSquareRNG):
                instancia_test = MiddleSquareRNG(seed=instancia_sim.initial_seed, num_digits=params_rng['digits'])
            else: continue
            
            resultados = perform_rng_quality_tests_from_scratch(instancia_test, self.config_actual.RNG_TEST_NUM_SAMPLES)
            
            for test_name_key, result_data in resultados.items():
                if test_name_key in ['rng_type', 'initial_seed_for_test_sequence', 'num_samples_tested']:
                    print(f"  {test_name_key.replace('_', ' ').title()}: {result_data}")
                    continue
                print(f"  Test: {result_data.get('test_name', test_name_key)}")
                if isinstance(result_data, dict):
                    if 'error' in result_data: print(f"    Error: {result_data['error']}")
                    else:
                        for k, v in result_data.items():
                            if k == 'test_name': continue
                            print(f"    {k.replace('_', ' ').title()}: {v:.4f}" if isinstance(v, float) else f"    {k.replace('_', ' ').title()}: {v}")
                else: print(f"    {result_data}")
        print("-----------------------------------------------------\n")

    # --- Métodos para Dash (Placeholder por ahora si no se usan) ---
    def start_sim(self): self.corriendo_simulacion = True
    def pause_sim(self): self.corriendo_simulacion = False
    def reset_sim_from_dash(self, params_dash=None): self._reset_simulation_state()
    def get_estado_para_dash(self) -> dict: return {} 
    def ejecutar_pruebas_rng_para_dash(self) -> dict: return {}