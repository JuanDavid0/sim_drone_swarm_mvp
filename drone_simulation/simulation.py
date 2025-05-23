import pygame
import numpy as np
import json
import os
import sys
from . import config as defaultConfig_module
from .drone import Drone
from .obstaculo import Obstaculo
from . import cbf as cbf_module
from .rng import LCG, MiddleSquareRNG
from .rng_validator import perform_rng_quality_tests_from_scratch
import types
import matplotlib.pyplot as plt
from datetime import datetime
import subprocess

class Simulation:
    def __init__(self):
        # 1) Inicializamos config_actual con defaults
        self.config_actual = types.SimpleNamespace()
        for attr in dir(defaultConfig_module):
            if not attr.startswith("__"):
                setattr(self.config_actual, attr, getattr(defaultConfig_module, attr))

        # 2) Si pasaron la flag y existe el JSON, sobrescribimos
        if "--use-runtime-config" in sys.argv and os.path.exists("config_runtime.json"):
            runtime = json.load(open("config_runtime.json", "r"))
            for key, val in runtime.items():
                if hasattr(self.config_actual, key):
                    setattr(self.config_actual, key, val)

        # 3) Inicialización de Pygame
        pygame.init()
        pygame.font.init()
        self.pantalla = pygame.display.set_mode((
            self.config_actual.ANCHO_PANTALLA,
            self.config_actual.ALTO_PANTALLA
        ))
        pygame.display.set_caption("Sim Enjambre - Pygame")
        self.reloj = pygame.time.Clock()

        # 4) Banderas de control
        self.corriendo_simulacion = False
        self.game_loop_activo    = True

        # 5) Instancias RNG con las semillas ya cargadas
        self.rng_entorno = LCG(
            seed      = self.config_actual.GCL_SEED_ENTORNO,
            multiplier= self.config_actual.GCL_MULTIPLIER_A,
            increment = self.config_actual.GCL_INCREMENT_C,
            modulus   = self.config_actual.GCL_MODULUS_M
        )
        self.rng_drones_decisiones = MiddleSquareRNG(
            seed      = self.config_actual.MIDDLE_SQUARE_SEED_DRONES,
            num_digits= self.config_actual.N_DIGITS_MIDDLE_SQUARE
        )
        self.rng_obstaculos_dinamicos = LCG(
            seed      = self.config_actual.GCL_SEED_OBSTACULOS_DYN,
            multiplier= self.config_actual.GCL_MULTIPLIER_A_OBS,
            increment = self.config_actual.GCL_INCREMENT_C_OBS,
            modulus   = self.config_actual.GCL_MODULUS_M_OBS
        )

        if self.config_actual.VERBOSE:
            print("Simulación instanciada con:")
            print("  RNG Entorno — semilla:", self.rng_entorno.initial_seed)
            print("  RNG Drones — semilla:", self.rng_drones_decisiones.initial_seed)
            print("  RNG Obstáculos — semilla:", self.rng_obstaculos_dinamicos.initial_seed)

        # 6) Preparar grilla, contadores y fuentes
        self.drones  = []
        self.obstaculos = []
        self.num_celdas_x = self.config_actual.ANCHO_PANTALLA // self.config_actual.TAMANO_CELDA_COBERTURA
        self.num_celdas_y = self.config_actual.ALTO_PANTALLA  // self.config_actual.TAMANO_CELDA_COBERTURA
        self.grilla_cobertura = np.zeros((self.num_celdas_x, self.num_celdas_y), dtype=int)
        self.celdas_totales_cobertura = self.num_celdas_x * self.num_celdas_y
        self.porcentaje_cobertura = 0.0
        self.tiempo_simulacion_total  = 0.0
        self.colisiones_criticas_contador = 0
        self.font_metrics     = pygame.font.SysFont(None, 22)
        self.font_params = pygame.font.SysFont("Consolas", 15)
        self.font_instructions= pygame.font.SysFont(None, 18)

        # 7) Estado inicial de la simulación
        self._reset_simulation_state(inicial=True)
    
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
        if not inicial:
            self.export_rng_test_results_as_pdf()
        if not inicial:
            try:
                subprocess.Popen([sys.executable, "rng_dashboard.py"])
                if self.config_actual.VERBOSE:
                    print("[✓] Dashboard RNG lanzado en ventana separada.")
            except Exception as e:
                print(f"[X] Error al lanzar dashboard RNG: {e}")


        
    # Preparar el módulo para exportar gráficos de pruebas RNG desde la simulación automáticamente
    def export_rng_test_results_as_pdf(self):
        os.makedirs("rng_reports", exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filepath = os.path.join("rng_reports", f"rng_report_{timestamp}.pdf")

        from .rng_validator import perform_rng_quality_tests_from_scratch
        from .rng import LCG, MiddleSquareRNG

        resultados = {
            "Entorno": perform_rng_quality_tests_from_scratch(
                LCG(seed=self.rng_entorno.initial_seed,
                    multiplier=self.config_actual.GCL_MULTIPLIER_A,
                    increment=self.config_actual.GCL_INCREMENT_C,
                    modulus=self.config_actual.GCL_MODULUS_M),
                self.config_actual.RNG_TEST_NUM_SAMPLES
            ),
            "Drones": perform_rng_quality_tests_from_scratch(
                MiddleSquareRNG(seed=self.rng_drones_decisiones.initial_seed,
                                num_digits=self.config_actual.N_DIGITS_MIDDLE_SQUARE),
                self.config_actual.RNG_TEST_NUM_SAMPLES
            ),
            "Obstaculos": perform_rng_quality_tests_from_scratch(
                LCG(seed=self.rng_obstaculos_dinamicos.initial_seed,
                    multiplier=self.config_actual.GCL_MULTIPLIER_A_OBS,
                    increment=self.config_actual.GCL_INCREMENT_C_OBS,
                    modulus=self.config_actual.GCL_MODULUS_M_OBS),
                self.config_actual.RNG_TEST_NUM_SAMPLES
            )
        }

        fig, axes = plt.subplots(3, 1, figsize=(8, 10))
        fig.suptitle("Informe de Pruebas RNG (Simulación)", fontsize=14)

        # Chi²
        chi2 = resultados["Entorno"]["chi_squared_uniformity"]
        axes[0].bar(range(len(chi2["observed_counts"])), chi2["observed_counts"], label="Observado")
        axes[0].axhline(chi2["expected_per_bin"], color='r', linestyle='--', label="Esperado")
        axes[0].set_title("Chi² - Entorno")
        axes[0].legend()

        # K-S
        ks = resultados["Drones"]["kolmogorov_smirnov_uniformity"]
        sorted_samples = sorted([self.rng_drones_decisiones.next_float() for _ in range(1000)])
        ecdf_y = [i / len(sorted_samples) for i in range(1, len(sorted_samples)+1)]
        axes[1].plot(sorted_samples, ecdf_y, label="ECDF")
        axes[1].plot(sorted_samples, sorted_samples, linestyle="--", label="CDF Teórica")
        axes[1].set_title("Kolmogorov-Smirnov - Drones")
        axes[1].legend()

        # Autocorrelación
        autocorr = resultados["Obstaculos"]["autocorrelation_lag1_numpy"].get("value", 0)
        axes[2].text(0.1, 0.5, f"Autocorrelación lag-1:\n{autocorr:.4f}", fontsize=14)
        axes[2].axis("off")

        plt.tight_layout(rect=[0, 0, 1, 0.95])
        fig.savefig(filepath)
        plt.close(fig)

        if self.config_actual.VERBOSE:
            print(f"[✓] Informe de pruebas RNG guardado en: {filepath}")

        # Lanzar dashboard si existe el archivo
        dashboard_path = os.path.join(os.path.dirname(__file__), "..", "rng_dashboard.py")
        dashboard_path = os.path.abspath(dashboard_path)

        if os.path.exists(dashboard_path):
            try:
                subprocess.Popen(["python", dashboard_path, filepath])
                if self.config_actual.VERBOSE:
                    print("[✓] Dashboard RNG lanzado.")
            except Exception as e:
                print(f"[!] Error al lanzar el dashboard: {e}")
        else:
            print(f"[!] No se encontró el dashboard en: {dashboard_path}")



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

    def controlar_eventos_pygame(self, eventos):  # Usa self.config_actual
        for evento in eventos:
            if evento.type == pygame.QUIT:
                self.game_loop_activo = False
                self.corriendo_simulacion = False

            elif evento.type == pygame.KEYDOWN:
                if evento.key == self.config_actual.TECLA_PAUSA_REANUDAR:
                    self.corriendo_simulacion = not self.corriendo_simulacion
                    if self.config_actual.VERBOSE:
                        estado = "reanudada" if self.corriendo_simulacion else "pausada"
                        print(f"Simulación {estado}.")

                elif evento.key == self.config_actual.TECLA_RESETEAR:
                    self._reset_simulation_state()
                    if self.config_actual.VERBOSE:
                        print("Simulación reseteada.")

                elif evento.key == self.config_actual.TECLA_ANADIR_DRON:
                    self._crear_drones_iniciales(cantidad=1)

                elif evento.key == self.config_actual.TECLA_QUITAR_DRON:
                    self._quitar_dron()

                elif evento.key == self.config_actual.TECLA_EJECUTAR_RNG_TESTS:
                    self.ejecutar_y_mostrar_pruebas_rng_consola()

                            
    def dibujar_elementos_pygame(self, pantalla_pygame, font_pygame_metrics, font_pygame_params):
        pantalla_pygame.fill(self.config_actual.GRIS_CLARO)

        # Dibujar grilla, obstáculos y drones
        for x in range(self.num_celdas_x):
            for y in range(self.num_celdas_y):
                rect = pygame.Rect(
                    x * self.config_actual.TAMANO_CELDA_COBERTURA,
                    y * self.config_actual.TAMANO_CELDA_COBERTURA,
                    self.config_actual.TAMANO_CELDA_COBERTURA,
                    self.config_actual.TAMANO_CELDA_COBERTURA
                )
                color = (self.config_actual.COLOR_CELDA_CUBIERTA
                        if self.grilla_cobertura[x, y] == 1
                        else self.config_actual.COLOR_CELDA_NO_CUBIERTA)
                pygame.draw.rect(pantalla_pygame, color, rect)

        for obs in self.obstaculos:
            obs.dibujar(pantalla_pygame)
        for dron in self.drones:
            dron.dibujar(pantalla_pygame)

        # Métricas en pantalla
        drones_act = sum(1 for d in self.drones if d.esta_activo)
        drones_inact = len(self.drones) - drones_act
        metrics = [
            f"Tiempo: {self.tiempo_simulacion_total:.2f}s",
            f"Cobertura: {self.porcentaje_cobertura:.2f}%",
            f"Drones Activos: {drones_act}",
            f"Drones Inactivos: {drones_inact}",
            f"Colisiones Críticas: {self.colisiones_criticas_contador}",
            f"Activaciones CBF: {cbf_module.get_cbf_activation_count()}"
        ]
        y_off = 10
        for line in metrics:
            surf = font_pygame_metrics.render(line, True, self.config_actual.NEGRO)
            pantalla_pygame.blit(surf, (10, y_off))
            y_off += 20

        # Mensaje de pausa
        if not self.corriendo_simulacion:
            pause_s = font_pygame_metrics.render("PAUSADO (Espacio)", True, self.config_actual.ROJO)
            rc = pause_s.get_rect(center=(self.config_actual.ANCHO_PANTALLA // 2, 20))
            pantalla_pygame.blit(pause_s, rc)
            
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
