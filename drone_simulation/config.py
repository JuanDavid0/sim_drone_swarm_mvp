import pygame

# Dimensiones de la pantalla de Pygame
ANCHO_PANTALLA = 800
ALTO_PANTALLA = 800

# Colores (formato RGB) - Se mantienen los que tenías
BLANCO = (219, 31, 0, 86)
NEGRO = (0, 0, 0, 86)
ROJO = (92, 52, 46, 86)
VERDE = (92, 46, 79, 86)
AZUL = (44, 217, 219, 86)
GRIS_CLARO = (89, 92, 77, 86)

# ==============================================================================
# --- VARIABLES "ESTABLES" ---
# Estas variables tendrán valores fijos para simplificar la experimentación.
# Se cargarán desde aquí si no se especifican o editan en la GUI/JSON.
# ==============================================================================

# Simulación
DELTA_T = 1 / 60.0  # Fijado a 1/60 (se puede recalcular si FPS cambia)

# Drones Físicos
RADIO_DRONE = 10.0
MASA_DRONE = 10.0 # Valor estable sugerido (el 2.0 que tenías era muy bajo)

# Fuerzas Internas
DISTANCIA_SEPARACION_MIN = RADIO_DRONE * 2.5  # 25.0
EPSILON_FUERZA = 1e-4

# Obstáculos Físicos
MIN_TAMANO_OBSTACULO = 10.0
MAX_TAMANO_OBSTACULO = 20.0
RADIO_OBSTACULO = MAX_TAMANO_OBSTACULO / 2.0
DISTANCIA_REACCION_OBSTACULO = RADIO_DRONE + MAX_TAMANO_OBSTACULO

# Obstáculos Dinámicos (Fijos)
TIEMPO_VIDA_OBSTACULO_MIN = 10.0
TIEMPO_VIDA_OBSTACULO_MAX = 20.0
TIEMPO_RESPAWN_OBSTACULO_MIN = 5.0
TIEMPO_RESPAWN_OBSTACULO_MAX = 15.0
MAX_OBSTACULOS_SIMULTANEOS = 10 # Se puede ajustar K_OBSTACLE_REPULSION si son muchos

# Exploración Fija
RADIO_BUSQUEDA_FRONTERA_DRONE = 400.0

# Colisiones Fijas
PROBABILIDAD_FALLO_POR_COLISION_OBSTACULO = 0.9 # Reducido para observar más el movimiento
PROBABILIDAD_FALLO_POR_COLISION_DRON = 0.9 # Reducido
DISTANCIA_COLISION_DRON_DRON = 2
DISTANCIA_COLISION_DRON_OBSTACULO = 2 # Aumentado para evitar colisiones frecuentes

# Bordes Fijos
K_BORDE_REPULSION = 2000.0
DISTANCIA_REACCION_BORDE = 10 # 20.0

# CBF Fijos
CBF_D_MIN_DRON_OBSTACULO = RADIO_DRONE + RADIO_OBSTACULO
CBF_GAMMA = 0.5 

# RNG Fijos (Mantén tus semillas para reproducibilidad)
GCL_SEED_ENTORNO = 946548256
GCL_MULTIPLIER_A = 1664525
GCL_INCREMENT_C = 1013904223
GCL_MODULUS_M = 2**32
MIDDLE_SQUARE_SEED_DRONES = 6453215
N_DIGITS_MIDDLE_SQUARE = 4
GCL_SEED_OBSTACULOS_DYN = 7485316
GCL_MULTIPLIER_A_OBS = 1103515245
GCL_INCREMENT_C_OBS = 12345
GCL_MODULUS_M_OBS = 2**31

# Otros Fijos
COLOR_CELDA_NO_CUBIERTA = (45, 134, 64, 53)
COLOR_CELDA_CUBIERTA = (29, 177, 61, 69)
COLOR_DRON_INACTIVO = (100, 100, 100)
TECLA_PAUSA_REANUDAR = pygame.K_SPACE
TECLA_RESETEAR = pygame.K_r
TECLA_ANADIR_DRON = pygame.K_a
TECLA_QUITAR_DRON = pygame.K_q
RNG_TEST_NUM_SAMPLES = 10000
RNG_TEST_NUM_BINS_CHI2 = 10
TECLA_EJECUTAR_RNG_TESTS = pygame.K_t
VERBOSE = True

# ==============================================================================
# --- VARIABLES "EDITABLES" ---
# Valores por defecto para el ESCENARIO 1: Masa Compacta.
# Comentarios indican valores para otros escenarios.
# ==============================================================================

# Simulación General
NUM_DRONES_INICIAL = 15     # Base: 15; Disperso: 10; Agresivo: 25; Cauteloso: 10
FPS = 60.0                  # Mantener a 60 para DELTA_T = 1/60

# Comportamiento del Enjambre
K_COHESION = 4.0            # Base: 1.0; Disperso: 0.2; Agresivo: 0.8; Cauteloso: 1.0
K_SEPARATION = 500.0        # Base: 1000; Disperso: 2000; Agresivo: 1200; Cauteloso: 800
K_ALIGNMENT = 1.5           # Base: 1.0; Disperso: 0.5; Agresivo: 0.8; Cauteloso: 1.0
K_FRONTIER_ATTRACTION = 100 # Base: 1.0; Disperso: 2.0; Agresivo: 3.0; Cauteloso: 0.5
SENSOR_RANGE_DRONE = 500.0  # Base: 150; Disperso: 200; Agresivo: 120; Cauteloso: 100; RangoCorto: 50; RangoLargo: 300
MAX_VELOCIDAD = 300.0       # Base: 300; Disperso: 350; Agresivo: 400; Cauteloso: 100
MAX_FUERZA = 150.0          # Base: 150; Disperso: 150; Agresivo: 200; Cauteloso: 50

# Obstáculos
NUM_OBSTACULOS = 5          # Base: 5; Agresivo: 15
K_OBSTACLE_REPULSION = 1500.0 # Base: 1000; Agresivo: 1500
OBSTACULOS_DINAMICOS_PORCENTAJE = 0.7 # Base: 0.2; Agresivo: 0.7
GENERAR_NUEVOS_OBSTACULOS_INTERVALO = 30.0 # Base: 30; Agresivo: 10

# Control Barrier Functions (CBF)
CBF_ACTIVADO = True         # SinCBF: False
CBF_D_MIN_DRON_DRON = 25.0  # Cauteloso: 35.0 (Base y otros: 25.0)
CBF_FACTOR_CORRECCION_VELOCIDAD = 0.2 # Cauteloso: 0.05 (Base y otros: 0.1)

# Cobertura
TAMANO_CELDA_COBERTURA = 50 # Agresivo: 20 (Base y otros: 25)

# ==============================================================================
# --- RECALCULO DE DELTA_T ---
# Asegurarse que DELTA_T siempre refleje FPS si FPS es editable.
# Si FPS es editable, es mejor calcular DELTA_T en el motor de simulación
# al inicio, basado en el config cargado. Si FPS se fija, esto es redundante.
# Por seguridad, mantenemos el cálculo aquí.
DELTA_T = 1 / FPS if FPS > 0 else 0.016667
# ==============================================================================