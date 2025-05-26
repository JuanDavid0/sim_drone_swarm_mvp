import pygame

# Dimensiones de la pantalla de Pygame
ANCHO_PANTALLA = 800
ALTO_PANTALLA = 700

# Colores (formato RGB)
BLANCO = (255, 255, 255)
NEGRO = (0, 0, 0)
ROJO = (255, 0, 0)
VERDE = (0, 255, 0)
AZUL = (0, 0, 255)
GRIS_CLARO = (220, 220, 220)

# Parámetros de simulación
FPS = 60
DELTA_T = 1 / FPS

# Parámetros de los Drones
NUM_DRONES_INICIAL = 15        # Más drones para apreciar comportamientos
RADIO_DRONE = 12               # Radio en px para colisiones claras
# VELOCIDAD_DRONE_INICIAL ya no se usa para movimiento principal

# --- Parámetros de Fuerza ---
K_COHESION = 0.5               # Atracción grupal moderada
K_SEPARATION = 800.0           # Repulsión fuerte para evitar choques
K_ALIGNMENT = 0.7              # Alineamiento con vecinos efectivo
K_FRONTIER_ATTRACTION = 0.8    # Exploración activa de fronteras
# El multiplicador interno en drone.py sigue en 25

# Parámetros para cálculo de fuerzas
DISTANCIA_SEPARACION_MIN = RADIO_DRONE * 3.0
EPSILON_FUERZA = 1e-6
MASA_DRONE = 10.0               # Más liviano para mayor maniobrabilidad
MAX_FUERZA = 100.0              # Fuerzas suaves pero visibles
MAX_VELOCIDAD = 120.0           # Velocidad máxima más ágil

# --- Parámetros de Obstáculos ---
MIN_TAMANO_OBSTACULO = 25
MAX_TAMANO_OBSTACULO = 50
RADIO_OBSTACULO = 30
NUM_OBSTACULOS = 6
K_OBSTACLE_REPULSION = 1500.0   # Evitar obstáculos con claridad
DISTANCIA_REACCION_OBSTACULO = RADIO_DRONE + MAX_TAMANO_OBSTACULO + 10

OBSTACULOS_DINAMICOS_PORCENTAJE = 0.3
TIEMPO_VIDA_OBSTACULO_MIN = 10.0
TIEMPO_VIDA_OBSTACULO_MAX = 20.0
TIEMPO_RESPAWN_OBSTACULO_MIN = 5.0
TIEMPO_RESPAWN_OBSTACULO_MAX = 15.0
GENERAR_NUEVOS_OBSTACULOS_INTERVALO = 25.0
MAX_OBSTACULOS_SIMULTANEOS = 8

# --- Parámetros de Exploración de Frontera ---
SENSOR_RANGE_DRONE = 200
RADIO_BUSQUEDA_FRONTERA_DRONE = 300

# --- Parámetros de Cobertura ---
TAMANO_CELDA_COBERTURA = 25
COLOR_CELDA_NO_CUBIERTA = (200, 200, 200)
COLOR_CELDA_CUBIERTA = (173, 216, 230)

# --- Parámetros de Fallo y Colisión ---
PROBABILIDAD_FALLO_POR_COLISION_OBSTACULO = 0.3
PROBABILIDAD_FALLO_POR_COLISION_DRON = 0.9
COLOR_DRON_INACTIVO = (100, 100, 100)
DISTANCIA_COLISION_DRON_DRON = 0
DISTANCIA_COLISION_DRON_OBSTACULO = RADIO_DRONE + RADIO_OBSTACULO * 0.7

# --- Parámetros para Repulsión de Bordes ---
K_BORDE_REPULSION = 1000.0
DISTANCIA_REACCION_BORDE = RADIO_DRONE * 5

# --- Parámetros de Control Barrier Functions (CBF - Etapa 6) ---
CBF_ACTIVADO = False
CBF_D_MIN_DRON_DRON = RADIO_DRONE * 1.5
CBF_D_MIN_DRON_OBSTACULO = RADIO_DRONE + RADIO_OBSTACULO + 5.0
CBF_GAMMA = 0.7
CBF_FACTOR_CORRECCION_VELOCIDAD = 1.0

TECLA_PAUSA_REANUDAR = pygame.K_SPACE
TECLA_RESETEAR = pygame.K_r
TECLA_ANADIR_DRON = pygame.K_a
TECLA_QUITAR_DRON = pygame.K_q

# --- Parámetros para Generadores Pseudoaleatorios (Etapa 7) ---
GCL_SEED_ENTORNO = None
GCL_MULTIPLIER_A = 1664525
GCL_INCREMENT_C = 1013904223
GCL_MODULUS_M = 2**32

MIDDLE_SQUARE_SEED_DRONES = None
N_DIGITS_MIDDLE_SQUARE = 4

GCL_SEED_OBSTACULOS_DYN = None
GCL_MULTIPLIER_A_OBS = 1103515245
GCL_INCREMENT_C_OBS = 12345
GCL_MODULUS_M_OBS = 2**31

# --- Parámetros para Pruebas RNG ---
RNG_TEST_NUM_SAMPLES = 10000
RNG_TEST_NUM_BINS_CHI2 = 10

TECLA_EJECUTAR_RNG_TESTS = pygame.K_t

VERBOSE = True # Mostrar detalles en consola