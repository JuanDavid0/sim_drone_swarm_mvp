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
NUM_DRONES_INICIAL = 5        # Más drones para apreciar comportamientos
RADIO_DRONE = 10               # Radio en px para colisiones claras
# VELOCIDAD_DRONE_INICIAL ya no se usa para movimiento principal

# --- Parámetros de Fuerza ---
K_COHESION = 1               # Atracción grupal moderada
K_SEPARATION = 3000.0           # Repulsión fuerte para evitar choques
K_ALIGNMENT = 1              # Alineamiento con vecinos efectivo
K_FRONTIER_ATTRACTION = 1    # Exploración activa de fronteras

# Parámetros para cálculo de fuerzas
DISTANCIA_SEPARACION_MIN = RADIO_DRONE * 2.5  # Separación mínima para evitar colisiones
EPSILON_FUERZA = 1e-6
MASA_DRONE = 10.0               # Más liviano para mayor maniobrabilidad
MAX_FUERZA = 100.0              # Fuerzas suaves pero visibles
MAX_VELOCIDAD = 250.0           # Velocidad máxima más ágil

# --- Parámetros de Obstáculos ---
MIN_TAMANO_OBSTACULO = 10
MAX_TAMANO_OBSTACULO = 20
RADIO_OBSTACULO = MAX_TAMANO_OBSTACULO / 2
NUM_OBSTACULOS = 5
K_OBSTACLE_REPULSION = 3000.0   # Evitar obstáculos con claridad
DISTANCIA_REACCION_OBSTACULO = RADIO_DRONE + MAX_TAMANO_OBSTACULO + 20 # Distancia de reacción más amplia

OBSTACULOS_DINAMICOS_PORCENTAJE = 1 # Porcentaje de obstáculos que serán dinámicos
TIEMPO_VIDA_OBSTACULO_MIN = 10.0
TIEMPO_VIDA_OBSTACULO_MAX = 20.0
TIEMPO_RESPAWN_OBSTACULO_MIN = 5.0
TIEMPO_RESPAWN_OBSTACULO_MAX = 15.0
GENERAR_NUEVOS_OBSTACULOS_INTERVALO = 25.0
MAX_OBSTACULOS_SIMULTANEOS = 8

# --- Parámetros de Exploración de Frontera ---
SENSOR_RANGE_DRONE = 400 # Rango de sensor del dron para detectar fronteras
RADIO_BUSQUEDA_FRONTERA_DRONE = 400

# --- Parámetros de Cobertura ---
TAMANO_CELDA_COBERTURA = 50 # Tamaño de cada celda en la cuadrícula de cobertura
COLOR_CELDA_NO_CUBIERTA = (255, 0, 0)
COLOR_CELDA_CUBIERTA = (0, 255, 0)

# --- Parámetros de Fallo y Colisión ---
PROBABILIDAD_FALLO_POR_COLISION_OBSTACULO = 0.99
PROBABILIDAD_FALLO_POR_COLISION_DRON = 0.99
COLOR_DRON_INACTIVO = (100, 100, 100)
DISTANCIA_COLISION_DRON_DRON = 2
DISTANCIA_COLISION_DRON_OBSTACULO = RADIO_DRONE + RADIO_OBSTACULO * 2

# --- Parámetros para Repulsión de Bordes ---
K_BORDE_REPULSION = 5000.0 # Fuerza de repulsión al borde ej: 5000.0 para evitar bordes
DISTANCIA_REACCION_BORDE = RADIO_DRONE * 2 # Distancia de reacción al borde

# --- Parámetros de Control Barrier Functions (CBF - Etapa 6) ---
CBF_ACTIVADO = True
CBF_D_MIN_DRON_DRON = RADIO_DRONE * 1.5
CBF_D_MIN_DRON_OBSTACULO = RADIO_DRONE + RADIO_OBSTACULO + 5.0
CBF_GAMMA = 0.7
CBF_FACTOR_CORRECCION_VELOCIDAD = 0.5 # Factor de corrección de velocidad para CBF ej: 1.0 para no modificar la velocidad, 0.5 para reducir a la mitad

TECLA_PAUSA_REANUDAR = pygame.K_SPACE
TECLA_RESETEAR = pygame.K_r
TECLA_ANADIR_DRON = pygame.K_a
TECLA_QUITAR_DRON = pygame.K_q

# --- Parámetros para Generadores Pseudoaleatorios (Etapa 7) ---
GCL_SEED_ENTORNO = 946548256 # Semilla para el generador congruencial lineal (GCL) del entorno ejemplo de una buena semilla es  
GCL_MULTIPLIER_A = 1664525
GCL_INCREMENT_C = 1013904223
GCL_MODULUS_M = 2**32

MIDDLE_SQUARE_SEED_DRONES = 6453215
N_DIGITS_MIDDLE_SQUARE = 4

GCL_SEED_OBSTACULOS_DYN = 7485316
GCL_MULTIPLIER_A_OBS = 1103515245
GCL_INCREMENT_C_OBS = 12345
GCL_MODULUS_M_OBS = 2**31

# --- Parámetros para Pruebas RNG ---
RNG_TEST_NUM_SAMPLES = 10000 
RNG_TEST_NUM_BINS_CHI2 = 10

TECLA_EJECUTAR_RNG_TESTS = pygame.K_t

VERBOSE = True # Mostrar detalles en consola