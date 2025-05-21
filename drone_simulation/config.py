# drone_simulation/config.py (Propuesta Perfil 4 - Más Estable)
import pygame
# Dimensiones de la pantalla de Pygame
ANCHO_PANTALLA = 800
ALTO_PANTALLA = 600

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
NUM_DRONES_INICIAL = 5 # Un buen número para ver interacciones
RADIO_DRONE = 10
# VELOCIDAD_DRONE_INICIAL ya no se usa para movimiento principal

# --- Parámetros de Fuerza ---
K_COHESION = 0.4
K_SEPARATION = 1000.0 # Reducido
K_ALIGNMENT = 0.6
K_FRONTIER_ATTRACTION = 0.4 # Reducido, y el multiplicador en drone.py también se puede revisar
# El multiplicador '50' en drone.py para K_FRONTIER_ATTRACTION podría reducirse a 20 o 25.

# Parámetros para cálculo de fuerzas
DISTANCIA_SEPARACION_MIN = RADIO_DRONE * 4.5 # Un poco más para que se separen antes
EPSILON_FUERZA = 1e-6
MASA_DRONE = 15.0           # Aumentada SIGNIFICATIVAMENTE para más inercia
MAX_FUERZA = 80.0           # Reducido para movimientos más suaves
MAX_VELOCIDAD = 80.0        # Reducido

# --- Parámetros de Obstáculos ---
MIN_TAMANO_OBSTACULO = 20
MAX_TAMANO_OBSTACULO = 40
RADIO_OBSTACULO = 25 # Lo dejamos como un valor promedio si quieres variar el tamaño al crear
NUM_OBSTACULOS = 4
K_OBSTACLE_REPULSION = 2000.0 # Reducido
DISTANCIA_REACCION_OBSTACULO = RADIO_DRONE + MAX_TAMANO_OBSTACULO + 15 # Basado en el tamaño máximo

OBSTACULOS_DINAMICOS_PORCENTAJE = 0.4
TIEMPO_VIDA_OBSTACULO_MIN = 12.0
TIEMPO_VIDA_OBSTACULO_MAX = 25.0
TIEMPO_RESPAWN_OBSTACULO_MIN = 8.0
TIEMPO_RESPAWN_OBSTACULO_MAX = 18.0
GENERAR_NUEVOS_OBSTACULOS_INTERVALO = 20.0
MAX_OBSTACULOS_SIMULTANEOS = 6

# --- Parámetros de Exploración de Frontera ---
SENSOR_RANGE_DRONE = 180 # Aumentado para más interacción de enjambre
RADIO_BUSQUEDA_FRONTERA_DRONE = 250 # Aumentado

# --- Parámetros de Cobertura ---
TAMANO_CELDA_COBERTURA = 20
COLOR_CELDA_NO_CUBIERTA = (200, 200, 200)
COLOR_CELDA_CUBIERTA = (173, 216, 230)

# --- Parámetros de Fallo y Colisión ---
PROBABILIDAD_FALLO_POR_COLISION_OBSTACULO = 0.15 # Reducido para observar más tiempo
PROBABILIDAD_FALLO_POR_COLISION_DRON = 0.1    # Reducido
COLOR_DRON_INACTIVO = (100, 100, 100)
DISTANCIA_COLISION_DRON_DRON = RADIO_DRONE * 1.8 # Para que un contacto más leve cuente
DISTANCIA_COLISION_DRON_OBSTACULO = RADIO_DRONE + RADIO_OBSTACULO * 0.7 # Menos superposición para colisión

# --- Parámetros para Repulsión de Bordes ---
K_BORDE_REPULSION = 800.0  # Reducido para que no sea tan violenta
DISTANCIA_REACCION_BORDE = RADIO_DRONE * 6 # Que se active desde un poco más lejos

# --- Parámetros de Control Barrier Functions (CBF - Etapa 6) ---
CBF_ACTIVADO = True
CBF_D_MIN_DRON_DRON = RADIO_DRONE * 0.5  # Distancia de seguridad mínima entre centros de drones
CBF_D_MIN_DRON_OBSTACULO = RADIO_DRONE + RADIO_OBSTACULO + 5.0 # Distancia de seguridad mínima del centro del dron a la superficie del obstáculo (aproximado)
CBF_GAMMA = 0.5             # Parámetro de la CBF (qué tan rápido debe corregir, 0 < gamma <= 1)
# Para la corrección simplificada, no usaremos un QP, sino un ajuste directo.
CBF_FACTOR_CORRECCION_VELOCIDAD = 0.8 # Cuánto ajustar la velocidad para evitar colisión

# --- Teclas de Control (Etapa 6) ---
TECLA_PAUSA_REANUDAR = pygame.K_SPACE # Tecla Espacio
TECLA_RESETEAR = pygame.K_r
TECLA_ANADIR_DRON = pygame.K_a
TECLA_QUITAR_DRON = pygame.K_q

# --- Parámetros para Generadores Pseudoaleatorios (Etapa 7) ---

# GCL Principal (para el entorno, obstáculos generales)
GCL_SEED_ENTORNO = None # None para semilla basada en tiempo, o un entero
GCL_MULTIPLIER_A = 1664525
GCL_INCREMENT_C = 1013904223
GCL_MODULUS_M = 2**32

# Generador de Cuadrados Medios (para decisiones de drones, fallos)
MIDDLE_SQUARE_SEED_DRONES = None # Entero de N_DIGITS_MIDDLE_SQUARE dígitos. Si es None, generar uno.
N_DIGITS_MIDDLE_SQUARE = 4 # Número de dígitos para los números y la semilla (ej. 4 para números de 0000-9999)
                           # Semillas deben tener N_DIGITS_MIDDLE_SQUARE.
                           # Resultados tendrán 2*N_DIGITS_MIDDLE_SQUARE, se extraen los N del medio.

# (Opcional) GCL Secundario (para parámetros específicos de obstáculos dinámicos)
GCL_SEED_OBSTACULOS_DYN = None
GCL_MULTIPLIER_A_OBS = 1103515245 # Diferentes parámetros
GCL_INCREMENT_C_OBS = 12345
GCL_MODULUS_M_OBS = 2**31 # Puede ser diferente

# --- Parámetros para Pruebas RNG ---
RNG_TEST_NUM_SAMPLES = 10000
RNG_TEST_NUM_BINS_CHI2 = 10

# Teclas de Control (Adicional para Etapa 7)
TECLA_EJECUTAR_RNG_TESTS = pygame.K_t