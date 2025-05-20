# drone_simulation/config.py

# Dimensiones de la pantalla de Pygame
ANCHO_PANTALLA = 800
ALTO_PANTALLA = 600

# Colores (formato RGB)
BLANCO = (255, 255, 255)
NEGRO = (0, 0, 0)
ROJO = (255, 0, 0)
VERDE = (0, 255, 0)
AZUL = (0, 0, 255)
GRIS_CLARO = (220, 220, 220) # Para el fondo

# Parámetros de simulación
FPS = 60  # Fotogramas por segundo deseados
DELTA_T = 1 / FPS  # Paso de tiempo para la simulación (inverso de FPS)

# Parámetros de los Drones (para Etapa 0)
NUM_DRONES_INICIAL = 2
RADIO_DRONE = 10
VELOCIDAD_DRONE_INICIAL = 50 # Píxeles por segundo (para movimiento simple)

# drone_simulation/config.py
# ... (configuraciones existentes) ...

# --- Parámetros de Fuerza (Etapa 1) ---
# Constantes de fuerza (necesitarán ajuste)
K_COHESION = 1 # Atracción hacia el centro de los vecinos
K_SEPARATION = 1500.0 # Repulsión de vecinos cercanos (más grande porque es inversamente proporcional al cuadrado de la dist.)
K_ATTRACTION_GOAL = 0.6 # Atracción hacia el punto objetivo

# Parámetros para fuerzas
DISTANCIA_SEPARACION_MIN = RADIO_DRONE * 4 # Distancia dentro de la cual se aplica la fuerza de separación más fuerte
EPSILON_FUERZA = 1e-6     # Pequeña constante para evitar división por cero en fuerza de separación
MASA_DRONE = 3.0          # Masa del dron (ajustar según la magnitud de las fuerzas)
MAX_FUERZA = 100.0        # Límite opcional para la fuerza total
MAX_VELOCIDAD = 150.0     # Límite para la velocidad del dron (píxeles/segundo)

# Punto objetivo simple para Etapa 1
PUNTO_OBJETIVO_X = ANCHO_PANTALLA / 2
PUNTO_OBJETIVO_Y = ALTO_PANTALLA / 2

# --- Parámetros de Fuerza (Etapa 2 - Adicionales) ---
K_ALIGNMENT = 1.2         # Fuerza para alinear velocidad con vecinos
K_OBSTACLE_REPULSION = 2500.0 # Fuerza de repulsión de obstáculos
RADIO_OBSTACULO = 10      # Radio de los obstáculos circulares simples
NUM_OBSTACULOS = 3
DISTANCIA_REACCION_OBSTACULO = RADIO_DRONE + RADIO_OBSTACULO + 50 # Distancia para empezar a reaccionar al obstáculo

# Distancia para considerar vecinos para cohesión y alineación (sensor_range simple)
SENSOR_RANGE_DRONE = 150

# Etapa 3
TAMANO_CELDA_COBERTURA = 20 # Tamaño de cada celda de la grilla en píxeles
COLOR_CELDA_NO_CUBIERTA = (200, 200, 200) # Un gris un poco más oscuro que el fondo
COLOR_CELDA_CUBIERTA = (173, 216, 230)   # Azul claro para celdas cubiertas

# drone_simulation/config.py
# ... (configuraciones existentes) ...

# --- Parámetros de Fuerza (Ajustes y Adicionales para Etapa 4) ---
K_FRONTIER_ATTRACTION = 0.7 # Nueva constante para la atracción a la frontera
# K_ATTRACTION_GOAL ya no será el principal motor de exploración para todos los drones a la vez.

# --- Parámetros de Obstáculos (Etapa 4) ---
# RADIO_OBSTACULO ya existe
# NUM_OBSTACULOS ya existe
MIN_TAMANO_OBSTACULO = 15
MAX_TAMANO_OBSTACULO = 35
OBSTACULOS_DINAMICOS_PORCENTAJE = 0.3 # 30% de los obstáculos serán dinámicos
TIEMPO_VIDA_OBSTACULO_MIN = 10.0  # segundos
TIEMPO_VIDA_OBSTACULO_MAX = 20.0  # segundos
TIEMPO_RESPAWN_OBSTACULO_MIN = 5.0 # segundos que está inactivo antes de reaparecer
TIEMPO_RESPAWN_OBSTACULO_MAX = 15.0 # segundos
GENERAR_NUEVOS_OBSTACULOS_INTERVALO = 25.0 # Intervalo para generar un nuevo obstáculo dinámico (si hay espacio)
MAX_OBSTACULOS_SIMULTANEOS = 7 # Límite para no saturar

# --- Parámetros de Exploración de Frontera (Etapa 4) ---
RADIO_BUSQUEDA_FRONTERA_DRONE = 200 # En píxeles, qué tan lejos busca un dron una celda no cubierta

