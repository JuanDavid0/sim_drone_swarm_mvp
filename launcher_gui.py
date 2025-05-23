import sys, os, json, importlib, subprocess
from PyQt5.QtWidgets import (
    QApplication, QWidget, QLabel, QVBoxLayout, QHBoxLayout,
    QScrollArea, QGroupBox, QFormLayout, QSpinBox, QDoubleSpinBox,
    QLineEdit, QPushButton, QSizePolicy
)
from PyQt5.QtCore import Qt

# Carga de valores por defecto y descripciones
def cargar_config_y_descripciones():
    cfg = importlib.import_module("drone_simulation.config")
    return {
        "Simulación": [
            ("NUM_DRONES_INICIAL",   int,   "Cantidad de drones que aparecen al iniciar la simulación"),
            ("FPS",                  float, "Velocidad de refresco de la simulación (frames por segundo)"),
            ("DELTA_T",              float, "Paso de tiempo entre cada frame (1/FPS)"),
        ],
        "Drones": [
            ("RADIO_DRONE",          float, "Radio (en píxeles) que ocupa cada dron en pantalla"),
            ("MASA_DRONE",           float, "Masa usada en el cálculo de aceleraciones (inercia)"),
            ("MAX_VELOCIDAD",        float, "Velocidad máxima permitida para cada dron"),
        ],
        "Fuerzas": [
            ("K_COHESION",           float, "Fuerza de atracción hacia el centro de los vecinos"),
            ("K_SEPARATION",         float, "Fuerza de repulsión para evitar aglomeraciones"),
            ("K_ALIGNMENT",          float, "Fuerza que alinea su velocidad con la de los vecinos"),
            ("K_FRONTIER_ATTRACTION",float, "Fuerza que empuja al dron hacia zonas sin cubrir"),
        ],
        "Obstáculos": [
            ("NUM_OBSTACULOS",               int,   "Número inicial de obstáculos estáticos"),
            ("MIN_TAMANO_OBSTACULO",         float, "Diámetro mínimo de los obstáculos generados"),
            ("MAX_TAMANO_OBSTACULO",         float, "Diámetro máximo de los obstáculos generados"),
            ("K_OBSTACLE_REPULSION",         float, "Constante de repulsión frente a obstáculos"),
            ("OBSTACULOS_DINAMICOS_PORCENTAJE", float, "Fracción de obstáculos que son dinámicos"),
            ("TIEMPO_VIDA_OBSTACULO_MIN",    float, "Tiempo mínimo que permanece activo un obstáculo dinámico (s)"),
            ("TIEMPO_VIDA_OBSTACULO_MAX",    float, "Tiempo máximo que permanece activo un obstáculo dinámico (s)"),
            ("TIEMPO_RESPAWN_OBSTACULO_MIN", float, "Retraso mínimo antes de regenerar un obstáculo dinámico (s)"),
            ("TIEMPO_RESPAWN_OBSTACULO_MAX", float, "Retraso máximo antes de regenerar un obstáculo dinámico (s)"),
            ("GENERAR_NUEVOS_OBSTACULOS_INTERVALO", float, "Intervalo entre creación automática de nuevos obstáculos (s)"),
            ("MAX_OBSTACULOS_SIMULTANEOS",   int,   "Límite de obstáculos activos en pantalla"),
        ],
        "Frontera": [
            ("SENSOR_RANGE_DRONE",          float, "Distancia máxima a la que detecta otros drones"),
            ("RADIO_BUSQUEDA_FRONTERA_DRONE", float, "Radio de búsqueda para localizar celdas no cubiertas"),
        ],
        "Cobertura": [
            ("TAMANO_CELDA_COBERTURA",      int,   "Tamaño de cada celda de la grilla de cobertura (píxeles)"),
        ],
        "Colisiones": [
            ("PROBABILIDAD_FALLO_POR_COLISION_OBSTACULO", float, "Probabilidad de inactivar un dron al chocar con obstáculo"),
            ("PROBABILIDAD_FALLO_POR_COLISION_DRON",      float, "Probabilidad de inactivar un dron al colisionar con otro dron"),
        ],
        "Bordes": [
            ("K_BORDE_REPULSION",           float, "Constante de repulsión cerca de los límites de la ventana"),
            ("DISTANCIA_REACCION_BORDE",    float, "Distancia desde el borde a la que empieza a repeler"),
        ],
        "CBF": [
            ("CBF_ACTIVADO",                int,   "0 = CBF desactivadas, 1 = activadas para evitar colisiones"),
            ("CBF_D_MIN_DRON_DRON",         float, "Distancia mínima de seguridad entre drones (CBF)"),
            ("CBF_D_MIN_DRON_OBSTACULO",    float, "Distancia mínima de seguridad dron-obstáculo (CBF)"),
            ("CBF_GAMMA",                   float, "Tasa de corrección de la función de barrera (γ)"),
            ("CBF_FACTOR_CORRECCION_VELOCIDAD", float, "Multiplicador para ajustar velocidad bajo CBF"),
        ],
        "RNG": [
            ("GCL_SEED_ENTORNO",            str,   "Semilla inicial del generador congruencial para entorno"),
            ("GCL_MULTIPLIER_A",            str,   "Multiplicador 'a' en la fórmula LCG (entero grande)"),
            ("GCL_INCREMENT_C",             str,   "Incremento 'c' en la fórmula LCG (entero grande)"),
            ("GCL_MODULUS_M",               str,   "Módulo 'm' en la fórmula LCG (entero grande)"),
            ("MIDDLE_SQUARE_SEED_DRONES",   str,   "Semilla de N dígitos para Middle-Square (drones)"),
            ("N_DIGITS_MIDDLE_SQUARE",      int,   "Número de dígitos usados en Middle-Square"),
            ("GCL_SEED_OBSTACULOS_DYN",     str,   "Semilla LCG para obst dinámicos"),
            ("GCL_MULTIPLIER_A_OBS",        str,   "Multiplicador 'a' LCG obstáculos dinámicos"),
            ("GCL_INCREMENT_C_OBS",         str,   "Incremento 'c' LCG obstáculos dinámicos"),
            ("GCL_MODULUS_M_OBS",           str,   "Módulo 'm' LCG obstáculos dinámicos"),
            ("RNG_TEST_NUM_SAMPLES",        int,   "Cantidad de muestras para las pruebas RNG"),
            ("RNG_TEST_NUM_BINS_CHI2",      int,   "Número de bins en la prueba Chi²"),
        ]
    }

class ConfigLauncher(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Configuración Simulación de Drones")
        self.resize(600, 700)
        self.defaults = cargar_config_y_descripciones()
        self.campos = {}
        self._build_ui()

    def _build_ui(self):
        main_layout = QVBoxLayout(self)
        scroll = QScrollArea()
        container = QWidget()
        vbox = QVBoxLayout(container)

        # Para cada categoría, crear un GroupBox
        for cat, items in self.defaults.items():
            gb = QGroupBox(cat)
            form = QFormLayout(gb)
            for key, typ, desc in items:
                # obtener valor por defecto
                val = getattr(importlib.import_module("drone_simulation.config"), key)
                # escoger widget según tipo
                if typ is int:
                    w = QSpinBox()
                    w.setMaximum(1_000_000)
                    w.setValue(val or 0)
                elif typ is float:
                    w = QDoubleSpinBox()
                    w.setDecimals(4)
                    w.setSingleStep(0.1)
                    w.setMaximum(1e6)
                    w.setValue(val or 0.0)
                else:  # str / semilla
                    w = QLineEdit("" if val is None else str(val))
                w.setToolTip(desc)
                form.addRow(f"{key}:", w)
                lbl = QLabel(desc)
                lbl.setStyleSheet("color: gray; font-size: 9pt")
                form.addRow("", lbl)
                self.campos[key] = w
            vbox.addWidget(gb)

        container.setLayout(vbox)
        scroll.setWidget(container)
        scroll.setWidgetResizable(True)
        main_layout.addWidget(scroll)

        btn = QPushButton("Iniciar Simulación")
        btn.clicked.connect(self._save_and_launch)
        btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        main_layout.addWidget(btn)

    def _save_and_launch(self):
        out = {}
        for key, w in self.campos.items():
            if isinstance(w, (QSpinBox,)):
                out[key] = w.value()
            elif isinstance(w, QDoubleSpinBox):
                out[key] = w.value()
            else:
                txt = w.text().strip()
                out[key] = None if txt.lower() in ("", "none") else txt
        # guardar
        with open("config_runtime.json", "w") as f:
            json.dump(out, f, indent=2)
        # lanzar
        subprocess.Popen([sys.executable, "main.py", "--use-runtime-config"])
        self.close()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = ConfigLauncher()
    win.show()
    sys.exit(app.exec_())
