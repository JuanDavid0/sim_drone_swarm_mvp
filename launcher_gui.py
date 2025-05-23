import sys
import json
import subprocess
from PyQt5.QtWidgets import (
    QApplication, QWidget, QLabel, QVBoxLayout, QHBoxLayout,
    QSlider, QPushButton, QLineEdit, QGridLayout, QSpinBox, QDoubleSpinBox
)
from PyQt5.QtCore import Qt
import importlib

# Cargar el módulo config.py para obtener los valores por defecto
def cargar_configuracion_default():
    config_mod = importlib.import_module("drone_simulation.config")
    return {
        "NUM_DRONES_INICIAL": config_mod.NUM_DRONES_INICIAL,
        "MASA_DRONE": config_mod.MASA_DRONE,
        "MAX_VELOCIDAD": config_mod.MAX_VELOCIDAD,
        "K_COHESION": config_mod.K_COHESION,
        "K_SEPARATION": config_mod.K_SEPARATION,
        "K_ALIGNMENT": config_mod.K_ALIGNMENT,
        "K_FRONTIER_ATTRACTION": config_mod.K_FRONTIER_ATTRACTION,
        "K_OBSTACLE_REPULSION": config_mod.K_OBSTACLE_REPULSION,
        "K_BORDE_REPULSION": config_mod.K_BORDE_REPULSION,
        "SENSOR_RANGE_DRONE": config_mod.SENSOR_RANGE_DRONE,
        "RADIO_BUSQUEDA_FRONTERA_DRONE": config_mod.RADIO_BUSQUEDA_FRONTERA_DRONE,
        "GCL_SEED_ENTORNO": config_mod.GCL_SEED_ENTORNO,
        "MIDDLE_SQUARE_SEED_DRONES": config_mod.MIDDLE_SQUARE_SEED_DRONES,
        "N_DIGITS_MIDDLE_SQUARE": config_mod.N_DIGITS_MIDDLE_SQUARE,
        "PROBABILIDAD_FALLO_POR_COLISION_OBSTACULO": config_mod.PROBABILIDAD_FALLO_POR_COLISION_OBSTACULO,
        "PROBABILIDAD_FALLO_POR_COLISION_DRON": config_mod.PROBABILIDAD_FALLO_POR_COLISION_DRON
    }

class ConfigLauncher(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Simulación de Drones - Configuración Inicial")
        self.config = cargar_configuracion_default()
        self.campos = {}
        self.init_ui()

    def init_ui(self):
        layout = QVBoxLayout()

        grid = QGridLayout()
        row = 0
        for clave, valor in self.config.items():
            grid.addWidget(QLabel(clave), row, 0)
            if isinstance(valor, int):
                campo = QSpinBox()
                campo.setMaximum(10000)
                campo.setValue(valor)
            elif isinstance(valor, float):
                campo = QDoubleSpinBox()
                campo.setDecimals(3)
                campo.setSingleStep(0.1)
                campo.setMaximum(10000.0)
                campo.setValue(valor)
            else:
                campo = QLineEdit(str(valor) if valor is not None else "")
            self.campos[clave] = campo
            grid.addWidget(campo, row, 1)
            row += 1

        layout.addLayout(grid)

        boton_iniciar = QPushButton("Iniciar Simulación")
        boton_iniciar.clicked.connect(self.iniciar_simulacion)
        layout.addWidget(boton_iniciar)

        self.setLayout(layout)

    def iniciar_simulacion(self):
        parametros_modificados = {}
        for clave, campo in self.campos.items():
            if isinstance(campo, QLineEdit):
                texto = campo.text().strip()
                if texto.lower() == "none" or texto == "":
                    parametros_modificados[clave] = None
                elif texto.isdigit():
                    parametros_modificados[clave] = int(texto)
                else:
                    try:
                        parametros_modificados[clave] = float(texto)
                    except ValueError:
                        parametros_modificados[clave] = texto
            elif isinstance(campo, QSpinBox):
                parametros_modificados[clave] = campo.value()
            elif isinstance(campo, QDoubleSpinBox):
                parametros_modificados[clave] = campo.value()

        with open("config_runtime.json", "w") as f:
            json.dump(parametros_modificados, f, indent=4)

        subprocess.Popen([sys.executable, "main.py", "--use-runtime-config"])
        self.close()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    launcher = ConfigLauncher()
    launcher.show()
    sys.exit(app.exec_())
