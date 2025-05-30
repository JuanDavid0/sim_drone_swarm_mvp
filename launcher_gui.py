import sys, os, json, importlib, subprocess
from PyQt5.QtWidgets import (
    QApplication, QWidget, QLabel, QVBoxLayout, QHBoxLayout,
    QScrollArea, QGroupBox, QFormLayout, QSpinBox, QDoubleSpinBox,
    QLineEdit, QPushButton, QSizePolicy, QCheckBox
)
from PyQt5.QtCore import Qt

# Carga de valores por defecto y descripciones SOLO para las variables editables
def cargar_config_y_descripciones_editables():
    """
    Define qué variables aparecerán en la GUI y sus tipos.
    Las variables no listadas aquí usarán su valor de config.py
    y no serán visibles ni editables en el launcher.
    """
    return {
        "Simulación General": [
            ("NUM_DRONES_INICIAL", int, "Cantidad de drones al iniciar"),
            ("FPS", float, "Velocidad de refresco visual (frames por segundo)"),
        ],
        "Comportamiento del Enjambre": [
            ("K_COHESION", float, "Fuerza de atracción grupal"),
            ("K_SEPARATION", float, "Fuerza de repulsión entre drones"),
            ("K_ALIGNMENT", float, "Fuerza de alineamiento de velocidad"),
            ("K_FRONTIER_ATTRACTION", float, "Fuerza de atracción a zonas no cubiertas"),
            ("SENSOR_RANGE_DRONE", float, "Rango de detección de otros drones"),
            ("MAX_VELOCIDAD", float, "Velocidad máxima permitida"),
            ("MAX_FUERZA", float, "Fuerza máxima aplicable"),
        ],
        "Obstáculos": [
            ("NUM_OBSTACULOS", int, "Número inicial de obstáculos"),
            ("K_OBSTACLE_REPULSION", float, "Fuerza de repulsión a obstáculos"),
            ("OBSTACULOS_DINAMICOS_PORCENTAJE", float, "Fracción de obstáculos dinámicos (0 a 1)"),
            ("GENERAR_NUEVOS_OBSTACULOS_INTERVALO", float, "Intervalo de generación de nuevos obs. (s)"),
        ],
        "Control Barrier Functions (CBF)": [
            ("CBF_ACTIVADO", bool, "Activar/Desactivar Control Barrier Functions"),
            ("CBF_D_MIN_DRON_DRON", float, "Distancia mínima de seguridad (CBF)"),
            ("CBF_FACTOR_CORRECCION_VELOCIDAD", float, "Agresividad de corrección CBF"),
        ],
         "Cobertura": [
            ("TAMANO_CELDA_COBERTURA", int, "Tamaño de celda para mapa de cobertura (px)"),
        ],
        "Configuración RNG (Semillas)": [ # Nueva categoría para las semillas
            ("GCL_SEED_ENTORNO", str, "Semilla LCG Entorno (vacío o 'None' para aleatorio)"),
            ("MIDDLE_SQUARE_SEED_DRONES", str, "Semilla Middle Sq. Drones (vacío o 'None' para aleatorio)"),
            ("GCL_SEED_OBSTACULOS_DYN", str, "Semilla LCG Obst. Dinámicos (vacío o 'None' para aleatorio)"),
        ]
    }

class ConfigLauncher(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Configuración Simulación de Drones")
        self.resize(700, 700)
        self.config_editable = cargar_config_y_descripciones_editables()
        self.campos = {}
        # Cargamos el módulo config base para obtener los valores por defecto
        self.cfg_base = importlib.import_module("drone_simulation.config")
        self._build_ui()
        self.cargar_desde_json("config_runtime.json") # Intentar cargar la última config

    def _build_ui(self):
        main_layout = QVBoxLayout(self)
        scroll = QScrollArea()
        container = QWidget()
        vbox = QVBoxLayout(container)

        # Crear GroupBox solo para las categorías con variables editables
        for cat, items in self.config_editable.items():
            gb = QGroupBox(cat)
            form = QFormLayout(gb)
            for key, typ, desc in items:
                try:
                    val = getattr(self.cfg_base, key)
                except AttributeError:
                    print(f"Advertencia: La variable {key} no se encontró en config.py, usando valor genérico.")
                    val = 0 if typ is int else 0.0 if typ is float else False if typ is bool else ""

                if typ is int:
                    w = QSpinBox()
                    w.setMaximum(1_000_000)
                    w.setValue(int(val) if val is not None else 0)
                elif typ is float:
                    w = QDoubleSpinBox()
                    w.setDecimals(2) 
                    w.setSingleStep(0.01)
                    w.setMaximum(1_000_000.0)
                    w.setMinimum(-1_000_000.0)
                    w.setValue(float(val) if val is not None else 0.0)
                elif typ is bool:
                    w = QCheckBox()
                    w.setChecked(bool(val) if val is not None else False)
                elif typ is str: # Para las semillas
                    w = QLineEdit()
                    w.setText(str(val) if val is not None else "")
                    if "SEED" in key.upper():
                        w.setPlaceholderText("Número o vacío/None")
                        
                w.setToolTip(desc)
                # Asegurarse que 'w' se haya asignado antes de usarlo
                if w: # Solo añadir si el widget fue creado
                    form.addRow(f"{key}:", w)
                    lbl = QLabel(desc)
                    lbl.setStyleSheet("color: gray; font-size: 9pt")
                    if not isinstance(w, QCheckBox):
                        form.addRow("", lbl)
                    self.campos[key] = w
                else:
                    print(f"Error: No se pudo crear el widget para la clave '{key}' con tipo '{typ}'.")

            vbox.addWidget(gb)

        container.setLayout(vbox)
        scroll.setWidget(container)
        scroll.setWidgetResizable(True)
        main_layout.addWidget(scroll)

        btn_layout = QHBoxLayout()
        btn_reset = QPushButton("Cargar Defectos")
        btn_reset.clicked.connect(self.cargar_defectos)
        btn = QPushButton("Iniciar Simulación")
        btn.clicked.connect(self._save_and_launch)
        btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        
        btn_layout.addWidget(btn_reset)
        btn_layout.addWidget(btn)
        main_layout.addLayout(btn_layout)


    def cargar_defectos(self):
        """ Recarga los valores por defecto desde config.py """
        for key, w in self.campos.items():
             try:
                val = getattr(self.cfg_base, key)
                if isinstance(w, QSpinBox): w.setValue(int(val))
                elif isinstance(w, QDoubleSpinBox): w.setValue(float(val))
                elif isinstance(w, QCheckBox): w.setChecked(bool(val))
                else: w.setText(str(val))
             except AttributeError:
                 print(f"Error al cargar defecto para {key}")


    def cargar_desde_json(self, filepath="config_runtime.json"):
        """ Carga valores desde el JSON si existe. """
        if os.path.exists(filepath):
            try:
                with open(filepath, "r") as f:
                    data = json.load(f)
                
                for key, w in self.campos.items():
                    if key in data:
                        val = data[key]
                        if isinstance(w, QSpinBox): w.setValue(int(val))
                        elif isinstance(w, QDoubleSpinBox): w.setValue(float(val))
                        elif isinstance(w, QCheckBox): w.setChecked(bool(val))
                        else: w.setText(str(val))
            except Exception as e:
                print(f"Error al cargar {filepath}: {e}")
        else:
            self.cargar_defectos() # Si no hay JSON, carga los de config.py

    def _save_and_launch(self):
        out = {}
        for key, w in self.campos.items():
            if isinstance(w, (QSpinBox,)):
                out[key] = w.value()
            elif isinstance(w, QDoubleSpinBox):
                out[key] = w.value()
            elif isinstance(w, QCheckBox):
                out[key] = bool(w.isChecked()) # Guardar como booleano o int 0/1
            else:
                txt = w.text().strip()
                out[key] = None if txt.lower() in ("", "none") else txt
        
        # Cargar TODAS las variables de config.py
        all_config = {k: getattr(self.cfg_base, k) 
                      for k in dir(self.cfg_base) 
                      if not k.startswith('_') and isinstance(getattr(self.cfg_base, k), (int, float, bool, str, tuple))}

        # Sobreescribir con las variables editadas en la GUI
        all_config.update(out)

        # Guardar el JSON completo (variables fijas + editadas)
        with open("config_runtime.json", "w") as f:
            json.dump(all_config, f, indent=2)

        # Lanzar la simulación
        subprocess.Popen([sys.executable, "main.py", "--use-runtime-config"])
        self.close()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = ConfigLauncher()
    win.show()
    sys.exit(app.exec_())