# rng_dashboard.py
import sys
import os
import json
import numpy as np
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QLabel,
    QPushButton, QComboBox, QHBoxLayout
)
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from drone_simulation.rng import LCG, MiddleSquareRNG
from drone_simulation.rng_validator import perform_rng_quality_tests_from_scratch

class RNGDashboard(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Dashboard - Pruebas RNG")
        self.setGeometry(100, 100, 1000, 700)
        self.config_data = self.cargar_config_runtime()
        self.generadores = {}
        self.setup_generadores()
        self.setup_ui()

    def cargar_config_runtime(self):
        ruta = "config_runtime.json"
        if os.path.exists(ruta):
            with open(ruta, "r") as f:
                raw = json.load(f)
                config = {}
                for k, v in raw.items():
                    if v in ("", None):
                        config[k] = None
                    else:
                        try:
                            config[k] = int(v)
                        except ValueError:
                            try:
                                config[k] = float(v)
                            except ValueError:
                                config[k] = v
                return config
        return {}

    def setup_generadores(self):
        # Cargar desde config, si existe. Si no, usar valores por defecto.
        conf = self.config_data
        self.generadores = {
            "LCG - Entorno": lambda: LCG(
                seed=conf.get("GCL_SEED_ENTORNO", 12345),
                multiplier=conf.get("GCL_MULTIPLIER_A", 1664525),
                increment=conf.get("GCL_INCREMENT_C", 1013904223),
                modulus=conf.get("GCL_MODULUS_M", 2**32)
            ),
            "Middle Square - Drones": lambda: MiddleSquareRNG(
                seed=conf.get("MIDDLE_SQUARE_SEED_DRONES", 1234),
                num_digits=conf.get("N_DIGITS_MIDDLE_SQUARE", 4)
            ),
            "LCG - Obstáculos": lambda: LCG(
                seed=conf.get("GCL_SEED_OBSTACULOS_DYN", 54321),
                multiplier=conf.get("GCL_MULTIPLIER_A_OBS", 1103515245),
                increment=conf.get("GCL_INCREMENT_C_OBS", 12345),
                modulus=conf.get("GCL_MODULUS_M_OBS", 2**31)
            )
        }

    def setup_ui(self):
        central_widget = QWidget()
        layout = QVBoxLayout()

        # Selección del generador
        controls = QHBoxLayout()
        self.combo = QComboBox()
        self.combo.addItems(self.generadores.keys())
        controls.addWidget(QLabel("Generador:"))
        controls.addWidget(self.combo)

        self.btn = QPushButton("Ejecutar Pruebas")
        self.btn.clicked.connect(self.ejecutar_pruebas)
        controls.addWidget(self.btn)
        layout.addLayout(controls)

        # Texto de resultados básicos
        self.result_label = QLabel()
        layout.addWidget(self.result_label)

        # Canvas de matplotlib
        self.figure = Figure()
        self.canvas = FigureCanvas(self.figure)
        layout.addWidget(self.canvas)

        central_widget.setLayout(layout)
        self.setCentralWidget(central_widget)

    def ejecutar_pruebas(self):
        self.figure.clear()
        gen_key = self.combo.currentText()
        rng = self.generadores[gen_key]()
        resultados = perform_rng_quality_tests_from_scratch(rng, 10000)

        ax1 = self.figure.add_subplot(211)
        chi2 = resultados["chi_squared_uniformity"]
        if "observed_counts" in chi2:
            ax1.bar(range(len(chi2["observed_counts"])), chi2["observed_counts"], label="Observado")
            ax1.axhline(chi2["expected_per_bin"], color='r', linestyle='--', label="Esperado")
            ax1.set_title("Histograma - Chi²")
            ax1.legend()

        ax2 = self.figure.add_subplot(212)
        ks = resultados["kolmogorov_smirnov_uniformity"]
        if "statistic_D" in ks:
            sorted_samples = np.sort([rng.next_float() for _ in range(1000)])
            ecdf_y = np.arange(1, len(sorted_samples)+1) / len(sorted_samples)
            ax2.plot(sorted_samples, ecdf_y, label="ECDF")
            ax2.plot(sorted_samples, sorted_samples, linestyle="--", label="CDF Teórica")
            ax2.set_title("K-S: ECDF vs CDF")
            ax2.legend()

        texto_resultado = f"""
        Tipo RNG: {resultados['rng_type']}
        Semilla Inicial: {resultados.get('initial_seed_for_test_sequence', 'N/A')}
        Muestras: {resultados['num_samples_tested']}
        Chi² Statistic: {chi2.get('statistic', 0):.4f}
        K-S D: {ks.get('statistic_D', 0):.4f}
        Autocorrelación (lag-1): {resultados['autocorrelation_lag1_numpy'].get('value', 0):.4f}
        """
        if ks.get("hypothesis_H0_rejected_at_alpha_0.05 (approx)", False):
            texto_resultado += "\n⚠️ Se rechaza H0 (K-S, α=0.05)"
        else:
            texto_resultado += "\n✅ H0 no rechazada (K-S, α=0.05)"

        self.result_label.setText(texto_resultado)
        self.canvas.draw()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = RNGDashboard()
    window.show()
    sys.exit(app.exec_())
