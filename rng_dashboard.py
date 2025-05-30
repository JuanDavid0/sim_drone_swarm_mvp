# rng_dashboard.py
import sys
import os
import json
import numpy as np
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QLabel,
    QPushButton, QComboBox, QHBoxLayout, QSizePolicy, QScrollArea,
    QTextEdit, QGroupBox
)
from PyQt5.QtCore import Qt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from drone_simulation.rng import LCG, MiddleSquareRNG
from drone_simulation.rng_validator import perform_rng_quality_tests_from_scratch

class RNGDashboard(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Dashboard - Pruebas de Calidad RNG")
        self.setGeometry(100, 100, 1000, 800) # Un poco más alto para el texto
        self.config_data = self.cargar_config_runtime()
        self.generadores = {}
        self.setup_generadores()
        self.setup_ui()

    def cargar_config_runtime(self):
        ruta = "config_runtime.json"
        config_cargada = {}
        if os.path.exists(ruta):
            with open(ruta, "r") as f:
                try:
                    raw_data = json.load(f)
                    if not isinstance(raw_data, dict):
                        print(f"Advertencia: {ruta} no contiene un diccionario JSON válido.")
                        return {}
                except json.JSONDecodeError:
                    print(f"Error: No se pudo decodificar {ruta}. Usando configuración por defecto para el dashboard.")
                    return {}

                for k, v in raw_data.items():
                    if v in ("", None):
                        config_cargada[k] = None
                    elif isinstance(v, (list, tuple)):
                        config_cargada[k] = v
                    else:
                        try:
                            config_cargada[k] = int(v)
                        except (ValueError, TypeError):
                            try:
                                config_cargada[k] = float(v)
                            except (ValueError, TypeError):
                                config_cargada[k] = v
                return config_cargada
        return {}

    def setup_generadores(self):
        conf = self.config_data
        # Usar valores por defecto si no están en la configuración o son inválidos
        try:
            gcl_seed_entorno = int(conf.get("GCL_SEED_ENTORNO"))
            gcl_multiplier_a = int(conf.get("GCL_MULTIPLIER_A"))
            gcl_increment_c = int(conf.get("GCL_INCREMENT_C"))
            gcl_modulus_m = int(conf.get("GCL_MODULUS_M"))

            ms_seed_drones = int(conf.get("MIDDLE_SQUARE_SEED_DRONES"))
            ms_n_digits = int(conf.get("N_DIGITS_MIDDLE_SQUARE"))

            gcl_seed_obst = int(conf.get("GCL_SEED_OBSTACULOS_DYN"))
            gcl_mult_a_obs = int(conf.get("GCL_MULTIPLIER_A_OBS"))
            gcl_inc_c_obs = int(conf.get("GCL_INCREMENT_C_OBS"))
            gcl_mod_m_obs = int(conf.get("GCL_MODULUS_M_OBS"))

        except (ValueError, TypeError) as e:
            print(f"Error al convertir parámetros de RNG desde config: {e}. Usando defaults estrictos.")
            gcl_seed_entorno = 12345
            gcl_multiplier_a = 1664525
            gcl_increment_c = 1013904223
            gcl_modulus_m = 2**32
            ms_seed_drones = 1234
            ms_n_digits = 4
            gcl_seed_obst = 54321
            gcl_mult_a_obs = 1103515245
            gcl_inc_c_obs = 12345
            gcl_mod_m_obs = 2**31


        self.generadores = {
            "LCG - Entorno": lambda: LCG(
                seed=gcl_seed_entorno,
                multiplier=gcl_multiplier_a,
                increment=gcl_increment_c,
                modulus=gcl_modulus_m
            ),
            "Middle Square - Drones": lambda: MiddleSquareRNG(
                seed=ms_seed_drones,
                num_digits=ms_n_digits
            ),
            "LCG - Obstáculos": lambda: LCG(
                seed=gcl_seed_obst,
                multiplier=gcl_mult_a_obs,
                increment=gcl_inc_c_obs,
                modulus=gcl_mod_m_obs
            )
        }

    def setup_ui(self):
        central_widget = QWidget()
        main_layout = QVBoxLayout() # Layout principal

        # Sección de Controles
        controls_groupbox = QGroupBox("Controles de Prueba")
        controls_layout = QHBoxLayout()
        self.combo = QComboBox()
        self.combo.addItems(self.generadores.keys())
        self.combo.setToolTip("Selecciona el generador de números pseudoaleatorios a probar.")
        controls_layout.addWidget(QLabel("Generador:"))
        controls_layout.addWidget(self.combo)

        self.btn_run_tests = QPushButton("Ejecutar Pruebas")
        self.btn_run_tests.clicked.connect(self.ejecutar_pruebas)
        self.btn_run_tests.setStyleSheet("QPushButton { background-color: #4CAF50; color: white; padding: 5px; }")
        controls_layout.addWidget(self.btn_run_tests)
        controls_groupbox.setLayout(controls_layout)
        main_layout.addWidget(controls_groupbox)

        # Sección de Resultados (Texto y Gráficos)
        results_groupbox = QGroupBox("Resultados de las Pruebas")
        results_layout = QVBoxLayout()

        # Área de texto para resultados numéricos (más grande y con scroll)
        self.result_text_area = QTextEdit()
        self.result_text_area.setReadOnly(True)
        self.result_text_area.setPlaceholderText("Los resultados de las pruebas se mostrarán aquí.")
        self.result_text_area.setFixedHeight(150) # Altura fija para el texto
        results_layout.addWidget(self.result_text_area)

        # Canvas de matplotlib para los gráficos
        self.figure = Figure(figsize=(5, 6)) # Ajustar tamaño si es necesario
        self.canvas = FigureCanvas(self.figure)
        results_layout.addWidget(self.canvas) # Canvas ocupa el espacio restante

        results_groupbox.setLayout(results_layout)
        main_layout.addWidget(results_groupbox) # Añadir el groupbox de resultados al layout principal

        central_widget.setLayout(main_layout)
        self.setCentralWidget(central_widget)

    def ejecutar_pruebas(self):
        self.figure.clear() # Limpiar figuras anteriores
        gen_key = self.combo.currentText()
        
        if not gen_key:
            self.result_text_area.setText("Por favor, selecciona un generador.")
            return
        
        rng_instance = self.generadores[gen_key]() # Crear instancia del RNG seleccionado
        
        # Obtener parámetros para las pruebas desde la configuración cargada
        num_samples_test = self.config_data.get("RNG_TEST_NUM_SAMPLES", 10000)
        try: # Asegurarse que sean enteros
            num_samples_test = int(num_samples_test)
        except (ValueError, TypeError):
            num_samples_test = 10000
            print("Advertencia: RNG_TEST_NUM_SAMPLES no es un entero válido en config, usando 10000.")

        num_bins_chi2 = self.config_data.get("RNG_TEST_NUM_BINS_CHI2", 10)
        try:
            num_bins_chi2 = int(num_bins_chi2)
        except (ValueError, TypeError):
            num_bins_chi2 = 10
            print("Advertencia: RNG_TEST_NUM_BINS_CHI2 no es un entero válido en config, usando 10.")

        resultados = perform_rng_quality_tests_from_scratch(rng_instance, num_samples_test)

        # Formatear los resultados para el QTextEdit
        texto_resultado_html = "<h3>Resultados Detallados:</h3>" # Usar HTML para mejor formato
        texto_resultado_html += f"<p><b>Tipo RNG:</b> {resultados['rng_type']}<br>"
        texto_resultado_html += f"<b>Semilla Inicial (para esta secuencia de prueba):</b> {resultados.get('initial_seed_for_test_sequence', 'N/A')}<br>"
        texto_resultado_html += f"<b>Número de Muestras Probadas:</b> {resultados['num_samples_tested']}</p>"

        # Chi-Cuadrado
        chi2_res = resultados["chi_squared_uniformity"]
        texto_resultado_html += "<h4>Prueba Chi-Cuadrado (Uniformidad en [0,1))</h4>"
        if "error" in chi2_res:
            texto_resultado_html += f"<p><font color='red'>Error: {chi2_res['error']}</font></p>"
        else:
            texto_resultado_html += f"<p>Estadístico Chi² (calculado): {chi2_res.get('statistic', float('nan')):.4f}<br>"
            texto_resultado_html += f"Grados de Libertad: {chi2_res.get('degrees_freedom', 'N/A')}<br>"
            if chi2_res.get('warning'):
                 texto_resultado_html += f"<p><font color='orange'>Advertencia (Chi²): {chi2_res.get('warning')}</font></p>"


        # Kolmogorov-Smirnov
        ks_res = resultados["kolmogorov_smirnov_uniformity"]
        texto_resultado_html += "<h4>Prueba Kolmogorov-Smirnov (Uniformidad en [0,1))</h4>"
        if "error" in ks_res:
            texto_resultado_html += f"<p><font color='red'>Error: {ks_res['error']}</font></p>"
        else:
            texto_resultado_html += f"<p>Estadístico K-S (D) (calculado): {ks_res.get('statistic_D', float('nan')):.4f}<br>"
            # Interpretación aproximada de K-S si está disponible y N es grande
            h0_rejected_ks = ks_res.get('hypothesis_H0_rejected_at_alpha_0.05 (approx)', None)
            if h0_rejected_ks is not None:
                conclusion_ks = "⚠️ Se rechaza H0 (K-S, α≈0.05, N grande)" if h0_rejected_ks else "✅ H0 no rechazada (K-S, α≈0.05, N grande)"
                texto_resultado_html += f"Conclusión Aproximada (N grande): {conclusion_ks}<br>"
                texto_resultado_html += f"<font color='gray'><i>(Nota: Esta conclusión se basa en un valor crítico aproximado para N > ~35. Para N pequeñas o p-values exactos.)</i></font></p>"
            else:
                texto_resultado_html += f"<p></p>"

        # Autocorrelación
        autocorr_res = resultados.get("autocorrelation_lag1_numpy", {})
        texto_resultado_html += "<h4>Autocorrelación (Lag-1, con NumPy)</h4>"
        if "error" in autocorr_res:
            texto_resultado_html += f"<p><font color='red'>Error: {autocorr_res['error']}</font></p>"
        else:
            texto_resultado_html += f"<p>Valor: {autocorr_res.get('value', float('nan')):.4f}</p>"
        
        self.result_text_area.setHtml(texto_resultado_html)

        # --- GRÁFICOS ---
        # Gráfico 1: Histograma Chi-Cuadrado
        ax1 = self.figure.add_subplot(211) # Dos filas, una columna, primer gráfico
        if "observed_counts" in chi2_res and chi2_res.get("expected_per_bin") is not None:
            num_bins = len(chi2_res["observed_counts"])
            ax1.bar(range(num_bins), chi2_res["observed_counts"], label="Frecuencia Observada", color='skyblue', edgecolor='black')
            ax1.axhline(chi2_res["expected_per_bin"], color='r', linestyle='--', label=f"Frec. Esperada ({chi2_res['expected_per_bin']:.2f})")
            ax1.set_title(f"Histograma - Prueba Chi² (Bins: {num_bins})")
            ax1.set_xlabel("Intervalo (Bin)")
            ax1.set_ylabel("Frecuencia")
            ax1.legend()
        else:
            ax1.text(0.5, 0.5, "Datos para Chi² no disponibles o incompletos.", ha='center', va='center')
            ax1.set_title("Histograma - Prueba Chi²")

        # Gráfico 2: K-S ECDF vs CDF Teórica
        ax2 = self.figure.add_subplot(212) # Dos filas, una columna, segundo gráfico
        if "statistic_D" in ks_res and not np.isnan(ks_res['statistic_D']):
            # Volver a generar muestras para el gráfico ECDF (o pasarlas desde rng_validator)
            temp_rng_for_plot = self.generadores[gen_key]() # Nueva instancia
            # Usar un número menor de muestras para el gráfico para que no sea muy lento
            samples_for_ecdf = np.sort([temp_rng_for_plot.next_float() for _ in range(min(1000, num_samples_test))])
            
            if len(samples_for_ecdf) > 0:
                ecdf_y = np.arange(1, len(samples_for_ecdf) + 1) / len(samples_for_ecdf)
                ax2.plot(samples_for_ecdf, ecdf_y, label="ECDF (Muestra)", color='dodgerblue', marker='.', linestyle='none', markersize=4)
                ax2.plot([0, 1], [0, 1], linestyle="--", label="CDF Teórica U(0,1)", color='salmon') # Línea y=x para U(0,1)
                ax2.set_title(f"K-S: ECDF vs CDF Teórica (D={ks_res['statistic_D']:.4f})")
                ax2.set_xlabel("Valor de la Muestra")
                ax2.set_ylabel("Probabilidad Acumulada")
                ax2.legend()
                ax2.grid(True, linestyle=':', alpha=0.7)
            else:
                ax2.text(0.5, 0.5, "No hay muestras para graficar ECDF.", ha='center', va='center')
                ax2.set_title("K-S: ECDF vs CDF Teórica")
        else:
            ax2.text(0.5, 0.5, "Datos para K-S no disponibles o incompletos.", ha='center', va='center')
            ax2.set_title("K-S: ECDF vs CDF Teórica")

        self.figure.tight_layout(pad=3.0) # Añade espacio entre subplots y título general
        self.canvas.draw()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = RNGDashboard()
    window.show()
    sys.exit(app.exec_())