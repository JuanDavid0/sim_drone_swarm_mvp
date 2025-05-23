# reporter.py
import os, subprocess, sys
from datetime import datetime
import matplotlib.pyplot as plt
from .rng_validator import perform_rng_quality_tests_from_scratch
from .rng import LCG, MiddleSquareRNG

class Reporter:
    def __init__(self, config, rngs):
        self.config = config
        self.rng_entorno, self.rng_drones, self.rng_obst = rngs

    def export_pdf(self):
        os.makedirs("rng_reports", exist_ok=True)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        path = os.path.join("rng_reports", f"rng_report_{ts}.pdf")
        # Repetir pruebas
        results = {}
        # Entorno
        results['Entorno'] = perform_rng_quality_tests_from_scratch(
            LCG(seed=self.rng_entorno.initial_seed,
                multiplier=self.config.GCL_MULTIPLIER_A,
                increment=self.config.GCL_INCREMENT_C,
                modulus=self.config.GCL_MODULUS_M),
            self.config.RNG_TEST_NUM_SAMPLES)
        # Drones
        results['Drones'] = perform_rng_quality_tests_from_scratch(
            MiddleSquareRNG(seed=self.rng_drones.initial_seed,
                            num_digits=self.config.N_DIGITS_MIDDLE_SQUARE),
            self.config.RNG_TEST_NUM_SAMPLES)
        # Obstáculos
        results['Obstaculos'] = perform_rng_quality_tests_from_scratch(
            LCG(seed=self.rng_obst.initial_seed,
                multiplier=self.config.GCL_MULTIPLIER_A_OBS,
                increment=self.config.GCL_INCREMENT_C_OBS,
                modulus=self.config.GCL_MODULUS_M_OBS),
            self.config.RNG_TEST_NUM_SAMPLES)
        # Generar PDF
        fig, axes = plt.subplots(3,1, figsize=(8,10))
        # -- Chi2 Entorno --
        chi2 = results['Entorno']['chi_squared_uniformity']
        axes[0].bar(range(len(chi2['observed_counts'])), chi2['observed_counts'])
        axes[0].axhline(chi2['expected_per_bin'], linestyle='--')
        axes[0].set_title('Chi² - Entorno')
        # -- K-S Drones --
        ks = results['Drones']['kolmogorov_smirnov_uniformity']
        samples = sorted([self.rng_drones.next_float() for _ in range(1000)])
        ecdf = [i/len(samples) for i in range(1,len(samples)+1)]
        axes[1].plot(samples, ecdf)
        axes[1].plot(samples, samples, linestyle='--')
        axes[1].set_title('K-S - Drones')
        # -- Autocorrelación Obs --
        ac = results['Obstaculos']['autocorrelation_lag1_numpy']['value']
        axes[2].text(0.1,0.5,f'Autocorrelación lag-1: {ac:.4f}')
        axes[2].axis('off')
        plt.tight_layout()
        fig.savefig(path)
        plt.close(fig)
        # Lanzar dashboard
        dash = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'rng_dashboard.py'))
        subprocess.Popen([sys.executable, dash, path])
