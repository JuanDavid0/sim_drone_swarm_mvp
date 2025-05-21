# drone_simulation/rng_validator.py
import numpy as np
from scipy import stats # Usamos scipy.stats para las pruebas, no para generar
# No necesitamos importar LCG o MiddleSquareRNG aquí, ya que las funciones
# recibirán una instancia del generador a probar.
from . import config # Para parámetros de prueba

def generate_samples_from_rng_instance(rng_instance, num_samples: int, sample_type: str = 'float') -> list:
    """Genera una lista de muestras desde una instancia de RNG dada."""
    samples = []
    if sample_type == 'float':
        for _ in range(num_samples):
            samples.append(rng_instance.next_float())
    elif sample_type == 'int_0_99': # Ejemplo para Chi-cuadrado con enteros
        for _ in range(num_samples):
            # Asumimos que next_int(0,99) existe y funciona para el RNG dado
            samples.append(rng_instance.next_int(0, 99)) 
    return samples

def run_chi_squared_test_uniform_floats(float_samples: list, num_bins: int) -> dict:
    """
    Prueba de Chi-cuadrado para uniformidad de flotantes en [0,1).
    """
    if not float_samples or len(float_samples) < num_bins * 5 : # Condición para validez de Chi2
        return {'error': 'No suficientes muestras o bins para Chi-cuadrado confiable.'}
    
    observed_counts, _ = np.histogram(float_samples, bins=num_bins, range=(0,1))
    expected_counts = [len(float_samples) / num_bins] * num_bins
    
    # Eliminar bins con expected_counts == 0 para evitar error en chisquare
    valid_indices = [i for i, e in enumerate(expected_counts) if e > 0]
    observed_counts_valid = observed_counts[valid_indices]
    expected_counts_valid = [expected_counts[i] for i in valid_indices]

    if not expected_counts_valid or len(observed_counts_valid) < 2: # Necesita al menos 2 categorías
         return {'error': 'No suficientes categorías válidas para Chi-cuadrado.'}
    
    if np.min(expected_counts_valid) < 1 : # Scipy advierte si E < 5, pero puede fallar con E < 1
        print(f"Advertencia Chi2: Frecuencias esperadas bajas ({np.min(expected_counts_valid)}), resultados pueden no ser precisos.")


    try:
        chi2_stat, p_val = stats.chisquare(f_obs=observed_counts_valid, f_exp=expected_counts_valid)
        return {'statistic': chi2_stat, 'p_value': p_val, 'bins': num_bins, 'degrees_freedom': num_bins - 1}
    except Exception as e:
        return {'error': f'Error en Chi-cuadrado: {str(e)}'}


def run_kolmogorov_smirnov_test_uniform_floats(float_samples: list) -> dict:
    """
    Prueba de Kolmogorov-Smirnov para uniformidad de flotantes en [0,1).
    """
    if not float_samples:
        return {'error': 'No hay muestras para la prueba K-S.'}
        
    sample_data_np = np.array(float_samples)
    try:
        # Compara contra una distribución uniforme continua entre 0 y 1
        ks_stat, p_val = stats.kstest(sample_data_np, 'uniform', args=(0,1)) 
        return {'statistic': ks_stat, 'p_value': p_val}
    except Exception as e:
        return {'error': f'Error en K-S: {str(e)}'}


def perform_rng_quality_tests(rng_instance_to_test, num_samples: int) -> dict:
    """
    Ejecuta un conjunto de pruebas de calidad en la instancia de RNG dada.
    """
    test_suite_results = {}
    
    # Generar una única muestra de flotantes para todas las pruebas que los usan
    float_samples = generate_samples_from_rng_instance(rng_instance_to_test, num_samples, 'float')
    
    # Guardar una copia de la semilla inicial antes de generar muestras para las pruebas
    # para que podamos reportar la semilla con la que se inició la generación de la muestra de prueba.
    # Sin embargo, rng_instance_to_test ya habrá avanzado si se usó para generar float_samples.
    # Sería mejor pasar una *nueva* instancia con la misma semilla inicial para cada prueba,
    # o una función que genere la instancia. Por ahora, usaremos la misma secuencia.
    
    test_suite_results['rng_type'] = type(rng_instance_to_test).__name__
    if hasattr(rng_instance_to_test, 'initial_seed'):
        test_suite_results['initial_seed_for_test_sequence'] = rng_instance_to_test.initial_seed
    test_suite_results['num_samples_tested'] = len(float_samples)

    # --- Prueba 1: Chi-cuadrado para uniformidad de flotantes ---
    test_suite_results['chi_squared_uniformity'] = run_chi_squared_test_uniform_floats(
        float_samples, 
        config.RNG_TEST_NUM_BINS_CHI2
    )

    # --- Prueba 2: Kolmogorov-Smirnov para uniformidad de flotantes ---
    test_suite_results['kolmogorov_smirnov_uniformity'] = run_kolmogorov_smirnov_test_uniform_floats(
        float_samples
    )
    
    # --- (Opcional) Prueba 3: Autocorrelación simple (lag-1) ---
    if len(float_samples) > 1:
        try:
            # Convertir a NumPy array para np.corrcoef si no lo es ya
            samples_np = np.array(float_samples)
            autocorr_lag1 = np.corrcoef(samples_np[:-1], samples_np[1:])[0, 1]
            test_suite_results['autocorrelation_lag1'] = {'value': autocorr_lag1}
        except Exception as e:
            test_suite_results['autocorrelation_lag1'] = {'error': f'Error en Autocorrelación: {str(e)}'}
    else:
        test_suite_results['autocorrelation_lag1'] = {'error': 'No suficientes muestras'}

    return test_suite_results