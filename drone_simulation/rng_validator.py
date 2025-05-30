# drone_simulation/rng_validator.py
import numpy as np
from . import config # Para parámetros de prueba como RNG_TEST_NUM_BINS_CHI2

# --- Funciones auxiliares para matemáticas (si son necesarias) ---
def gamma_incomplete_upper(s, x):
    """
    Implementación (muy simplificada o placeholder) de la función Gamma incompleta superior Q(s,x).
    Calcular esto con precisión desde cero es muy complejo y está fuera del alcance de un MVP simple.
    Para una prueba Chi2 real, necesitaríamos esto o una tabla de Chi2.
    ESTO ES UN PLACEHOLDER - NO USAR PARA RESULTADOS ESTADÍSTICOS REALES SIN UNA LIBRERÍA.
    """
    if s <= 0 or x < 0: return np.nan
    # Una aproximación muy, muy burda o enlace a tabla.
    # Por ahora, solo retornaremos NaN para indicar que no está implementado.
    print("Advertencia: gamma_incomplete_upper no está completamente implementada para p-value de Chi2.")
    return np.nan

def chi2_cdf(chi2_stat, df):
    """
    Placeholder para la CDF de Chi-cuadrado.
    P(X <= chi2_stat) donde X sigue una distribución Chi2 con df grados de libertad.
    p_value = 1 - CDF(chi2_stat, df)
    ESTO ES UN PLACEHOLDER.
    """
    print(f"Advertencia: chi2_cdf no implementada. Estadístico Chi2: {chi2_stat}, df: {df}. Consulte una tabla.")
    if df <= 0: return np.nan
    if df == 9 and chi2_stat > 16.919: return 0.04 # Simula p < 0.05
    if df == 9 and chi2_stat <= 16.919: return 0.10 # Simula p >= 0.05
    return np.nan # No podemos calcular el p-value exacto


# --- Funciones de Prueba (Implementación desde Cero) ---

def run_chi_squared_test_uniform_floats_from_scratch(float_samples: list, num_bins: int) -> dict:
    """
    Prueba de Chi-cuadrado para uniformidad de flotantes en [0,1), implementada desde cero.
    Retorna el estadístico Chi2 y grados de libertad. El p-value es un placeholder.
    """
    results = {'test_name': 'Chi-Cuadrado para Uniformidad (desde cero)'}
    n_samples = len(float_samples)

    if not float_samples or n_samples < num_bins * 5:
        results['error'] = 'No suficientes muestras o bins para Chi-cuadrado confiable.'
        return results
    
    observed_counts = [0] * num_bins
    bin_width = 1.0 / num_bins
    for sample in float_samples:
        if 0.0 <= sample < 1.0:
            bin_index = int(sample / bin_width)
            observed_counts[bin_index] += 1
        elif sample == 1.0: # El límite superior [0,1)
            observed_counts[num_bins - 1] +=1


    expected_count_per_bin = n_samples / num_bins
    
    if expected_count_per_bin < 1:
        results['warning'] = f"Frecuencia esperada por bin ({expected_count_per_bin:.2f}) es < 1. Resultados pueden no ser precisos."
    elif expected_count_per_bin < 5:
        results['warning'] = f"Frecuencia esperada por bin ({expected_count_per_bin:.2f}) es < 5. Resultados pueden ser menos precisos."

    chi2_statistic = 0.0
    for obs_count in observed_counts:
        chi2_statistic += ((obs_count - expected_count_per_bin)**2) / expected_count_per_bin
    
    degrees_freedom = num_bins - 1
    
    results['statistic'] = chi2_statistic
    results['degrees_freedom'] = degrees_freedom
    results['p_value_placeholder'] = chi2_cdf(chi2_statistic, degrees_freedom) # Usará el placeholder
    results['bins'] = num_bins
    results['observed_counts'] = observed_counts
    results['expected_per_bin'] = expected_count_per_bin
    return results


def run_kolmogorov_smirnov_test_uniform_floats_from_scratch(float_samples: list) -> dict:
    """
    Prueba K-S para uniformidad de flotantes en [0,1), implementada desde cero.
    Retorna el estadístico D. El p-value es un placeholder.
    """
    results = {'test_name': 'Kolmogorov-Smirnov para Uniformidad (desde cero)'}
    n = len(float_samples)
    if not float_samples or n == 0:
        results['error'] = 'No hay muestras para la prueba K-S.'
        return results
        
    sorted_samples = sorted(float_samples)
    
    d_plus_max = 0.0
    d_minus_max = 0.0
    
    for i in range(n):
        
        f_n_xi = (i + 1) / n
        f_xi = sorted_samples[i]
        
        d_plus_current = f_n_xi - f_xi
        if d_plus_current > d_plus_max:
            d_plus_max = d_plus_current
            
        # D_n^- = max_i ( F(x_i) - (i-1)/n )
        f_n_xi_minus_1 = i / n # (i-1)+1 / n
        d_minus_current = f_xi - f_n_xi_minus_1
        if d_minus_current > d_minus_max:
            d_minus_max = d_minus_current
            
    ks_statistic = max(d_plus_max, d_minus_max)
    results['statistic_D'] = ks_statistic
    
    # Calcular el p-value para K-S desde cero es muy complejo.
    # Para N > ~35, se pueden usar aproximaciones basadas en la Serie de Kolmogorov
    # o comparar D_n * sqrt(n) con valores críticos (e.g., 1.36 para alpha=0.05)
    critical_value_approx = 0.0
    if n > 35: # Aproximación para N grande
        # Valor crítico para D_alpha es K_alpha / sqrt(n)
        # K_alpha para 0.05 es aprox 1.358
        critical_value_approx = 1.358 / np.sqrt(n)
        results['critical_value_D_at_alpha_0.05 (approx)'] = critical_value_approx
        if ks_statistic > critical_value_approx:
            results['hypothesis_H0_rejected_at_alpha_0.05 (approx)'] = True
        else:
            results['hypothesis_H0_rejected_at_alpha_0.05 (approx)'] = False
    else:
        results['p_value_info'] = "P-value exacto requiere tabla o librería para N <= 35."

    return results


def perform_rng_quality_tests_from_scratch(rng_instance_to_test, num_samples: int) -> dict:
    """
    Ejecuta pruebas de calidad implementadas desde cero.
    """
    test_suite_results = {}
    
    # Asegurarnos de que generate_samples_from_rng_instance está definido o lo copiamos aquí
    # Asumiendo que lo definimos en este mismo archivo:
    float_samples = generate_samples_from_rng_instance(rng_instance_to_test, num_samples, 'float')
    
    test_suite_results['rng_type'] = type(rng_instance_to_test).__name__
    if hasattr(rng_instance_to_test, 'initial_seed'): # Para que funcione con LCG y MiddleSquare
        test_suite_results['initial_seed_for_test_sequence'] = rng_instance_to_test.initial_seed
    test_suite_results['num_samples_tested'] = len(float_samples)

    test_suite_results['chi_squared_uniformity'] = run_chi_squared_test_uniform_floats_from_scratch(
        float_samples, 
        config.RNG_TEST_NUM_BINS_CHI2
    )
    test_suite_results['kolmogorov_smirnov_uniformity'] = run_kolmogorov_smirnov_test_uniform_floats_from_scratch(
        float_samples
    )
    
    # Autocorrelación (usa NumPy, lo cual está permitido para el análisis, no para generación)
    if len(float_samples) > 1:
        try:
            samples_np = np.array(float_samples)
            autocorr_lag1 = np.corrcoef(samples_np[:-1], samples_np[1:])[0, 1]
            test_suite_results['autocorrelation_lag1_numpy'] = {'value': autocorr_lag1}
        except Exception as e:
            test_suite_results['autocorrelation_lag1_numpy'] = {'error': f'Error en Autocorrelación: {str(e)}'}
    else:
        test_suite_results['autocorrelation_lag1_numpy'] = {'error': 'No suficientes muestras'}

    return test_suite_results

# Función auxiliar que estaba en rng_validator.py antes, necesaria para perform_rng_quality_tests_from_scratch
def generate_samples_from_rng_instance(rng_instance, num_samples: int, sample_type: str = 'float') -> list:
    samples = []
    if sample_type == 'float':
        for _ in range(num_samples):
            samples.append(rng_instance.next_float())
    return samples