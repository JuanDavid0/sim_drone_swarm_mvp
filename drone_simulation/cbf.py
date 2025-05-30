# drone_simulation/cbf.py
import numpy as np
import random # Para el fallback de n_ij
# from . import config # YA NO IMPORTAMOS EL CONFIG GLOBAL AQUÍ

cbf_activation_count = 0
def reset_cbf_activation_count(): global cbf_activation_count; cbf_activation_count = 0
def get_cbf_activation_count(): global cbf_activation_count; return cbf_activation_count

def aplicar_cbf_simplificada(dron_actual, entidad_proxima, d_min, es_obstaculo, config_obj):
    global cbf_activation_count
    if not dron_actual.esta_activo or (not es_obstaculo and not entidad_proxima.esta_activo): return False

    pos_dron = dron_actual.posicion; vel_dron = dron_actual.velocidad
    pos_entidad = entidad_proxima.posicion
    vel_entidad = entidad_proxima.velocidad if not es_obstaculo else np.array([0.0, 0.0])
    p_diff = pos_entidad - pos_dron; dist_sq = np.sum(p_diff**2); dist = np.sqrt(dist_sq)
    h = dist_sq - d_min**2 
    v_rel = vel_dron - vel_entidad
    h_dot = 2 * np.dot(pos_dron - pos_entidad, v_rel)

    # Usar config_obj para los parámetros de CBF
    if h_dot + config_obj.CBF_GAMMA * h < 0:
        cbf_activation_count += 1
        if dist > 0: n_ij = p_diff / dist
        else:
            # Fallback si la distancia es cero
            temp_rng_for_cbf_fallback = random # Usar random global como fallback aquí
            n_ij = np.array([temp_rng_for_cbf_fallback.uniform(-1,1), temp_rng_for_cbf_fallback.uniform(-1,1)])
            norm_n_ij = np.linalg.norm(n_ij)
            if norm_n_ij > 0 : n_ij /= norm_n_ij
            else: n_ij = np.array([1.0, 0.0])
        
        vel_comp_hacia_entidad = np.dot(vel_dron, n_ij)
        if vel_comp_hacia_entidad > 0:
            correccion = -n_ij * vel_comp_hacia_entidad * (1 + config_obj.CBF_FACTOR_CORRECCION_VELOCIDAD) 
            dron_actual.velocidad += correccion
            norma_vel = np.linalg.norm(dron_actual.velocidad)
            if norma_vel > dron_actual.max_velocidad: # dron_actual.max_velocidad viene de su config_propia
                dron_actual.velocidad = (dron_actual.velocidad / norma_vel) * dron_actual.max_velocidad
            return True
    return False