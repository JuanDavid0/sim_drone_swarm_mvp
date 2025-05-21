# drone_simulation/cbf.py
import numpy as np
from . import config

# Contador global para activaciones de CBF
cbf_activation_count = 0

def reset_cbf_activation_count():
    global cbf_activation_count
    cbf_activation_count = 0

def get_cbf_activation_count():
    global cbf_activation_count
    return cbf_activation_count

def aplicar_cbf_simplificada(dron_actual, entidad_proxima, d_min, es_obstaculo=False):
    """
    CBF Simplificada para evitar colisión entre dron_actual y entidad_proxima (otro dron u obstáculo).
    Modifica la velocidad de dron_actual si es necesario.
    Retorna True si la CBF se activó y modificó la velocidad, False en caso contrario.
    
    Args:
        dron_actual: El objeto Drone que se está evaluando.
        entidad_proxima: Otro objeto Drone o un objeto Obstaculo.
        d_min: La distancia mínima de seguridad deseada entre los centros (o centro a superficie).
        es_obstaculo: Booleano, True si entidad_proxima es un obstáculo.
    """
    global cbf_activation_count

    if not dron_actual.esta_activo or (not es_obstaculo and not entidad_proxima.esta_activo):
        return False

    pos_dron = dron_actual.posicion
    vel_dron = dron_actual.velocidad
    radio_dron = dron_actual.radio

    pos_entidad = entidad_proxima.posicion
    radio_entidad = entidad_proxima.radio
    vel_entidad = entidad_proxima.velocidad if not es_obstaculo else np.array([0.0, 0.0]) # Obstáculos son estáticos para CBF aquí

    # Vector del dron actual a la entidad próxima
    p_diff = pos_entidad - pos_dron
    dist_sq = np.sum(p_diff**2)
    dist = np.sqrt(dist_sq)

    # Función de barrera h(x) = ||p_i - p_j||^2 - d_min^2 para centros
    # O h(x) = ||p_i - p_j|| - (radio_i + radio_j + d_safety_surface) para superficies
    # Usaremos d_min como la distancia deseada entre centros por simplicidad aquí,
    # así que h(x) = dist^2 - d_min^2
    
    # d_min que se pasa ya debería ser la distancia de seguridad entre centros
    h = dist_sq - d_min**2 
    
    # Si ya estamos demasiado cerca (h < 0), la CBF debería actuar
    # y si nos estamos acercando aún más (p_diff y v_relativa tienen producto escalar negativo)
    
    v_rel = vel_dron - vel_entidad # Velocidad relativa del dron hacia la entidad
    
    # Derivada de h: h_dot = 2 * (p_i - p_j) . (v_i - v_j) = -2 * p_diff . v_rel
    # (Nota: p_diff = pos_entidad - pos_dron, por lo que (pos_dron - pos_entidad) . v_rel)
    h_dot = 2 * np.dot(pos_dron - pos_entidad, v_rel)

    # Condición de seguridad de la CBF: h_dot + gamma * h >= 0
    # Si se viola (h_dot + gamma * h < 0), se necesita corrección.
    if h_dot + config.CBF_GAMMA * h < 0:
        cbf_activation_count += 1
        
        # Corrección Simplificada:
        # Queremos modificar vel_dron para que la condición se cumpla.
        # Una forma simple es proyectar la velocidad del dron en la dirección perpendicular
        # al vector p_diff, o reducir la componente de velocidad que causa la aproximación.

        # Vector normalizado de dron a entidad
        if dist > 0:
            n_ij = p_diff / dist
        else: # Superposición exacta, empujar en una dirección aleatoria o predefinida
            n_ij = np.array([random.uniform(-1,1), random.uniform(-1,1)])
            norm_n_ij = np.linalg.norm(n_ij)
            if norm_n_ij > 0 : n_ij /= norm_n_ij
            else: n_ij = np.array([1.0, 0.0])


        # Componente de la velocidad del dron en la dirección de n_ij (hacia la entidad)
        vel_componente_hacia_entidad = np.dot(vel_dron, n_ij)

        if vel_componente_hacia_entidad > 0: # Solo corregir si se está moviendo hacia la entidad
            # Reducir o invertir esta componente de velocidad.
            # Por ejemplo, hacerla cero o ligeramente negativa.
            # vel_dron_corregida = vel_dron - vel_componente_hacia_entidad * n_ij # Elimina componente hacia
            # vel_dron_corregida -= n_ij * config.CBF_FACTOR_CORRECCION_VELOCIDAD * vel_componente_hacia_entidad # Invierte y reduce
            
            # Otra corrección más directa: frenar o desviar
            # Intentar modificar la velocidad para que h_dot + gamma * h = 0 (límite de seguridad)
            # Original: Lf_h + Lg_h * u + gamma * h >= 0.  u es el control (aceleración o cambio de vel).
            # Aquí modificamos la velocidad directamente.
            # Si h_dot = 2 * np.dot(pos_dron - pos_entidad, vel_dron - vel_entidad)
            # Queremos h_dot_corregido = -gamma * h
            # 2 * np.dot(pos_dron - pos_entidad, vel_dron_corregido - vel_entidad) = -gamma * h
            # Sea p_ji = pos_dron - pos_entidad
            # 2 * np.dot(p_ji, vel_dron_corregido) - 2 * np.dot(p_ji, vel_entidad) = -gamma * h
            # 2 * np.dot(p_ji, vel_dron_corregido) = -gamma * h + 2 * np.dot(p_ji, vel_entidad)
            
            # Esta es la parte que resolvería un QP. Como es simplificado:
            # Desviar la velocidad actual perpendicularmente a p_diff o reducirla.
            # Reducir la componente de velocidad que va hacia la colisión.
            correccion = -n_ij * vel_componente_hacia_entidad * (1 + config.CBF_FACTOR_CORRECCION_VELOCIDAD) 
            # El (1 + factor) intenta invertir y añadir un margen. Factor=1 invierte completamente.
            dron_actual.velocidad += correccion
            
            # Asegurarse de que la velocidad no exceda el máximo después de la corrección.
            norma_vel = np.linalg.norm(dron_actual.velocidad)
            if norma_vel > dron_actual.max_velocidad:
                dron_actual.velocidad = (dron_actual.velocidad / norma_vel) * dron_actual.max_velocidad
            return True # Se activó y modificó

    return False # No se activó o no se modificó