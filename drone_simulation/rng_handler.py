# rng_handler.py
import sys, os, json, types
from . import config as default_config
from .rng import LCG, MiddleSquareRNG

def load_config_runtime():
    """Carga valores por defecto y reescribe con config_runtime.json si existe."""
    cfg = types.SimpleNamespace()
    # 1. Copiar todos los valores por defecto desde default_config a cfg
    for attr in dir(default_config):
        if not attr.startswith("__"):
            setattr(cfg, attr, getattr(default_config, attr))

    # 2. Sobrescribir con valores de config_runtime.json si existe y se usa
    json_path = "config_runtime.json"
    if "--use-runtime-config" in sys.argv and os.path.exists(json_path):
        try:
            with open(json_path, "r") as f:
                data_from_json = json.load(f)
        except json.JSONDecodeError:
            if hasattr(default_config, 'VERBOSE') and default_config.VERBOSE:
                print(f"Error al decodificar {json_path}. Se usarán los valores de config.py por defecto.")
            data_from_json = {} # Evita más errores, usa defaults

        for key, raw_value_from_json in data_from_json.items():
            if not hasattr(cfg, key): # Si la clave del JSON no es un parámetro conocido en config.py
                if hasattr(default_config, 'VERBOSE') and default_config.VERBOSE:
                    print(f"Nota: Clave '{key}' en {json_path} no reconocida. Se ignora.")
                continue

            default_value_from_config_module = getattr(default_config, key)
            value_to_set = None # Valor final a asignar

            if "SEED" in key: # Manejo especial para todas las semillas
                if raw_value_from_json in (None, ""): # JSON 'null' o cadena vacía
                    value_to_set = None # Indica semilla aleatoria
                else:
                    try:
                        value_to_set = int(str(raw_value_from_json)) # Convertir a int (después de str para robustez)
                    except (ValueError, TypeError):
                        if hasattr(default_config, 'VERBOSE') and default_config.VERBOSE:
                            print(f"Advertencia: Valor de semilla '{raw_value_from_json}' para '{key}' no es un entero válido. Se usará semilla aleatoria (None).")
                        value_to_set = None
            
            elif isinstance(default_value_from_config_module, int):
                if raw_value_from_json is None: # Si JSON provee 'null' para un campo entero
                    if hasattr(default_config, 'VERBOSE') and default_config.VERBOSE:
                        print(f"Advertencia: Valor para '{key}' (entero) es 'null' en JSON. Se mantendrá el valor por defecto de config.py: {default_value_from_config_module}.")
                    value_to_set = default_value_from_config_module # Usar el valor original de config.py
                else:
                    try:
                        value_to_set = int(raw_value_from_json)
                    except (ValueError, TypeError):
                        if hasattr(default_config, 'VERBOSE') and default_config.VERBOSE:
                            print(f"Advertencia: No se pudo convertir '{raw_value_from_json}' a entero para '{key}'. Usando valor por defecto: {default_value_from_config_module}.")
                        value_to_set = default_value_from_config_module

            elif isinstance(default_value_from_config_module, float):
                if raw_value_from_json is None: # Si JSON provee 'null' para un campo flotante
                    if hasattr(default_config, 'VERBOSE') and default_config.VERBOSE:
                        print(f"Advertencia: Valor para '{key}' (flotante) es 'null' en JSON. Se mantendrá el valor por defecto de config.py: {default_value_from_config_module}.")
                    value_to_set = default_value_from_config_module # Usar el valor original de config.py
                else:
                    try:
                        value_to_set = float(raw_value_from_json)
                    except (ValueError, TypeError):
                        if hasattr(default_config, 'VERBOSE') and default_config.VERBOSE:
                            print(f"Advertencia: No se pudo convertir '{raw_value_from_json}' a flotante para '{key}'. Usando valor por defecto: {default_value_from_config_module}.")
                        value_to_set = default_value_from_config_module
            
            elif isinstance(default_value_from_config_module, bool):
                # bool(None) es False, bool("") es False. Esto es generalmente aceptable.
                # Si raw_value_from_json es "False" o "True" como cadena, bool() no funciona como se espera.
                # Asumiendo que el JSON guarda booleanos como true/false literales, o el launcher los convierte.
                if isinstance(raw_value_from_json, str):
                    if raw_value_from_json.lower() == 'true':
                        value_to_set = True
                    elif raw_value_from_json.lower() == 'false':
                        value_to_set = False
                    else: # No es una cadena booleana reconocible, usar default
                        if hasattr(default_config, 'VERBOSE') and default_config.VERBOSE:
                             print(f"Advertencia: Valor de cadena '{raw_value_from_json}' para booleano '{key}' no reconocido. Usando por defecto: {default_value_from_config_module}")
                        value_to_set = default_value_from_config_module
                else: # Si ya es bool, o None, o numérico (0=False, otro=True)
                    value_to_set = bool(raw_value_from_json)

            else: # Para strings u otros tipos que se copian tal cual (listas, tuplas si las hubiera)
                value_to_set = raw_value_from_json
            
            setattr(cfg, key, value_to_set)
    return cfg

def init_rngs(config):
    """Instancia y retorna las tuplas de RNG usados en la simulación."""
    return (
        LCG(
            seed      = config.GCL_SEED_ENTORNO,
            multiplier= config.GCL_MULTIPLIER_A,
            increment = config.GCL_INCREMENT_C,
            modulus   = config.GCL_MODULUS_M
        ),
        MiddleSquareRNG(
            seed      = config.MIDDLE_SQUARE_SEED_DRONES,
            num_digits= config.N_DIGITS_MIDDLE_SQUARE
        ),
        LCG(
            seed      = config.GCL_SEED_OBSTACULOS_DYN,
            multiplier= config.GCL_MULTIPLIER_A_OBS,
            increment = config.GCL_INCREMENT_C_OBS,
            modulus   = config.GCL_MODULUS_M_OBS
        )
    )