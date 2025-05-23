# rng_handler.py
import sys, os, json, types
from . import config as default_config
from .rng import LCG, MiddleSquareRNG

def load_config_runtime():
    """Carga valores por defecto y reescribe con config_runtime.json si existe."""
    cfg = types.SimpleNamespace()
    # Copiar defaults
    for attr in dir(default_config):
        if not attr.startswith("__"):
            setattr(cfg, attr, getattr(default_config, attr))
    # Sobrescribir con runtime
    json_path = "config_runtime.json"
    if "--use-runtime-config" in sys.argv and os.path.exists(json_path):
        data = json.load(open(json_path))
        for key, raw in data.items():
            if not hasattr(cfg, key):
                continue
            default = getattr(default_config, key)
            # Semillas o default None -> int
            if ("SEED" in key or default is None) and raw not in (None, ""):
                try:
                    val = int(raw)
                except ValueError:
                    val = None
            # Enteros
            elif isinstance(default, int):
                val = int(raw)
            # Flotantes
            elif isinstance(default, float):
                val = float(raw)
            else:
                val = raw
            setattr(cfg, key, val)
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