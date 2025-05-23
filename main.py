from drone_simulation.rng_handler import load_config_runtime, init_rngs
from drone_simulation.ui          import SimulationUI

if __name__ == '__main__':
    cfg  = load_config_runtime()
    rngs = init_rngs(cfg)
    ui   = SimulationUI(cfg, rngs)
    ui.run()
