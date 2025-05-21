# drone_simulation/obstaculo.py
import pygame
import numpy as np
from . import config

class Obstaculo:
    # _id_counter = 0 # Si quieres IDs para obstáculos

    def __init__(self, x, y, radio, color, es_dinamico=False, tiempo_vida=0, tiempo_respawn=0, rng_para_dinamica=None):
        # Obstaculo._id_counter += 1
        # self.id = Obstaculo._id_counter
        self.posicion_original = np.array([float(x), float(y)])
        self.posicion = np.array([float(x), float(y)])
        self.radio_original = radio # Guardar radio original si cambia
        self.radio = radio
        self.color = color
        
        self.es_dinamico = es_dinamico
        self.esta_activo = True
        self.rng_propio = rng_para_dinamica # RNG para sus decisiones dinámicas
        
        if self.es_dinamico:
            self.tiempo_vida_configurado = tiempo_vida
            self.tiempo_respawn_configurado = tiempo_respawn
            self.contador_tiempo_estado = 0.0
            
            # Usar el RNG propio para la decisión inicial
            if self.rng_propio.next_float() < 0.5 : # 50/50
                self.esta_activo = True
                self.contador_tiempo_estado = self.tiempo_vida_configurado
            else:
                self.esta_activo = False
                self.contador_tiempo_estado = self.tiempo_respawn_configurado
        else:
            self.tiempo_vida_configurado = float('inf')
            self.tiempo_respawn_configurado = 0
            self.contador_tiempo_estado = float('inf')

    def actualizar(self, delta_t, rng_para_nuevos_tiempos=None): # rng_para_nuevos_tiempos podría ser el mismo que self.rng_propio
        if not self.es_dinamico:
            return

        if rng_para_nuevos_tiempos is None: # Fallback
            rng_para_nuevos_tiempos = self.rng_propio

        self.contador_tiempo_estado -= delta_t

        if self.esta_activo and self.contador_tiempo_estado <= 0:
            self.esta_activo = False
            self.contador_tiempo_estado = rng_para_nuevos_tiempos.next_float() * \
                                         (config.TIEMPO_RESPAWN_OBSTACULO_MAX - config.TIEMPO_RESPAWN_OBSTACULO_MIN) + \
                                         config.TIEMPO_RESPAWN_OBSTACULO_MIN
            self.tiempo_respawn_configurado = self.contador_tiempo_estado
        elif not self.esta_activo and self.contador_tiempo_estado <= 0:
            self.esta_activo = True
            self.posicion = self.posicion_original # Reaparece en su posición original
            # Nuevo tamaño y tiempo de vida usando el RNG
            self.radio = rng_para_nuevos_tiempos.next_float() * \
                         (config.MAX_TAMANO_OBSTACULO - config.MIN_TAMANO_OBSTACULO) + config.MIN_TAMANO_OBSTACULO
            self.contador_tiempo_estado = rng_para_nuevos_tiempos.next_float() * \
                                         (config.TIEMPO_VIDA_OBSTACULO_MAX - config.TIEMPO_VIDA_OBSTACULO_MIN) + \
                                         config.TIEMPO_VIDA_OBSTACULO_MIN
            self.tiempo_vida_configurado = self.contador_tiempo_estado

    def dibujar(self, pantalla):
        # ... (sin cambios) ...
        if self.esta_activo:
            pygame.draw.circle(pantalla, self.color, 
                               (int(self.posicion[0]), int(self.posicion[1])), 
                               int(self.radio))