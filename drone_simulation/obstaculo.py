# drone_simulation/obstaculo.py
import pygame
import numpy as np
import random # Para la decisión inicial si no se pasa rng_propio
# from . import config # YA NO IMPORTAMOS EL CONFIG GLOBAL AQUÍ

class Obstaculo:
    _id_counter = -1 # Si quieres IDs para obstáculos y reiniciarlos

    def __init__(self, x, y, radio_param, color, config_obj, es_dinamico=False, tiempo_vida=0, tiempo_respawn=0, rng_para_dinamica=None):
        if hasattr(Obstaculo, '_id_counter'): Obstaculo._id_counter +=1
        self.id = Obstaculo._id_counter if hasattr(Obstaculo, '_id_counter') else -1 # Asignar ID
        
        self.config_propia = config_obj

        self.posicion_original = np.array([float(x), float(y)])
        self.posicion = np.array([float(x), float(y)])
        self.radio_original = radio_param
        self.radio = radio_param
        self.color = color
        
        self.es_dinamico = es_dinamico
        self.esta_activo = True
        self.rng_propio = rng_para_dinamica
        
        if self.es_dinamico:
            self.tiempo_vida_configurado = tiempo_vida
            self.tiempo_respawn_configurado = tiempo_respawn
            self.contador_tiempo_estado = 0.0
            
            rng_usar = self.rng_propio if self.rng_propio else random # Fallback
            if rng_usar.next_float() < 0.5 : 
                self.esta_activo = True
                self.contador_tiempo_estado = self.tiempo_vida_configurado
            else:
                self.esta_activo = False
                self.contador_tiempo_estado = self.tiempo_respawn_configurado
        else:
            self.tiempo_vida_configurado = float('inf')
            self.tiempo_respawn_configurado = 0
            self.contador_tiempo_estado = float('inf')

    def actualizar(self, delta_t, rng_para_nuevos_tiempos=None):
        if not self.es_dinamico:
            return

        rng_usar = rng_para_nuevos_tiempos if rng_para_nuevos_tiempos else self.rng_propio
        if rng_usar is None: rng_usar = random # Último fallback

        self.contador_tiempo_estado -= delta_t

        if self.esta_activo and self.contador_tiempo_estado <= 0:
            self.esta_activo = False
            self.contador_tiempo_estado = rng_usar.next_float() * \
                                         (self.config_propia.TIEMPO_RESPAWN_OBSTACULO_MAX - self.config_propia.TIEMPO_RESPAWN_OBSTACULO_MIN) + \
                                         self.config_propia.TIEMPO_RESPAWN_OBSTACULO_MIN
            self.tiempo_respawn_configurado = self.contador_tiempo_estado
        elif not self.esta_activo and self.contador_tiempo_estado <= 0:
            self.esta_activo = True
            self.posicion = self.posicion_original
            self.radio = rng_usar.next_float() * \
                         (self.config_propia.MAX_TAMANO_OBSTACULO - self.config_propia.MIN_TAMANO_OBSTACULO) + self.config_propia.MIN_TAMANO_OBSTACULO
            self.contador_tiempo_estado = rng_usar.next_float() * \
                                         (self.config_propia.TIEMPO_VIDA_OBSTACULO_MAX - self.config_propia.TIEMPO_VIDA_OBSTACULO_MIN) + \
                                         self.config_propia.TIEMPO_VIDA_OBSTACULO_MIN
            self.tiempo_vida_configurado = self.contador_tiempo_estado

    def dibujar(self, pantalla):
        if self.esta_activo:
            pygame.draw.circle(pantalla, self.color, 
                               (int(self.posicion[0]), int(self.posicion[1])), 
                               int(self.radio))