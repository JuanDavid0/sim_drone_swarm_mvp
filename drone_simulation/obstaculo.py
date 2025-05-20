# drone_simulation/obstaculo.py
import pygame
import numpy as np
import random
from . import config

class Obstaculo:
    def __init__(self, x, y, radio, color, es_dinamico=False, tiempo_vida=0, tiempo_respawn=0):
        self.posicion_original = np.array([float(x), float(y)]) # Guardamos la original para respawn
        self.posicion = np.array([float(x), float(y)])
        self.radio = radio
        self.color = color
        
        self.es_dinamico = es_dinamico
        self.esta_activo = True
        
        if self.es_dinamico:
            self.tiempo_vida_configurado = tiempo_vida
            self.tiempo_respawn_configurado = tiempo_respawn
            self.contador_tiempo_estado = 0.0 # Contador para vida o respawn
            
            # Decidir estado inicial para dinámicos (50/50 activo o inactivo esperando respawn)
            if random.choice([True, False]):
                self.esta_activo = True
                self.contador_tiempo_estado = self.tiempo_vida_configurado
            else:
                self.esta_activo = False
                self.contador_tiempo_estado = self.tiempo_respawn_configurado
        else: # Obstáculos estáticos
            self.tiempo_vida_configurado = float('inf')
            self.tiempo_respawn_configurado = 0
            self.contador_tiempo_estado = float('inf')


    def actualizar(self, delta_t):
        if not self.es_dinamico:
            return

        self.contador_tiempo_estado -= delta_t

        if self.esta_activo and self.contador_tiempo_estado <= 0:
            # Tiempo de vida terminó, desactivar y poner a respawn
            self.esta_activo = False
            self.contador_tiempo_estado = self.tiempo_respawn_configurado
            # print(f"Obstáculo en {self.posicion_original} desactivado, respawning en {self.contador_tiempo_estado}s")
        elif not self.esta_activo and self.contador_tiempo_estado <= 0:
            # Tiempo de respawn terminó, reactivar y poner tiempo de vida
            self.esta_activo = True
            self.posicion = self.posicion_original # Reaparece en su posición original
            self.radio = random.uniform(config.MIN_TAMANO_OBSTACULO, config.MAX_TAMANO_OBSTACULO) # Puede reaparecer con otro tamaño
            self.contador_tiempo_estado = random.uniform(config.TIEMPO_VIDA_OBSTACULO_MIN, config.TIEMPO_VIDA_OBSTACULO_MAX)
            self.tiempo_vida_configurado = self.contador_tiempo_estado # Actualiza el tiempo de vida para este ciclo
            # print(f"Obstáculo en {self.posicion_original} reactivado por {self.contador_tiempo_estado}s")


    def dibujar(self, pantalla):
        if self.esta_activo:
            pygame.draw.circle(pantalla, self.color, 
                               (int(self.posicion[0]), int(self.posicion[1])), 
                               int(self.radio)) # Asegurar que el radio sea entero para dibujar