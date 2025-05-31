# drone_simulation/rng.py
import time # Solo para generar una semilla inicial si no se proporciona

class LCG:
    """Generador Congruencial Lineal: X_n+1 = (a * X_n + c) mod m."""
    def __init__(self, seed=None, multiplier=1664525, increment=1013904223, modulus=2**32):
        self.multiplier = multiplier
        self.increment = increment
        self.modulus = modulus
        self.initial_seed = seed
        if seed is None:
            self.current_value = int(time.time() * 1000) % self.modulus
            if self.initial_seed is None: self.initial_seed = self.current_value # Store generated seed
        else:
            self.current_value = seed % self.modulus
        
        if self.initial_seed is None and seed is not None: # Ensure initial_seed is set if seed was provided
             self.initial_seed = seed % self.modulus


    def set_seed(self, seed):
        self.initial_seed = seed
        if seed is None:
            self.current_value = int(time.time() * 1000) % self.modulus
            if self.initial_seed is None: self.initial_seed = self.current_value
        else:
            self.current_value = seed % self.modulus

        if self.initial_seed is None and seed is not None:
             self.initial_seed = seed % self.modulus


    def _next_raw(self):
        self.current_value = (self.multiplier * self.current_value + self.increment) % self.modulus
        return self.current_value

    def next_float(self):
        """Genera un flotante en [0.0, 1.0)."""
        return self._next_raw() / self.modulus

    def next_int(self, lower_bound, upper_bound):
        """Genera un entero en [lower_bound, upper_bound]."""
        if lower_bound > upper_bound:
            raise ValueError("El límite inferior no puede ser mayor que el superior.")
        range_width = upper_bound - lower_bound + 1
        return lower_bound + (self._next_raw() % range_width)

class MiddleSquareRNG:
    """Generador de Cuadrados Medios."""
    def __init__(self, seed=None, num_digits=4):
        self.num_digits = num_digits
        self.max_seed_val = (10**self.num_digits) -1
        self.initial_seed = seed

        if seed is None:
            # Generar una semilla aleatoria de num_digits si no se provee
            # Usamos LCG para generar esta semilla inicial para no usar 'random' directamente aquí.
            temp_lcg_for_seed = LCG(int(time.time() * 1000))
            self.current_seed_int = temp_lcg_for_seed.next_int(0, self.max_seed_val)
            if self.initial_seed is None: self.initial_seed = self.current_seed_int
        else:
            if not (0 <= seed <= self.max_seed_val and len(str(seed)) <= self.num_digits) :
                 # Si la semilla es muy grande, la truncamos o tomamos módulo, o advertimos.
                 # Por ahora, un módulo simple o truncamiento
                 print(f"Advertencia: Semilla {seed} para MiddleSquareRNG fuera de rango para {num_digits} digitos. Se ajustará.")
                 self.current_seed_int = seed % (10**self.num_digits)
            else:
                self.current_seed_int = seed
        
        if self.initial_seed is None and seed is not None:
            self.initial_seed = self.current_seed_int


    def set_seed(self, seed):
        self.initial_seed = seed
        if seed is None:
            temp_lcg_for_seed = LCG(int(time.time() * 1000))
            self.current_seed_int = temp_lcg_for_seed.next_int(0, self.max_seed_val)
            if self.initial_seed is None: self.initial_seed = self.current_seed_int
        else:
            if not (0 <= seed <= self.max_seed_val and len(str(seed)) <= self.num_digits) :
                 print(f"Advertencia: Semilla {seed} para MiddleSquareRNG fuera de rango para {self.num_digits} digitos. Se ajustará.")
                 self.current_seed_int = seed % (10**self.num_digits)
            else:
                self.current_seed_int = seed
        
        if self.initial_seed is None and seed is not None:
            self.initial_seed = self.current_seed_int

    def _next_raw_int(self):
        squared = self.current_seed_int ** 2
        s_squared = str(squared).zfill(self.num_digits * 2) # Asegurar 2N dígitos con ceros a la izquierda
        
        # Extraer los N dígitos del medio
        start_index = self.num_digits // 2 
        # Si num_digits es par, ej 4: 2*N=8, start=2, end=2+4=6 -> s_squared[2:6]
        # Si num_digits es impar, ej 3: 2*N=6, start=1, end=1+3=4 -> s_squared[1:4]
        # La fórmula para el inicio del segmento medio de longitud N en una cadena de 2N es (2N - N) / 2 = N/2
        middle_digits_str = s_squared[start_index : start_index + self.num_digits]
        
        self.current_seed_int = int(middle_digits_str)
        return self.current_seed_int

    def next_float(self):
        """Genera un flotante en [0.0, 1.0) normalizando el entero de N dígitos."""
        # El entero raw es de 0 a 10^N - 1
        return self._next_raw_int() / (10**self.num_digits)

    def next_int(self, lower_bound, upper_bound):
        """Genera un entero en [lower_bound, upper_bound]."""
        if lower_bound > upper_bound:
            raise ValueError("El límite inferior no puede ser mayor que el superior.")
        range_width = upper_bound - lower_bound + 1
        # Usamos el flotante para obtener una distribución más uniforme en el rango
        return lower_bound + int(self.next_float() * range_width)
