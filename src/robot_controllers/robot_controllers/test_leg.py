import time
from adafruit_servokit import ServoKit

# Inizializza il modulo PCA9685
kit = ServoKit(channels=16)

# Imposta la frequenza PWM (Hz)
kit.frequency = 50

# Imposta i limiti di movimento dei servomotori
# Questi valori dipendono dai tuoi servomotori specifici
servo_min = 150  # Valore minimo del segnale PWM
servo_max = 600  # Valore massimo del segnale PWM

# Funzione per mappare un valore da un intervallo a un altro
def map_value(value, from_min, from_max, to_min, to_max):
    return (value - from_min) * (to_max - to_min) / (from_max - from_min) + to_min

try:
    while True:
        # Muovi i servomotori da 0 a 11
        for channel in range(12):
            # Calcola il valore PWM desiderato (0° a 180°)
            angle = map_value(channel, 0, 11, 0, 180)
            # Manda il segnale PWM al servomotore
            kit.servo[channel].angle = angle
            time.sleep(0.5)  # Pausa di 0.5 secondi
except KeyboardInterrupt:
    # Chiudi il programma in modo pulito alla pressione di Ctrl+C
    pass

# Riporta i servomotori alla posizione iniziale (90°) prima di uscire
for channel in range(12):
    kit.servo[channel].angle = 90

# Attendere un momento prima di terminare il programma
time.sleep(1)

# Chiudi il modulo PCA9685
kit.deinit()