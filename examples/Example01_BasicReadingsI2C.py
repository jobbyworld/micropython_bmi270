"""Example01_BasicReadingsI2C.py

Port MicroPython de :
    examples/Example01_BasicReadingsI2C/Example01_BasicReadingsI2C.ino

Carte cible : Arduino NESSO (ESP32-C6) avec MicroPython.

Brochage :
    BMI270 SDA → GPIO 10  (I2C SDA)
    BMI270 SCL → GPIO 8   (I2C SCL)
    BMI270 INT → GPIO 3   (non utilisé ici)
    BMI270 VDD → 3.3 V
    BMI270 GND → GND

Pré-requis :
    Les fichiers suivants doivent être présents dans le système de fichiers
    MicroPython (flash) de la carte, dans le même répertoire ou dans sys.path :
        bmi2_defs.py
        bmi2.py
        bmi270.py          (avec BMI270_CONFIG_FILE renseigné !)
        SparkFun_BMI270_Arduino.py

Usage :
    Copier ce fichier sur la carte (ex. avec mpremote ou Thonny), puis :
        import Example01_BasicReadingsI2C
    ou l'exécuter directement depuis le REPL.
"""

import utime
from SparkFun_BMI270_Arduino import BMI270

# Créer l'objet capteur
# imu = BMI270()
from machine import I2C, Pin

from expander import ExpanderPin,Expander

i2c = I2C(0, scl=Pin(8), sda=Pin(10), freq=400_000)
expander = Expander(i2c)
imu = BMI270(i2c)


# Adresse I2C (0x68 par défaut ; utiliser 0x69 si la broche SDO est haute)
I2C_ADDRESS = 0x68  # BMI2_I2C_PRIM_ADDR

print("BMI270 Example 1 - Basic Readings I2C")

# Initialiser le capteur (boucle jusqu'à ce qu'il réponde)
while True:
    err = imu.beginI2C()  # SCL=GPIO8, SDA=GPIO10 par défaut
    #err = imu.beginI2C(address=I2C_ADDRESS)  # SCL=GPIO8, SDA=GPIO10 par défaut
    if err == 0:
        break
    print("Erreur : BMI270 non connecte, verifier le cablage et l'adresse I2C !")
    utime.sleep_ms(1000)

print("BMI270 connecte !")

# Boucle de mesure
while True:
    # Lire les données du capteur (doit être appelé avant d'accéder à imu.data)
    imu.getSensorData()

    # Afficher l'accélération
    print(
        "Acceleration (g)"
        f"\tX: {imu.data.accelX:.3f}"
        f"\tY: {imu.data.accelY:.3f}"
        f"\tZ: {imu.data.accelZ:.3f}"
        "\t",
        end=""
    )

    # Afficher la vitesse angulaire
    print(
        "Rotation (deg/s)"
        f"\tX: {imu.data.gyroX:.3f}"
        f"\tY: {imu.data.gyroY:.3f}"
        f"\tZ: {imu.data.gyroZ:.3f}"
    )

    # 50 Hz
    utime.sleep_ms(20)
