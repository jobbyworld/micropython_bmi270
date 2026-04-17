"""Example09_LowPower.py -- Port MicroPython de Example09_LowPower.ino

Démontre le mode basse consommation du BMI270 :
  - Filtrage en mode puissance optimisée (POWER_OPT_MODE)
  - ODR réduit à 25 Hz pour les deux capteurs
  - Mode advanced power save (APS) activé
  - Suspend mode : capteurs désactivés entre les mesures

Consommation typique :
  - Accel seul actif : ~4 µA
  - Gyro seul actif  : ~400 µA
  - Accel + Gyro     : ~420 µA
  - Suspend (les deux désactivés) : ~3.5 µA

Matériel :
    Arduino NESSO / ESP32-C6 avec MicroPython
    BMI270 connecté en I2C (SCL=GPIO8, SDA=GPIO10)

Usage :
    Copier ce fichier et les modules bmi270 sur la carte, puis exécuter :
        import Example09_LowPower
"""

from machine import I2C, Pin
import utime
from expander import ExpanderPin, Expander
from st7789 import ST7789
import inconsolata_16 as medium

i2c = I2C(0, scl=Pin(8), sda=Pin(10), freq=400_000)
expander = Expander(i2c)
lcd = ST7789(rst=expander.LCD_RESET)
lcd.set_rotation(1)
lcd.fill("#FFFFFF")
lcd.text("Init imu...", 10, 10, '#000000', font=medium)
lcd.update()
expander.LCD_BACKLIGHT.write(1)

from SparkFun_BMI270_Arduino import BMI270
from bmi2_defs import (
    BMI2_OK,
    BMI2_ACCEL,
    BMI2_GYRO,
    BMI2_DRDY_ACC,
    BMI2_DRDY_GYR,
    BMI2_POWER_OPT_MODE,
    BMI2_ACC_ODR_25HZ,
    BMI2_GYR_ODR_25HZ,
)

# ---------------------------------------------------------------------------
# Initialisation du capteur
# ---------------------------------------------------------------------------

imu = BMI270(i2c)

print("Connexion au BMI270 en cours...")
while imu.beginI2C() != 0:
    print("BMI270 non connecté, vérification du câblage...")
    utime.sleep_ms(1000)

lcd.fill("#FFFFFF")
lcd.text("Connected", 10, 10, '#000000', font=medium)
lcd.update()
print("BMI270 connecté!")

# Passe les deux capteurs en mode basse consommation
imu.setAccelPowerMode(BMI2_POWER_OPT_MODE)
imu.setGyroPowerMode(BMI2_POWER_OPT_MODE, BMI2_POWER_OPT_MODE)

# ODR minimum commun aux deux capteurs (25 Hz)
imu.setAccelODR(BMI2_ACC_ODR_25HZ)
imu.setGyroODR(BMI2_GYR_ODR_25HZ)

# Active le mode advanced power save
imu.enableAdvancedPowerSave()

# Entre en suspend mode : désactive les deux capteurs
# (consommation ~3.5 µA pendant la veille)
imu.disableFeature(BMI2_ACCEL)
imu.disableFeature(BMI2_GYRO)

print("Mode basse consommation activé — mesure 1 fois/seconde")

# ---------------------------------------------------------------------------
# Boucle principale — mesure toutes les secondes
# ---------------------------------------------------------------------------
counter = 0
while True:
    # Période de veille (le MCU peut ici être mis en sommeil profond)
    utime.sleep_ms(1000)

    # Réveil : réactive les capteurs
    imu.enableFeature(BMI2_ACCEL)
    imu.enableFeature(BMI2_GYRO)

    # Attend que les données soient prêtes (DRDY bits dans STATUS)
    accel_drdy = False
    gyro_drdy  = False
    timeout_ms = 200
    t0 = utime.ticks_ms()
    while not (accel_drdy and gyro_drdy):
        rslt, status = imu.getStatus()
        if rslt == BMI2_OK:
            if status & BMI2_DRDY_ACC:
                accel_drdy = True
            if status & BMI2_DRDY_GYR:
                gyro_drdy = True
        if utime.ticks_diff(utime.ticks_ms(), t0) > timeout_ms:
            print("Timeout DRDY")
            break
        utime.sleep_ms(5)

    # Lit les données
    imu.getSensorData()

    # Suspend : désactive à nouveau les capteurs
    # (doit être fait APRÈS getSensorData(), sinon les valeurs seraient nulles)
    imu.disableFeature(BMI2_ACCEL)
    imu.disableFeature(BMI2_GYRO)

    counter += 1
    ax = imu.data.accelX
    ay = imu.data.accelY
    az = imu.data.accelZ
    gx = imu.data.gyroX
    gy = imu.data.gyroY
    gz = imu.data.gyroZ

    # Affichage série
    print("#{} Acc(g) X:{:.3f} Y:{:.3f} Z:{:.3f}  Gyr(°/s) X:{:.1f} Y:{:.1f} Z:{:.1f}".format(
        counter, ax, ay, az, gx, gy, gz))

    # Affichage LCD
    lcd.fill("#F0F8FF")
    lcd.text("LowPower #{}".format(counter), 10, 5, '#000055', font=medium)
    lcd.text("Acc (g)", 10, 25, '#000000', font=medium)
    lcd.text("X:{:6.3f}".format(ax), 10, 42, '#FF0000', font=medium)
    lcd.text("Y:{:6.3f}".format(ay), 10, 58, '#00AA00', font=medium)
    lcd.text("Z:{:6.3f}".format(az), 10, 74, '#0000FF', font=medium)
    lcd.text("Gyr (d/s)", 10, 95, '#000000', font=medium)
    lcd.text("X:{:7.1f}".format(gx), 10, 112, '#FF6600', font=medium)
    lcd.text("Y:{:7.1f}".format(gy), 10, 128, '#AA00FF', font=medium)
    lcd.text("Z:{:7.1f}".format(gz), 10, 144, '#00AAFF', font=medium)
    lcd.update()
