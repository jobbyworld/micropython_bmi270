"""Example06_CalibrationNVM.py -- Port MicroPython de Example06_CalibrationNVM.ino

Effectue la calibration du BMI270 et sauvegarde les valeurs en NVM.

Séquence :
  1. Attend une entrée utilisateur (touche Entrée sur le terminal série)
  2. Affiche les valeurs moyennes AVANT calibration
  3. Effectue le réétalonnage composant gyroscope (CRT — stub en MicroPython)
  4. Effectue la calibration FOC accéléromètre (gravity +Z)
  5. Effectue la calibration FOC gyroscope
  6. Affiche les valeurs moyennes APRÈS calibration
  7. Propose de sauvegarder en NVM (entrée 'Y')

AVERTISSEMENT :
    La NVM du BMI270 ne supporte que 14 cycles d'écriture au TOTAL !

Matériel :
    Arduino NESSO / ESP32-C6 avec MicroPython
    BMI270 connecté en I2C (SCL=GPIO8, SDA=GPIO10)
    Capteur posé à PLAT, immobile, axe Z vers le haut pendant la calibration

Usage :
    Copier ce fichier et les modules bmi270 sur la carte, puis exécuter :
        import Example06_CalibrationNVM
    Suivre les instructions dans le terminal série (REPL).
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

from SparkFun_BMI270_Arduino import (
    BMI270,
    BMI2_GRAVITY_POS_Z,
)
from bmi2_defs import BMI2_OK

# ---------------------------------------------------------------------------
# Initialisation du capteur
# ---------------------------------------------------------------------------

imu = BMI270(i2c)

print("Connexion au BMI270 en cours...")
while imu.beginI2C() != 0:
    print("BMI270 non connecté, vérification du câblage...")
    utime.sleep_ms(1000)

lcd.fill("#FFFFFF")
lcd.text("BMI270 connecte!", 10, 10, '#000000', font=medium)
lcd.update()
print("BMI270 connecté!")


# ---------------------------------------------------------------------------
# Fonction helper : moyenne sur 50 mesures à 50 Hz
# ---------------------------------------------------------------------------

def print_average_sensor_values():
    """Lit 50 échantillons et affiche les moyennes."""
    sum_ax = sum_ay = sum_az = 0.0
    sum_gx = sum_gy = sum_gz = 0.0
    n = 50

    lcd.fill("#FFFFF0")
    lcd.text("Mesure...", 10, 10, '#444400', font=medium)
    lcd.update()

    for _ in range(n):
        imu.getSensorData()
        sum_ax += imu.data.accelX
        sum_ay += imu.data.accelY
        sum_az += imu.data.accelZ
        sum_gx += imu.data.gyroX
        sum_gy += imu.data.gyroY
        sum_gz += imu.data.gyroZ
        utime.sleep_ms(20)

    ax, ay, az = sum_ax/n, sum_ay/n, sum_az/n
    gx, gy, gz = sum_gx/n, sum_gy/n, sum_gz/n

    print("  Acc(g)   X:{:.3f}  Y:{:.3f}  Z:{:.3f}".format(ax, ay, az))
    print("  Gyr(d/s) X:{:.3f}  Y:{:.3f}  Z:{:.3f}".format(gx, gy, gz))

    lcd.fill("#FFFFF0")
    lcd.text("Acc (g) moy.", 10, 5, '#000055', font=medium)
    lcd.text("X:{:6.3f}".format(ax), 10, 22, '#FF0000', font=medium)
    lcd.text("Y:{:6.3f}".format(ay), 10, 38, '#00AA00', font=medium)
    lcd.text("Z:{:6.3f}".format(az), 10, 54, '#0000FF', font=medium)
    lcd.text("Gyr (d/s) moy.", 10, 75, '#000055', font=medium)
    lcd.text("X:{:7.3f}".format(gx), 10, 92, '#FF6600', font=medium)
    lcd.text("Y:{:7.3f}".format(gy), 10, 108, '#AA00FF', font=medium)
    lcd.text("Z:{:7.3f}".format(gz), 10, 124, '#00AAFF', font=medium)
    lcd.update()


# ---------------------------------------------------------------------------
# Attente de l'utilisateur (simulée — appui sur Entrée dans le REPL)
# ---------------------------------------------------------------------------

print()
print("Posez le capteur sur une surface plane et immobile.")
print("Appuyez sur Entrée pour démarrer la calibration...")
lcd.fill("#FFFFFF")
lcd.text("Pose le capteur", 10, 10, '#000000', font=medium)
lcd.text("a plat + immobile", 10, 28, '#000000', font=medium)
lcd.text("Entrée = start", 10, 55, '#0000AA', font=medium)
lcd.update()

try:
    input()   # attend Entrée dans le terminal REPL
except Exception:
    utime.sleep_ms(3000)  # fallback si input() non disponible

# ---------------------------------------------------------------------------
# Valeurs avant calibration
# ---------------------------------------------------------------------------

print()
print("Valeurs moyennes AVANT calibration :")
print_average_sensor_values()
print()

# ---------------------------------------------------------------------------
# Calibration
# ---------------------------------------------------------------------------

print("Réétalonnage composant (CRT) gyroscope...")
lcd.fill("#FFFFF0")
lcd.text("CRT gyro...", 10, 10, '#555500', font=medium)
lcd.update()
rslt = imu.performComponentRetrim()
if rslt != BMI2_OK:
    print("Erreur CRT:", rslt)

print("Calibration FOC accéléromètre (gravity = +Z)...")
lcd.fill("#FFFFF0")
lcd.text("FOC accel...", 10, 10, '#555500', font=medium)
lcd.text("NE PAS BOUGER!", 10, 28, '#AA0000', font=medium)
lcd.update()
rslt = imu.performAccelOffsetCalibration(BMI2_GRAVITY_POS_Z)
if rslt != BMI2_OK:
    print("Erreur FOC accel:", rslt)
else:
    print("FOC accéléromètre OK!")

print("Calibration FOC gyroscope...")
lcd.fill("#FFFFF0")
lcd.text("FOC gyro...", 10, 10, '#555500', font=medium)
lcd.text("NE PAS BOUGER!", 10, 28, '#AA0000', font=medium)
lcd.update()
rslt = imu.performGyroOffsetCalibration()
if rslt != BMI2_OK:
    print("Erreur FOC gyro:", rslt)
else:
    print("FOC gyroscope OK!")

# ---------------------------------------------------------------------------
# Valeurs après calibration
# ---------------------------------------------------------------------------

print()
print("Calibration terminée!")
print()
print("Valeurs moyennes APRÈS calibration :")
print_average_sensor_values()
print()

# ---------------------------------------------------------------------------
# Sauvegarde NVM (optionnelle)
# ---------------------------------------------------------------------------

print("Ces valeurs peuvent être sauvegardées en NVM (14 cycles max!).")
print("Voulez-vous sauvegarder en NVM ? (tapez Y + Entrée pour confirmer)")
lcd.fill("#FFEECC")
lcd.text("Sauvegarder NVM?", 10, 10, '#664400', font=medium)
lcd.text("Y + Enter = oui", 10, 35, '#0000AA', font=medium)
lcd.text("14 cycles max!", 10, 60, '#AA0000', font=medium)
lcd.update()

try:
    answer = input().strip()
except Exception:
    answer = ""

if answer.upper() == 'Y':
    print()
    print("!!! ATTENTION : 14 cycles NVM au TOTAL !!!")
    print("Confirmez en tapant Y à nouveau :")
    try:
        confirm = input().strip()
    except Exception:
        confirm = ""

    if confirm.upper() == 'Y':
        rslt = imu.saveNVM()
        if rslt == BMI2_OK:
            print("Valeurs sauvegardées en NVM!")
            lcd.fill("#00FF88")
            lcd.text("NVM sauvegarde!", 10, 10, '#004400', font=medium)
        else:
            print("Erreur NVM:", rslt)
            lcd.fill("#FF4444")
            lcd.text("Erreur NVM!", 10, 10, '#440000', font=medium)
        lcd.update()
    else:
        print("Sauvegarde annulée.")
else:
    print("Sauvegarde NVM ignorée.")

print()
print("Exemple terminé!")
lcd.fill("#FFFFFF")
lcd.text("Calibration OK!", 10, 10, '#000000', font=medium)
lcd.text("Example termine", 10, 35, '#0000AA', font=medium)
lcd.update()

# Rien à faire en boucle
while True:
    utime.sleep_ms(1000)
