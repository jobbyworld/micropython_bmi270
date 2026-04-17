"""Example07_SelfTest.py -- Port MicroPython de Example07_SelfTest.ino

Effectue le self-test matériel de l'accéléromètre du BMI270 :
  1. Configure l'accéléromètre en mode self-test (ODR 1600 Hz, plage ±16g)
  2. Mesure les valeurs avec polarité positive puis négative
  3. Vérifie que les différences dépassent les seuils minimaux du datasheet :
       X : différence > 16 000 mg (~1g)
       Y : différence < -15 000 mg (axe inversé par convention hardware)
       Z : différence > 10 000 mg
  4. Remet le capteur à zéro après le test

Codes de retour :
    0  = BMI2_OK   → self-test réussi
   -16 = BMI2_E_SELF_TEST_FAIL → self-test échoué

Matériel :
    Arduino NESSO / ESP32-C6 avec MicroPython
    BMI270 connecté en I2C (SCL=GPIO8, SDA=GPIO10)

Usage :
    Copier ce fichier et les modules bmi270 sur la carte, puis exécuter :
        import Example07_SelfTest
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
from bmi2_defs import BMI2_OK,format_error_code

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

# ---------------------------------------------------------------------------
# Self-test (exécuté une seule fois, ~120 ms)
# ---------------------------------------------------------------------------
lcd.fill("#FFFFCC")
lcd.text("Self-test...", 10, 10, '#555500', font=medium)
lcd.update()
print("Démarrage du self-test accéléromètre...")

rslt = imu.selfTest()

if rslt == BMI2_OK:
    print("Self-test RÉUSSI!")
    lcd.fill("#00FF88")
    lcd.text("SELF-TEST OK", 10, 10, '#004400', font=medium)
    lcd.text("Accel: PASS", 10, 35, '#006600', font=medium)
else:
    print("Self-test ÉCHOUÉ! Code d'erreur:", rslt)
    lcd.fill("#FF4444")
    lcd.text("SELF-TEST FAIL", 10, 10, '#440000', font=medium)
    lcd.text("Err: {}".format(format_error_code(rslt)), 10, 35, '#660000', font=medium)

lcd.update()

# Le capteur est réinitialisé après le self-test (soft reset dans selfTest())
# Rien d'autre à faire — boucle principale vide
while True:
    utime.sleep_ms(1000)
