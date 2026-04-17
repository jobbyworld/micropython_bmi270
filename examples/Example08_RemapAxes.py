"""Example08_RemapAxes.py -- Port MicroPython de Example08_RemapAxes.ino

Reconfigure le mapping des axes X/Y/Z du BMI270 afin d'adapter l'orientation
du capteur à celle souhaitée pour le projet.

Exemple par défaut :
    axes.x = BMI2_AXIS_POS_Y   → ce qui était Y devient X
    axes.y = BMI2_AXIS_NEG_Z   → ce qui était -Z devient Y
    axes.z = BMI2_AXIS_POS_X   → ce qui était X devient Z

Matériel :
    Arduino NESSO / ESP32-C6 avec MicroPython
    BMI270 connecté en I2C (SCL=GPIO8, SDA=GPIO10)

Usage :
    Copier ce fichier et les modules bmi270 sur la carte, puis exécuter :
        import Example08_RemapAxes
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
    BMI2_AXIS_POS_X,
    BMI2_AXIS_POS_Y,
    BMI2_AXIS_POS_Z,
    BMI2_AXIS_NEG_X,
    BMI2_AXIS_NEG_Y,
    BMI2_AXIS_NEG_Z,
)
from bmi2_defs import BMI2_OK, Bmi2Remap

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
# Remapping des axes
# Chaque axe ne peut être sélectionné qu'une seule fois.
# ---------------------------------------------------------------------------
axes = Bmi2Remap()
axes.x = BMI2_AXIS_POS_Y   # X physique ← ancien Y
axes.y = BMI2_AXIS_NEG_Z   # Y physique ← ancien -Z
axes.z = BMI2_AXIS_POS_X   # Z physique ← ancien X

rslt = imu.remapAxes(axes)
if rslt != BMI2_OK:
    print("Erreur remapAxes:", rslt)
else:
    print("Axes remappés : X→+Y, Y→-Z, Z→+X")

# ---------------------------------------------------------------------------
# Boucle principale — lecture à 50 Hz
# ---------------------------------------------------------------------------
row_acc  = 10
row_gyr  = 80

while True:
    imu.getSensorData()

    # Affichage série
    print(
        "Acc (g)"
        "\tX: {:.3f}\tY: {:.3f}\tZ: {:.3f}".format(
            imu.data.accelX, imu.data.accelY, imu.data.accelZ),
        end="\t"
    )
    print(
        "Gyr (°/s)"
        "\tX: {:.3f}\tY: {:.3f}\tZ: {:.3f}".format(
            imu.data.gyroX, imu.data.gyroY, imu.data.gyroZ)
    )

    # Affichage LCD
    lcd.fill("#FFFFFF")
    lcd.text("Acc (g)", 10, row_acc - 10, '#000000', font=medium)
    lcd.text("X:{:6.3f}".format(imu.data.accelX), 10, row_acc + 10, '#FF0000', font=medium)
    lcd.text("Y:{:6.3f}".format(imu.data.accelY), 10, row_acc + 30, '#00AA00', font=medium)
    lcd.text("Z:{:6.3f}".format(imu.data.accelZ), 10, row_acc + 50, '#0000FF', font=medium)
    lcd.text("Gyr (d/s)", 10, row_gyr + 20, '#000000', font=medium)
    lcd.text("X:{:7.2f}".format(imu.data.gyroX), 10, row_gyr + 40, '#FF6600', font=medium)
    lcd.text("Y:{:7.2f}".format(imu.data.gyroY), 10, row_gyr + 60, '#AA00FF', font=medium)
    lcd.text("Z:{:7.2f}".format(imu.data.gyroZ), 10, row_gyr + 80, '#00AAFF', font=medium)
    lcd.update()

    utime.sleep_ms(20)
