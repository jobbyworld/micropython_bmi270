"""Example04_Filtering.py -- Port MicroPython de Example04_Filtering.ino

Démontre la configuration du filtrage de l'accéléromètre et du gyroscope :
  - Accéléromètre : ODR 50 Hz, BWP OSR4 (oversampling x4), mode perf, plage ±2g
  - Gyroscope     : ODR 50 Hz, BWP OSR4, mode perf, plage 125 °/s

Si la configuration est invalide, le capteur retourne un code d'erreur :
    BMI2_E_ACC_INVALID_CFG     (-5)
    BMI2_E_GYRO_INVALID_CFG    (-6)
    BMI2_E_ACC_GYR_INVALID_CFG (-7)

Matériel :
    Arduino NESSO / ESP32-C6 avec MicroPython
    BMI270 connecté en I2C (SCL=GPIO8, SDA=GPIO10)

Usage :
    Copier ce fichier et les modules bmi270 sur la carte, puis exécuter :
        import Example04_Filtering
"""

from machine import I2C, Pin
import utime
from expander import ExpanderPin, Expander
from st7789 import ST7789
import inconsolata_16 as medium

i2c = I2C(0, scl=Pin(8), sda=Pin(10), freq=400_000)
expander = Expander(i2c)
lcd = ST7789(rst=expander.LCD_RESET)
lcd.set_rotation(0)
lcd.fill("#FFFFFF")
lcd.text("Init imu...", 10, 10, '#000000', font=medium)
lcd.update()
expander.LCD_BACKLIGHT.write(1)

from SparkFun_BMI270_Arduino import BMI270
from bmi2_defs import (
    BMI2_OK,
    BMI2_ACCEL,
    BMI2_GYRO,
    BMI2_ACC_ODR_50HZ,
    BMI2_ACC_OSR4_AVG1,
    BMI2_PERF_OPT_MODE,
    BMI2_ACC_RANGE_2G,
    BMI2_GYR_ODR_50HZ,
    BMI2_GYR_OSR4_MODE,
    BMI2_GYR_OIS_250,
    BMI2_GYR_RANGE_125,
    BMI2_E_ACC_INVALID_CFG,
    BMI2_E_GYRO_INVALID_CFG,
    BMI2_E_ACC_GYR_INVALID_CFG,
    Bmi2SensConfig,
    Bmi2FormatError,
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

# ---------------------------------------------------------------------------
# Configuration du filtrage
# ---------------------------------------------------------------------------

# -- Accéléromètre --
cfg_acc = Bmi2SensConfig()
cfg_acc.type                = BMI2_ACCEL
cfg_acc.cfg.acc.odr         = BMI2_ACC_ODR_50HZ    # 50 Hz
cfg_acc.cfg.acc.bwp         = BMI2_ACC_OSR4_AVG1   # OSR4 (oversampling x4)
cfg_acc.cfg.acc.filter_perf = BMI2_PERF_OPT_MODE   # Mode haute performance
cfg_acc.cfg.acc.range       = BMI2_ACC_RANGE_2G     # ±2g

# -- Gyroscope --
cfg_gyr = Bmi2SensConfig()
cfg_gyr.type                = BMI2_GYRO
cfg_gyr.cfg.gyr.odr         = BMI2_GYR_ODR_50HZ    # 50 Hz
cfg_gyr.cfg.gyr.bwp         = BMI2_GYR_OSR4_MODE   # OSR4
cfg_gyr.cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE   # Mode haute performance
cfg_gyr.cfg.gyr.ois_range   = BMI2_GYR_OIS_250     # OIS range 250 °/s
cfg_gyr.cfg.gyr.range       = BMI2_GYR_RANGE_125   # ±125 °/s

rslt = imu.setConfigs([cfg_acc, cfg_gyr])

if rslt == BMI2_OK:
    print("Configuration filtrée appliquée avec succès!")
elif rslt == BMI2_E_ACC_INVALID_CFG:
    print("Erreur : configuration accéléromètre invalide!")
elif rslt == BMI2_E_GYRO_INVALID_CFG:
    print("Erreur : configuration gyroscope invalide!")
elif rslt == BMI2_E_ACC_GYR_INVALID_CFG:
    print("Erreur : configuration accel+gyro invalide!")
else:
    print("Erreur setConfig:", rslt)

lcd.fill("#FFFFFF")
lcd.text("Filtering", 10, 10, '#000000', font=medium)
lcd.text("ODR=50Hz OSR4", 10, 28, '#0000AA', font=medium)
lcd.update()

# ---------------------------------------------------------------------------
# Boucle principale — lecture à 50 Hz
# ---------------------------------------------------------------------------
counter = 0
while True:
    imu.getSensorData()

    ax = imu.data.accelX
    ay = imu.data.accelY
    az = imu.data.accelZ
    gx = imu.data.gyroX
    gy = imu.data.gyroY
    gz = imu.data.gyroZ

    counter += 1

    # Affichage série
    print("#{} Acc(g) X:{:.3f} Y:{:.3f} Z:{:.3f}  Gyr(°/s) X:{:.2f} Y:{:.2f} Z:{:.2f}".format(
        counter, ax, ay, az, gx, gy, gz))

    # Affichage LCD
    lcd.fill("#F5F5FF")
    lcd.text("Filter #{}".format(counter), 10, 5, '#000055', font=medium)
    lcd.text("Acc (g)", 10, 25, '#000000', font=medium)
    lcd.text("X:{:6.3f}".format(ax), 10, 42, '#FF0000', font=medium)
    lcd.text("Y:{:6.3f}".format(ay), 10, 58, '#00AA00', font=medium)
    lcd.text("Z:{:6.3f}".format(az), 10, 74, '#0000FF', font=medium)
    lcd.text("Gyr (d/s)", 10, 95, '#000000', font=medium)
    lcd.text("X:{:7.2f}".format(gx), 10, 112, '#FF6600', font=medium)
    lcd.text("Y:{:7.2f}".format(gy), 10, 128, '#AA00FF', font=medium)
    lcd.text("Z:{:7.2f}".format(gz), 10, 144, '#00AAFF', font=medium)
    lcd.update()

    utime.sleep_ms(20)
