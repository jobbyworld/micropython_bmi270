"""Example14_AuxI2C.py -- Port MicroPython de Example14_AuxI2C.ino

Démontre l'utilisation de l'interface I2C auxiliaire du BMI270 pour lire
un capteur externe (ex: BMP581 pression/température).

IMPORTANT — Limitations du portage MicroPython :
    L'interface AUX I2C du BMI270 nécessite le driver bmi2_aux (bmi2_read_aux_man_mode /
    bmi2_write_aux_man_mode) qui effectue des transactions via des registres spéciaux
    du BMI270 (AUX_IF_CONF, AUX_RD_ADDR, AUX_WR_ADDR, DATA_8...). Cette implémentation
    montre la structure de l'exemple mais les méthodes readAux/writeAux ne sont pas
    complètement fonctionnelles dans ce portage.

    Pour une utilisation réelle du capteur auxiliaire :
    - Soit implémenter bmi2_read_aux_man_mode et bmi2_write_aux_man_mode dans bmi2.py
      en suivant le driver Bosch bmi2.c (sections "auxiliary interface").
    - Soit connecter le BMP581 directement au bus I2C principal de l'ESP32-C6.

Matériel :
    Arduino NESSO / ESP32-C6 avec MicroPython
    BMI270 connecté en I2C (SCL=GPIO8, SDA=GPIO10)
    BMP581 (ou autre capteur) connecté à l'interface AUX du BMI270

Usage :
    Copier ce fichier et les modules bmi270 sur la carte, puis exécuter :
        import Example14_AuxI2C
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
    BMI2_AUX,
    BMI2_ENABLE,
    BMI2_DISABLE,
    BMI2_AUX_RD_BURST_FRM_LEN_1,
    BMI2_AUX_RD_BURST_FRM_LEN_6,
    BMI2_AUX_ODR_1_56HZ,
    BMI2_ASDA_PUPSEL_2K,
    Bmi2SensConfig,
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
lcd.text("BMI270 connecte!", 10, 10, '#000000', font=medium)
lcd.update()
print("BMI270 connecté!")

# ---------------------------------------------------------------------------
# Configuration de l'interface AUX (BMP581 @ 0x47)
# ---------------------------------------------------------------------------

# Active le capteur AUX
imu.enableFeature(BMI2_AUX)

# Configure l'interface AUX en mode manuel avec le BMP581
auxConfig = Bmi2SensConfig()
auxConfig.type                      = BMI2_AUX
auxConfig.cfg.aux.aux_en            = BMI2_ENABLE
auxConfig.cfg.aux.manual_en         = BMI2_ENABLE
auxConfig.cfg.aux.man_rd_burst      = BMI2_AUX_RD_BURST_FRM_LEN_1
auxConfig.cfg.aux.aux_rd_burst      = BMI2_AUX_RD_BURST_FRM_LEN_6
auxConfig.cfg.aux.odr               = BMI2_AUX_ODR_1_56HZ
auxConfig.cfg.aux.i2c_device_addr   = 0x47   # adresse BMP581
auxConfig.cfg.aux.read_addr         = 0x1D   # premier registre data BMP581
auxConfig.cfg.aux.fcu_write_en      = BMI2_DISABLE
auxConfig.cfg.aux.offset            = 0

rslt = imu.setConfig(auxConfig)
if rslt != BMI2_OK:
    print("Erreur setConfig(AUX):", rslt)

# Active les pull-ups sur les broches AUX
imu.setAuxPullUps(BMI2_ASDA_PUPSEL_2K)

# ---------------------------------------------------------------------------
# Lecture du chip ID du BMP581 (registre 0x01, devrait être 0x50)
# ---------------------------------------------------------------------------

# Note : readAux n'est pas entièrement implémenté dans ce portage.
# Dans une implémentation complète, on lirait via bmi2_read_aux_man_mode.
print()
print("Note : interface AUX I2C non entièrement implémentée en MicroPython.")
print("Pour une utilisation réelle, connectez le BMP581 directement à l'I2C ESP32-C6.")
print()

lcd.fill("#FFEECC")
lcd.text("AUX I2C", 10, 10, '#664400', font=medium)
lcd.text("Non supporte", 10, 35, '#AA0000', font=medium)
lcd.text("voir commentaire", 10, 55, '#664400', font=medium)
lcd.text("dans le code", 10, 72, '#664400', font=medium)
lcd.update()

# ---------------------------------------------------------------------------
# Suggestion : lecture directe du BMP581 via l'I2C principal de l'ESP32-C6
# ---------------------------------------------------------------------------

BMP581_ADDR = 0x47

try:
    bmp_id = i2c.readfrom_mem(BMP581_ADDR, 0x01, 1)[0]
    print("BMP581 chip ID (via I2C principal): 0x{:02X} (attendu: 0x50)".format(bmp_id))

    lcd.fill("#E0F0FF")
    lcd.text("BMP581 direct", 10, 10, '#000055', font=medium)
    lcd.text("ID: 0x{:02X}".format(bmp_id), 10, 35, '#000000', font=medium)
    if bmp_id == 0x50:
        lcd.text("OK!", 10, 55, '#00AA00', font=medium)
    else:
        lcd.text("ID inattendu", 10, 55, '#AA0000', font=medium)
    lcd.update()
except Exception as e:
    print("BMP581 non détecté sur I2C:", e)

# ---------------------------------------------------------------------------
# Boucle principale — affiche accel/gyro du BMI270
# ---------------------------------------------------------------------------

print()
print("Lecture BMI270 en cours (AUX = stub)...")
counter = 0
while True:
    imu.getSensorData()
    counter += 1
    print("#{} Acc(g) X:{:.3f} Y:{:.3f} Z:{:.3f}  Gyr(d/s) X:{:.2f} Y:{:.2f} Z:{:.2f}".format(
        counter,
        imu.data.accelX, imu.data.accelY, imu.data.accelZ,
        imu.data.gyroX, imu.data.gyroY, imu.data.gyroZ))

    utime.sleep_ms(1000)
