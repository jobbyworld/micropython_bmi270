"""Example02_BasicReadingsSPI.py -- Port MicroPython de Example02_BasicReadingsSPI.ino

Lit les données d'accéléromètre et de gyroscope du BMI270 via l'interface SPI.

Matériel :
    Arduino NESSO / ESP32-C6 avec MicroPython
    BMI270 connecté en SPI
    Adapter les numéros de broches ci-dessous à votre câblage.

Brochage SPI (à adapter) :
    CS   → GPIO 5
    SCK  → GPIO 6   (ou selon bus SPI de la carte)
    MOSI → GPIO 7
    MISO → GPIO 2

Usage :
    Copier ce fichier et les modules bmi270 sur la carte, puis exécuter :
        import Example02_BasicReadingsSPI
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
lcd.text("Init SPI IMU...", 10, 10, '#000000', font=medium)
lcd.update()
expander.LCD_BACKLIGHT.write(1)

from SparkFun_BMI270_Arduino import BMI270

# ---------------------------------------------------------------------------
# Paramètres SPI — adapter selon le câblage
# ---------------------------------------------------------------------------
CS_PIN   = 5     # Chip Select
SPI_ID   = 1     # Bus SPI (1 sur ESP32-C6)
SPI_FREQ = 100_000

imu = BMI270()   # pas d'I2C passé : on utilisera SPI

print("Connexion au BMI270 en SPI...")
while imu.beginSPI(cs_pin=CS_PIN, freq=SPI_FREQ, spi_id=SPI_ID) != 0:
    print("BMI270 non détecté, vérification du câblage...")
    utime.sleep_ms(1000)

lcd.fill("#FFFFFF")
lcd.text("SPI Connected!", 10, 10, '#000000', font=medium)
lcd.update()
print("BMI270 connecté en SPI!")

# ---------------------------------------------------------------------------
# Boucle principale — lecture à ~50 Hz
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
    print("#{} Acc(g) X:{:.3f} Y:{:.3f} Z:{:.3f}  Gyr(°/s) X:{:.2f} Y:{:.2f} Z:{:.2f}".format(
        counter, ax, ay, az, gx, gy, gz))

    lcd.fill("#F5F5F5")
    lcd.text("SPI #{:04d}".format(counter), 10, 5, '#000055', font=medium)
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
