"""Example03_Interrupts.py -- Port MicroPython de Example03_Interrupts.ino

Détecte les données prêtes (DRDY) de l'accéléromètre via une interruption GPIO
sur la broche INT1 du BMI270 (active haute, front montant).

Matériel :
    Arduino NESSO / ESP32-C6 avec MicroPython
    BMI270 connecté en I2C (SCL=GPIO8, SDA=GPIO10)
    Broche INT1 du BMI270 → GPIO 3

Usage :
    Copier ce fichier et les modules bmi270 sur la carte, puis exécuter :
        import Example03_Interrupts
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
    BMI2_FEATURE_DATA_OFFSET,
)
from bmi2_defs import (
    BMI2_OK,
    BMI2_DRDY_INT,
    BMI2_INT1,
    BMI2_INT_NON_LATCH,
    BMI2_INT_ACTIVE_HIGH,
    BMI2_INT_PUSH_PULL,
    BMI2_INT_OUTPUT_ENABLE,
    BMI2_INT_INPUT_DISABLE,
    BMI2_GYRO,
    BMI2_ACC_ODR_25HZ,
    BMI2_GYR_ODR_25HZ,
    BMI2_POWER_OPT_MODE,
    BMI2_ACC_DRDY_INT_MASK,
    BMI2_GYR_DRDY_INT_MASK,
    Bmi2IntPinConfig,
)

# ---------------------------------------------------------------------------
# Broche d'interruption (INT1 du BMI270 → GPIO 3 sur Arduino NESSO)
# ---------------------------------------------------------------------------
INTERRUPT_PIN = 3

# Drapeau levé par l'ISR
interrupt_occurred = False


def interrupt_handler(pin):
    global interrupt_occurred
    interrupt_occurred = True


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

# Réduit l'ODR à 25 Hz pour les deux capteurs
# (mode puissance pour permettre les ODR bas sur l'accel)
imu.setAccelPowerMode(BMI2_POWER_OPT_MODE)
imu.setAccelODR(BMI2_ACC_ODR_25HZ)
imu.setGyroODR(BMI2_GYR_ODR_25HZ)

# Désactive le gyroscope : seul l'accéléromètre génère des interruptions DRDY
imu.disableFeature(BMI2_GYRO)

# Mappe l'interruption DRDY sur INT1
rslt = imu.mapInterruptToPin(BMI2_DRDY_INT, BMI2_INT1)
if rslt != BMI2_OK:
    print("Erreur mapInterruptToPin(DRDY):", rslt)

# Configure la broche INT1 : active haute, push-pull, pulsé (non-latch)
intPinConfig = Bmi2IntPinConfig()
intPinConfig.pin_type = BMI2_INT1
intPinConfig.int_latch = BMI2_INT_NON_LATCH
intPinConfig.pin_cfg[0].lvl       = BMI2_INT_ACTIVE_HIGH
intPinConfig.pin_cfg[0].od        = BMI2_INT_PUSH_PULL
intPinConfig.pin_cfg[0].output_en = BMI2_INT_OUTPUT_ENABLE
intPinConfig.pin_cfg[0].input_en  = BMI2_INT_INPUT_DISABLE

rslt = imu.setInterruptPinConfig(intPinConfig)
if rslt != BMI2_OK:
    print("Erreur setInterruptPinConfig:", rslt)

# Configure la broche GPIO en entrée et attache l'ISR (front montant = active high)
int_pin = Pin(INTERRUPT_PIN, Pin.IN)
int_pin.irq(trigger=Pin.IRQ_RISING, handler=interrupt_handler)

print("En attente d'interruptions DRDY... (INT sur GPIO{})".format(INTERRUPT_PIN))

# ---------------------------------------------------------------------------
# Boucle principale
# ---------------------------------------------------------------------------

counter = 0
while True:
    if interrupt_occurred:
        interrupt_occurred = False

        rslt, status = imu.getInterruptStatus()
        if rslt != BMI2_OK:
            print("Erreur getInterruptStatus:", rslt)
        else:
            counter += 1
            print("Interruption n°{}:".format(counter), end="\t")

            if status & BMI2_GYR_DRDY_INT_MASK:
                imu.getSensorData()
                print("Gyro prêt!", end="\t")
                print("X: {:.3f}  Y: {:.3f}  Z: {:.3f}".format(
                    imu.data.gyroX, imu.data.gyroY, imu.data.gyroZ), end="\t")

            if status & BMI2_ACC_DRDY_INT_MASK:
                imu.getSensorData()
                ax = imu.data.accelX
                ay = imu.data.accelY
                az = imu.data.accelZ
                print("Accel prêt!", end="\t")
                print("X: {:.3f}  Y: {:.3f}  Z: {:.3f}".format(ax, ay, az), end="\t")

                # Affiche les données sur le LCD
                lcd.fill("#FFFFFF")
                lcd.text("Accel DRDY", 10, 10, '#000000', font=medium)
                lcd.text("X:{:.2f}".format(ax), 10, 40, '#FF0000', font=medium)
                lcd.text("Y:{:.2f}".format(ay), 10, 80, '#00AA00', font=medium)
                lcd.text("Z:{:.2f}".format(az), 10, 120, '#0000FF', font=medium)
                lcd.update()

            if not (status & (BMI2_GYR_DRDY_INT_MASK | BMI2_ACC_DRDY_INT_MASK)):
                print("Interruption inconnue (status=0x{:04X})".format(status), end="\t")

        print()

    utime.sleep_ms(5)
