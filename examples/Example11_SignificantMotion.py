"""Example11_SignificantMotion.py -- Port MicroPython de Example11_SignificantMotion.ino

Détecte les mouvements significatifs (significant motion) du BMI270 via une
interruption GPIO sur la broche INT1 du capteur.

Le mouvement significatif est défini comme un déplacement soutenu (marche,
vélo, transport…) distinct d'un simple choc ponctuel. Il ne se déclenche
qu'une fois par période d'activité continue.

Matériel :
    Arduino NESSO / ESP32-C6 avec MicroPython
    BMI270 connecté en I2C (SCL=GPIO8, SDA=GPIO10)
    Broche INT1 du BMI270 → GPIO 3

Usage :
    Copier ce fichier et les modules bmi270 sur la carte, puis exécuter :
        import Example11_SignificantMotion
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
    BMI2_SIG_MOTION_INT,
    BMI270_SIG_MOT_STATUS_MASK,
)
from bmi2_defs import (
    BMI2_OK,
    BMI2_SIG_MOTION,
    BMI2_INT1,
    BMI2_INT_NON_LATCH,
    BMI2_INT_ACTIVE_HIGH,
    BMI2_INT_PUSH_PULL,
    BMI2_INT_OUTPUT_ENABLE,
    BMI2_INT_INPUT_DISABLE,
    Bmi2IntPinConfig,
)

# ---------------------------------------------------------------------------
# Broche d'interruption (INT1 du BMI270 → GPIO 3 sur Arduino NESSO)
# ---------------------------------------------------------------------------
INTERRUPT_PIN = 3

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
lcd.text("BMI270 connecte!", 10, 10, '#000000', font=medium)
lcd.update()
print("BMI270 connecté!")

# Active la feature significant motion
rslt = imu.enableFeature(BMI2_SIG_MOTION)
if rslt != BMI2_OK:
    print("Erreur enableFeature(SIG_MOTION):", rslt)

# Mappe l'interruption significant motion sur INT1
rslt = imu.mapInterruptToPin(BMI2_SIG_MOTION_INT, BMI2_INT1)
if rslt != BMI2_OK:
    print("Erreur mapInterruptToPin(SIG_MOTION_INT):", rslt)

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

print("En attente de mouvement significatif... (INT sur GPIO{})".format(INTERRUPT_PIN))
lcd.fill("#FFFFFF")
lcd.text("En attente...", 10, 10, '#000000', font=medium)
lcd.text("Sig Motion", 10, 35, '#0000AA', font=medium)
lcd.update()

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

            if status & BMI270_SIG_MOT_STATUS_MASK:
                print("Mouvement significatif détecté!", end="\t")
                lcd.fill("#FF8800")
                lcd.text("SIG MOTION!", 10, 10, '#FFFFFF', font=medium)
                lcd.text("n°{}".format(counter), 10, 35, '#FFFFFF', font=medium)
                lcd.update()
            else:
                print("Interruption inconnue (status=0x{:04X})".format(status), end="\t")

        print()

    utime.sleep_ms(20)
