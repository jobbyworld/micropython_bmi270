"""Example13_WristGestures.py -- Port MicroPython de Example13_WristGestures.ino

Détecte les gestes de poignet (wrist gesture) et le réveil par orientation
(wrist wear wake-up) via une interruption GPIO sur la broche INT1 du BMI270.

Gestes reconnus :
    ARM_DOWN     - bras abaissé rapidement, puis ramené en position montre
    ARM_UP       - bras levé rapidement, puis ramené en position montre
    SHAKE_JIGGLE - secousse rapide du poignet
    FLICK_IN     - rotation lente vers l'extérieur, puis rapide vers l'utilisateur
    FLICK_OUT    - rotation rapide vers l'extérieur, puis lente vers l'utilisateur

Le capteur suppose une orientation : +Y vers 12h, +X vers 3h.

Matériel :
    Arduino NESSO / ESP32-C6 avec MicroPython
    BMI270 connecté en I2C (SCL=GPIO8, SDA=GPIO10)
    Broche INT1 du BMI270 → GPIO 3

Usage :
    Copier ce fichier et les modules bmi270 sur la carte, puis exécuter :
        import Example13_WristGestures
"""

from machine import I2C, Pin
import utime
from expander import ExpanderPin, Expander
from st7789 import ST7789
import NotoSansMonoRegular_16 as medium

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
    BMI2_WRIST_GESTURE_INT,
    BMI2_WRIST_WEAR_WAKE_UP_INT,
    BMI2_WRIST_GESTURE_UNKNOWN,
    BMI2_WRIST_GESTURE_ARM_DOWN,
    BMI2_WRIST_GESTURE_ARM_UP,
    BMI2_WRIST_GESTURE_SHAKE_JIGGLE,
    BMI2_WRIST_GESTURE_FLICK_IN,
    BMI2_WRIST_GESTURE_FLICK_OUT,
)
from bmi270 import (
    BMI270_WRIST_GEST_STATUS_MASK,
    BMI270_WRIST_WAKE_UP_STATUS_MASK,
)
from bmi2_defs import (
    BMI2_OK,
    BMI2_WRIST_GESTURE,
    BMI2_WRIST_WEAR_WAKE_UP,
    BMI2_INT1,
    BMI2_INT_NON_LATCH,
    BMI2_INT_ACTIVE_HIGH,
    BMI2_INT_PUSH_PULL,
    BMI2_INT_OUTPUT_ENABLE,
    BMI2_INT_INPUT_DISABLE,
    Bmi2IntPinConfig,
)

# ---------------------------------------------------------------------------
# Correspondances geste → (nom, couleur LCD)
# ---------------------------------------------------------------------------
GESTURE_INFO = {
    BMI2_WRIST_GESTURE_UNKNOWN:      ("Inconnu",     "#AAAAAA"),
    BMI2_WRIST_GESTURE_ARM_DOWN:     ("Bras bas",    "#FF4400"),
    BMI2_WRIST_GESTURE_ARM_UP:       ("Bras haut",   "#0044FF"),
    BMI2_WRIST_GESTURE_SHAKE_JIGGLE: ("Secousse",    "#FF00FF"),
    BMI2_WRIST_GESTURE_FLICK_IN:     ("Flick entré", "#00CC44"),
    BMI2_WRIST_GESTURE_FLICK_OUT:    ("Flick sorti", "#FFCC00"),
}

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
lcd.text("BMI270 connecté!", 10, 10, '#000000', font=medium)
lcd.update()
print("BMI270 connecté!")

# Active les features wrist gesture et wrist wear wake-up
rslt = imu.enableFeature(BMI2_WRIST_WEAR_WAKE_UP)
if rslt != BMI2_OK:
    print("Erreur enableFeature(WRIST_WEAR_WAKE_UP):", rslt)

rslt = imu.enableFeature(BMI2_WRIST_GESTURE)
if rslt != BMI2_OK:
    print("Erreur enableFeature(WRIST_GESTURE):", rslt)

# Mappe les deux interruptions sur INT1
rslt = imu.mapInterruptToPin(BMI2_WRIST_WEAR_WAKE_UP_INT, BMI2_INT1)
if rslt != BMI2_OK:
    print("Erreur mapInterruptToPin(WRIST_WEAR_WAKE_UP_INT):", rslt)

rslt = imu.mapInterruptToPin(BMI2_WRIST_GESTURE_INT, BMI2_INT1)
if rslt != BMI2_OK:
    print("Erreur mapInterruptToPin(WRIST_GESTURE_INT):", rslt)

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

print("En attente de gestes de poignet... (INT sur GPIO{})".format(INTERRUPT_PIN))
lcd.fill("#FFFFFF")
lcd.text("En +-attente...", 10, 10, '#000000', font=medium)
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

            if status & BMI270_WRIST_WAKE_UP_STATUS_MASK:
                print("Réveil poignet!", end="\t")
                lcd.fill("#00AAFF")
                lcd.text("Wake up!", 10, 10, '#FFFFFF', font=medium)
                lcd.update()

            if status & BMI270_WRIST_GEST_STATUS_MASK:
                rslt_g, gesture = imu.getWristGesture()
                name, color = GESTURE_INFO.get(gesture, ("?", "#888888"))
                print("Geste: {}".format(name), end="\t")
                lcd.fill(color)
                lcd.text(name, 10, 10, '#000000', font=medium)
                lcd.update()

            if not (status & (BMI270_WRIST_WAKE_UP_STATUS_MASK | BMI270_WRIST_GEST_STATUS_MASK)):
                print("Interruption inconnue (status=0x{:04X})".format(status), end="\t")

        print()

    utime.sleep_ms(20)
