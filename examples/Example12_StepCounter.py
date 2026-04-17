"""Example12_StepCounter.py -- Port MicroPython de Example12_StepCounter.ino

Détecte et compte les pas (step detector + step counter) et l'activité de
marche (step activity) via une interruption GPIO sur la broche INT1 du BMI270.

Features activées :
    BMI2_STEP_DETECTOR  — détecte chaque pas (bit d'interruption)
    BMI2_STEP_COUNTER   — cumule le nombre total de pas
    BMI2_STEP_ACTIVITY  — classifie l'activité : Immobile / Marche / Course

Watermark : 1 → interruption toutes les 20 pas (N × 20 pas).

Comportement :
    - À chaque interruption step counter : affiche le nombre total de pas
    - À chaque interruption step activity : affiche l'activité courante ;
      si Immobile, remet le compteur à zéro

Matériel :
    Arduino NESSO / ESP32-C6 avec MicroPython
    BMI270 connecté en I2C (SCL=GPIO8, SDA=GPIO10)
    Broche INT1 du BMI270 → GPIO 3

Usage :
    Copier ce fichier et les modules bmi270 sur la carte, puis exécuter :
        import Example12_StepCounter
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
    BMI2_STEP_COUNTER_INT,
    BMI2_STEP_ACTIVITY_INT,
    BMI2_STEP_ACTIVITY_STILL,
    BMI2_STEP_ACTIVITY_WALKING,
    BMI2_STEP_ACTIVITY_RUNNING,
    BMI270_STEP_CNT_STATUS_MASK,
    BMI270_STEP_ACT_STATUS_MASK,
)
from bmi2_defs import (
    BMI2_OK,
    BMI2_STEP_DETECTOR,
    BMI2_STEP_COUNTER,
    BMI2_STEP_ACTIVITY,
    BMI2_INT1,
    BMI2_INT_NON_LATCH,
    BMI2_INT_ACTIVE_HIGH,
    BMI2_INT_PUSH_PULL,
    BMI2_INT_OUTPUT_ENABLE,
    BMI2_INT_INPUT_DISABLE,
    Bmi2IntPinConfig,
)

# ---------------------------------------------------------------------------
# Correspondance activité → (nom, couleur LCD)
# ---------------------------------------------------------------------------
ACTIVITY_INFO = {
    BMI2_STEP_ACTIVITY_STILL:   ("Immobile", "#AAAAAA"),
    BMI2_STEP_ACTIVITY_WALKING: ("Marche",   "#44BB00"),
    BMI2_STEP_ACTIVITY_RUNNING: ("Course",   "#FF4400"),
}

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

# Active les trois features liées au comptage de pas
rslt = imu.enableFeature(BMI2_STEP_DETECTOR)
if rslt != BMI2_OK:
    print("Erreur enableFeature(STEP_DETECTOR):", rslt)

rslt = imu.enableFeature(BMI2_STEP_COUNTER)
if rslt != BMI2_OK:
    print("Erreur enableFeature(STEP_COUNTER):", rslt)

rslt = imu.enableFeature(BMI2_STEP_ACTIVITY)
if rslt != BMI2_OK:
    print("Erreur enableFeature(STEP_ACTIVITY):", rslt)

# Watermark = 1 → interruption toutes les 20 pas
rslt = imu.setStepCountWatermark(1)
if rslt != BMI2_OK:
    print("Erreur setStepCountWatermark:", rslt)

# Mappe les interruptions step counter et step activity sur INT1
rslt = imu.mapInterruptToPin(BMI2_STEP_COUNTER_INT, BMI2_INT1)
if rslt != BMI2_OK:
    print("Erreur mapInterruptToPin(STEP_COUNTER_INT):", rslt)

rslt = imu.mapInterruptToPin(BMI2_STEP_ACTIVITY_INT, BMI2_INT1)
if rslt != BMI2_OK:
    print("Erreur mapInterruptToPin(STEP_ACTIVITY_INT):", rslt)

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

print("En attente de pas... (INT sur GPIO{})".format(INTERRUPT_PIN))
lcd.fill("#FFFFFF")
lcd.text("En attente...", 10, 10, '#000000', font=medium)
lcd.text("Step Counter", 10, 35, '#0000AA', font=medium)
lcd.update()

# ---------------------------------------------------------------------------
# Boucle principale
# ---------------------------------------------------------------------------

int_counter = 0
while True:
    if interrupt_occurred:
        interrupt_occurred = False

        rslt, status = imu.getInterruptStatus()
        if rslt != BMI2_OK:
            print("Erreur getInterruptStatus:", rslt)
        else:
            int_counter += 1
            print("Interruption n°{}:".format(int_counter), end="\t")

            # -- Compteur de pas --
            if status & BMI270_STEP_CNT_STATUS_MASK:
                rslt_c, count = imu.getStepCount()
                print("Pas: {}".format(count), end="\t")
                lcd.fill("#E0F0FF")
                lcd.text("Pas: {}".format(count), 10, 10, '#000055', font=medium)
                lcd.update()

            # -- Activité de marche --
            if status & BMI270_STEP_ACT_STATUS_MASK:
                rslt_a, activity = imu.getStepActivity()
                name, color = ACTIVITY_INFO.get(activity, ("?", "#888888"))
                print("Activite: {}".format(name), end="\t")
                lcd.fill(color)
                lcd.text(name, 10, 10, '#000000', font=medium)
                lcd.update()

                # Si immobile, remet le compteur à zéro
                if activity == BMI2_STEP_ACTIVITY_STILL:
                    imu.resetStepCount()
                    print("Compteur remis à zéro", end="\t")

            if not (status & (BMI270_STEP_CNT_STATUS_MASK | BMI270_STEP_ACT_STATUS_MASK)):
                print("Interruption inconnue (status=0x{:04X})".format(status), end="\t")

        print()

    utime.sleep_ms(20)
