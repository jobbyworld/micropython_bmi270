"""Example10_MotionDetection.py -- Port MicroPython de Example10_MotionDetection.ino

Détecte les mouvements (any-motion) et les périodes sans mouvement (no-motion)
via une interruption GPIO sur la broche INT1 du BMI270.

Matériel :
    Arduino NESSO / ESP32-C6 avec MicroPython
    BMI270 connecté en I2C (SCL=GPIO8, SDA=GPIO10)
    Broche INT1 du BMI270 → GPIO 3

Usage :
    Copier ce fichier et les modules bmi270 sur la carte, puis exécuter :
        import Example10_MotionDetection
"""


from machine import I2C, Pin
import utime
from expander import ExpanderPin,Expander
from st7789 import ST7789
import NotoSans_32 as medium


i2c = I2C(0, scl=Pin(8), sda=Pin(10), freq=400_000)
expander = Expander(i2c)
lcd = ST7789(rst = expander.LCD_RESET)
lcd.set_rotation(1)
lcd.fill("#FFFFFF")
lcd.text("Init imu...", 10, 10, '#000000',font=medium)
lcd.update()
expander.LCD_BACKLIGHT.write(1)
# import sys
# #sys.exit(0)  # STOP - remove this line to run the example
# from ft6x36 import FT6X36
# from buzzer import Buzzer

from SparkFun_BMI270_Arduino import (
    BMI270,
    BMI2_FEATURE_DATA_OFFSET,
    BMI2_ANY_MOTION_INT,
    BMI2_NO_MOTION_INT,
    BMI270_ANY_MOT_STATUS_MASK,
    BMI270_NO_MOT_STATUS_MASK,
    BMI2_INT1,
)
from bmi2_defs import (
    BMI2_OK,
    BMI2_ENABLE,
    BMI2_ANY_MOTION,
    BMI2_NO_MOTION,
    BMI2_INT_NON_LATCH,
    BMI2_INT_ACTIVE_LOW,
    BMI2_INT_PUSH_PULL,
    BMI2_INT_OUTPUT_ENABLE,
    BMI2_INT_INPUT_DISABLE,
    Bmi2IntPinConfig,
    Bmi2SensConfig,
)

# Enum pour les axes de mouvement
class Axis:
    """Énumération des axes de mouvement (+ et -)"""
    X_POS = '+X'
    X_NEG = '-X'
    Y_POS = '+Y'
    Y_NEG = '-Y'
    Z_POS = '+Z'
    Z_NEG = '-Z'

# Broche d'interruption (INT1 du BMI270 → GPIO 3 sur Arduino NESSO)
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
lcd.text("Connected", 10, 10, '#000000',font=medium)
lcd.update()
print("BMI270 connecté!")

# Active any-motion et no-motion
rslt = imu.enableFeature(BMI2_ANY_MOTION)
if rslt != BMI2_OK:
    print("Erreur enableFeature(ANY_MOTION):", rslt)

rslt = imu.enableFeature(BMI2_NO_MOTION)
if rslt != BMI2_OK:
    print("Erreur enableFeature(NO_MOTION):", rslt)

# Configure any-motion : 100 ms (duration=5), seuil 83 mg (threshold=170), axes XYZ
# duration  : en unités de 20 ms → 5 × 20 ms = 100 ms
# threshold : en unités de 0.4883 mg → 170 × 0.4883 mg ≈ 83 mg
anyMotionConfig = Bmi2SensConfig()
anyMotionConfig.type = BMI2_ANY_MOTION
anyMotionConfig.cfg.any_motion.duration  = 5
anyMotionConfig.cfg.any_motion.threshold = 170
anyMotionConfig.cfg.any_motion.select_x  = BMI2_ENABLE
anyMotionConfig.cfg.any_motion.select_y  = BMI2_ENABLE
anyMotionConfig.cfg.any_motion.select_z  = BMI2_ENABLE


# rslt = imu.setConfig(anyMotionConfig)
# if rslt != BMI2_OK:
#     print("Erreur setConfig(ANY_MOTION):", rslt)

#  Set the "no motion" config. Default values:
# 
# .duration  - 5 = 100ms
# .threshold - 144 = 70mg
# .select_x  - Enabled
# .select_y  - Enabled
# .select_z  - Enabled
noMotionConfig = Bmi2SensConfig()
noMotionConfig.type = BMI2_NO_MOTION
noMotionConfig.cfg.no_motion.duration  = 3
noMotionConfig.cfg.no_motion.threshold = 500
noMotionConfig.cfg.no_motion.select_x  = BMI2_ENABLE
noMotionConfig.cfg.no_motion.select_y  = BMI2_ENABLE
noMotionConfig.cfg.no_motion.select_z  = BMI2_ENABLE

rslt = imu.setConfig(noMotionConfig)
if rslt != BMI2_OK:
    print("Erreur setConfig(NO_MOTION):", rslt)



# Mappe les deux interruptions sur INT1
rslt = imu.mapInterruptToPin(BMI2_ANY_MOTION_INT, BMI2_INT1)
if rslt != BMI2_OK:
    print("Erreur mapInterruptToPin(ANY_MOTION_INT):", rslt)

rslt = imu.mapInterruptToPin(BMI2_NO_MOTION_INT, BMI2_INT1)
if rslt != BMI2_OK:
    print("Erreur mapInterruptToPin(NO_MOTION_INT):", rslt)

# Configure la broche INT1 : active low, push-pull, pulsé (non-latch)
intPinConfig = Bmi2IntPinConfig()
intPinConfig.pin_type = BMI2_INT1
intPinConfig.int_latch = BMI2_INT_NON_LATCH
intPinConfig.pin_cfg[0].lvl       = BMI2_INT_ACTIVE_LOW
intPinConfig.pin_cfg[0].od        = BMI2_INT_PUSH_PULL
intPinConfig.pin_cfg[0].output_en = BMI2_INT_OUTPUT_ENABLE
intPinConfig.pin_cfg[0].input_en  = BMI2_INT_INPUT_DISABLE

rslt = imu.setInterruptPinConfig(intPinConfig)
if rslt != BMI2_OK:
    print("Erreur setInterruptPinConfig:", rslt)

# Configure la broche GPIO en entrée et attache l'ISR (front descendant = active low)
int_pin = Pin(INTERRUPT_PIN, Pin.IN)
int_pin.irq(trigger=Pin.IRQ_FALLING, handler=interrupt_handler)

print("En attente d'interruptions de mouvement... (INT sur GPIO{})".format(INTERRUPT_PIN))

# ---------------------------------------------------------------------------
# Boucle principale
# ---------------------------------------------------------------------------


def get_major_axe(imu):
    # Détermine l'axe majeur du mouvement (celui avec la plus grande accélération)
    imu.getSensorData()
    ax = imu.data.accelX
    ay = imu.data.accelY
    az = imu.data.accelZ
    max_val = max(abs(ax), abs(ay), abs(az))
    if max_val == abs(ax):
        return Axis.X_POS if ax > 0 else Axis.X_NEG
    elif max_val == abs(ay):
        return Axis.Y_POS if ay > 0 else Axis.Y_NEG
    elif max_val == abs(az):
        return Axis.Z_POS if az > 0 else Axis.Z_NEG
axe_colors = {
    Axis.X_POS: '#FF0000',
    Axis.X_NEG: "#F67700",
    Axis.Y_POS: '#00FF00',
    Axis.Y_NEG: "#C3FF00",
    Axis.Z_POS: '#0000FF',
    Axis.Z_NEG: "#00FBEB"
}

counter = 0
while True:
    if interrupt_occurred:
        interrupt_occurred = False
        # print("Interruption détectée!", end="\t")

        rslt, status = imu.getInterruptStatus()
        if rslt != BMI2_OK:
            print("Erreur getInterruptStatus:", rslt)
        else:
            major_axe = get_major_axe(imu)
            if major_axe in (Axis.X_POS):
                lcd.set_rotation(0)
            elif major_axe in (Axis.Y_NEG):
                lcd.set_rotation(1)
            elif major_axe in (Axis.X_NEG):
                lcd.set_rotation(2)
            elif major_axe in (Axis.Y_POS):
                lcd.set_rotation(3)
            else:
                #Axis.Z_POS, Axis.Z_NEG,
                pass


              
            lcd.fill(axe_colors[major_axe])
            lcd.text(major_axe, 10, 10, '#000000',font=medium)
            lcd.update()
            counter += 1
            print("Interruption n°{}:".format(counter), end="\t")
            print("Axe majeur:", major_axe, end="\t")
            if status & BMI270_ANY_MOT_STATUS_MASK:
                print("Mouvement détecté!", end="\t")
                
            if status & BMI270_NO_MOT_STATUS_MASK:
                print("Absence de mouvement!", end="\t")
            if not (status & (BMI270_ANY_MOT_STATUS_MASK | BMI270_NO_MOT_STATUS_MASK)):
                print("Interruption inconnue (status=0x{:04X})".format(status), end="\t")
        print()

    utime.sleep_ms(20)
