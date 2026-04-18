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




from SparkFun_BMI270_Arduino import (
    BMI270,
    BMI2_ANY_MOTION_INT,
    BMI2_NO_MOTION_INT,
    BMI270_ANY_MOT_STATUS_MASK,
    BMI270_NO_MOT_STATUS_MASK,
    BMI2_AXIS_POS_X,
    BMI2_AXIS_POS_Y,
    BMI2_AXIS_POS_Z,
    BMI2_AXIS_NEG_X,
    BMI2_AXIS_NEG_Y,
    BMI2_AXIS_NEG_Z,
)
from bmi2_defs import (
    BMI2_OK,
    BMI2_ENABLE,
    BMI2_ANY_MOTION,
    BMI2_NO_MOTION,

)

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
# Enum pour les axes de mouvement


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


rslt = imu.enableFeatures([BMI2_ANY_MOTION, BMI2_NO_MOTION])
if rslt != BMI2_OK:
    print("Erreur enableFeature(NO_MOTION):", rslt)


# rslt = imu.configureNoMotion(
#     duration=5,
#     threshold=300
# )
# print("configure NoMotion:", rslt)



rslt = imu.configureAnyMotion(
    duration=2,
    threshold=60,
    select_x=BMI2_ENABLE,
    select_y=BMI2_ENABLE,
    select_z=BMI2_ENABLE
)
print("configure AnyMotion:", rslt)



imu.configureInterrupts([
    BMI2_ANY_MOTION_INT,
    BMI2_NO_MOTION_INT
    ])



# Configure la broche GPIO en entrée et attache l'ISR (front descendant = active low)
int_pin = Pin(INTERRUPT_PIN, Pin.IN)
int_pin.irq(trigger=Pin.IRQ_FALLING, handler=interrupt_handler)

print("En attente d'interruptions de mouvement... (INT sur GPIO{})".format(INTERRUPT_PIN))

# ---------------------------------------------------------------------------
# Boucle principale
# ---------------------------------------------------------------------------



axes_repr = {
    BMI2_AXIS_POS_X: ('+X','#FF0000'),
    BMI2_AXIS_NEG_X: ("-X","#F67700"),
    BMI2_AXIS_POS_Y: ('+Y','#00FF00'),
    BMI2_AXIS_NEG_Y: ('-Y',"#C3FF00"),
    BMI2_AXIS_POS_Z: ('+Z','#0000FF'),
    BMI2_AXIS_NEG_Z: ('-Z',"#00FBEB")
}

counter = 0
countdown = 200
screen_on = True
last_axe = None
while True:
    if interrupt_occurred:
        interrupt_occurred = False
        
        if not screen_on:
            screen_on = True
            countdown = 200
            expander.LCD_BACKLIGHT.write(1)
        # print("Interruption détectée!", end="\t")

        rslt, status = imu.getInterruptStatus()
        if rslt != BMI2_OK:
            print("Erreur getInterruptStatus:", rslt)
        else:
            major_axe = imu.get_major_axis()
            counter += 1
            print("Interruption n°{}:".format(counter), end="\t")
        
           
            if status & BMI270_ANY_MOT_STATUS_MASK:
                print("Mouvement détecté!", end="\t")
                
            if status & BMI270_NO_MOT_STATUS_MASK:
                print("Absence de mouvement!", end="\t")
            
            # if status & BMI270_ANY_MOT_STATUS_MASK and status & BMI270_NO_MOT_STATUS_MASK:
            #     expander.LCD_BACKLIGHT.write(1)
            # if not status & BMI270_ANY_MOT_STATUS_MASK and status & BMI270_NO_MOT_STATUS_MASK:
            #     expander.LCD_BACKLIGHT.write(0)

            if not (status & (BMI270_ANY_MOT_STATUS_MASK | BMI270_NO_MOT_STATUS_MASK)):
                print("Interruption inconnue (status=0x{:04X})".format(status), end="\t")

            if major_axe != last_axe:
                last_axe = major_axe
                if major_axe in (BMI2_AXIS_POS_X,):
                    lcd.set_rotation(0)
                elif major_axe in (BMI2_AXIS_NEG_Y,):
                    lcd.set_rotation(1)
                elif major_axe in (BMI2_AXIS_NEG_X,):
                    lcd.set_rotation(2)
                elif major_axe in (BMI2_AXIS_POS_Y,):
                    lcd.set_rotation(3)
                else:
                    #Axis.Z_POS, Axis.Z_NEG,
                    pass
                axe_repr,axe_color = axes_repr[major_axe]
                lcd.fill(axe_color)
                lcd.text(axe_repr, 10, 10, '#000000',font=medium)
                lcd.update()
            print()
        utime.sleep_ms(20)
    else:
        if screen_on: 
            countdown -= 1
            if countdown == 0:
                expander.LCD_BACKLIGHT.write(0)
                screen_on = False
            utime.sleep_ms(20)
        else:
            utime.sleep_ms(200)


    
