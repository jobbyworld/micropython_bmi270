"""Example05_FIFOBuffer.py -- Port MicroPython de Example05_FIFOBuffer.ino

Démontre l'utilisation du buffer FIFO du BMI270 avec une interruption GPIO.

Configuration :
    - ODR accel et gyro réduits à 25 Hz (identiques pour le FIFO)
    - FIFO en mode headerless, accel + gyro activés
    - Watermark = 5 trames (interruption toutes les 5 mesures)
    - Sous-échantillonnage ÷16, filtrage activé, self-wake-up activé
    - Interruption FWM (FIFO WaterMark) sur INT1 → GPIO 3

Comportement :
    - Affiche la longueur courante du FIFO en continu
    - À chaque interruption watermark : lit les 5 trames et les affiche

Matériel :
    Arduino NESSO / ESP32-C6 avec MicroPython
    BMI270 connecté en I2C (SCL=GPIO8, SDA=GPIO10)
    Broche INT1 du BMI270 → GPIO 3

Usage :
    Copier ce fichier et les modules bmi270 sur la carte, puis exécuter :
        import Example05_FIFOBuffer
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
    BMI270_FIFOConfig,
    BMI2_FIFO_DOWN_SAMPLE_16,
)
from bmi2_defs import (
    BMI2_OK,
    BMI2_FIFO_ACC_EN,
    BMI2_FIFO_GYR_EN,
    BMI2_FWM_INT,
    BMI2_FWM_INT_STATUS_MASK,
    BMI2_INT1,
    BMI2_INT_NON_LATCH,
    BMI2_INT_ACTIVE_HIGH,
    BMI2_INT_PUSH_PULL,
    BMI2_INT_OUTPUT_ENABLE,
    BMI2_INT_INPUT_DISABLE,
    BMI2_ACC_ODR_25HZ,
    BMI2_GYR_ODR_25HZ,
    BMI2_ENABLE,
    Bmi2IntPinConfig,
)

# ---------------------------------------------------------------------------
# Broche d'interruption (INT1 du BMI270 → GPIO 3)
# ---------------------------------------------------------------------------
INTERRUPT_PIN = 3
NUM_SAMPLES   = 5   # watermark : 5 trames

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

# Réduit l'ODR à 25 Hz pour avoir le même taux sur accel et gyro
imu.setAccelODR(BMI2_ACC_ODR_25HZ)
imu.setGyroODR(BMI2_GYR_ODR_25HZ)

# Configure le FIFO
fifo_cfg = BMI270_FIFOConfig()
fifo_cfg.flags          = BMI2_FIFO_ACC_EN | BMI2_FIFO_GYR_EN
fifo_cfg.watermark      = NUM_SAMPLES
fifo_cfg.accelDownSample = BMI2_FIFO_DOWN_SAMPLE_16
fifo_cfg.gyroDownSample  = BMI2_FIFO_DOWN_SAMPLE_16
fifo_cfg.accelFilter    = BMI2_ENABLE
fifo_cfg.gyroFilter     = BMI2_ENABLE
fifo_cfg.selfWakeUp     = BMI2_ENABLE

rslt = imu.setFIFOConfig(fifo_cfg)
if rslt != BMI2_OK:
    print("Erreur setFIFOConfig:", rslt)

# Mappe l'interruption watermark FIFO sur INT1 (BMI2_FWM_INT est une data int, pas feature)
rslt = imu.mapInterruptToPin(BMI2_FWM_INT, BMI2_INT1)
if rslt != BMI2_OK:
    print("Erreur mapInterruptToPin(FWM):", rslt)

# Configure la broche INT1 : active haute, push-pull, pulsé
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

# Attache l'ISR sur front montant (active high)
int_pin = Pin(INTERRUPT_PIN, Pin.IN)
int_pin.irq(trigger=Pin.IRQ_RISING, handler=interrupt_handler)

print("En attente de données FIFO... (watermark={})".format(NUM_SAMPLES))
lcd.fill("#FFFFFF")
lcd.text("FIFO Buffer", 10, 10, '#000055', font=medium)
lcd.text("WM={}  ODR=25Hz".format(NUM_SAMPLES), 10, 28, '#0000AA', font=medium)
lcd.update()

# ---------------------------------------------------------------------------
# Boucle principale
# ---------------------------------------------------------------------------
prev_fifo_len = 0
int_counter   = 0

while True:
    # Affichage de la longueur courante du FIFO
    rslt, fifo_len = imu.getFIFOLength()
    if rslt == BMI2_OK and fifo_len != prev_fifo_len:
        prev_fifo_len = fifo_len
        print("FIFO length: {}/{}".format(fifo_len, NUM_SAMPLES))
        if fifo_len > NUM_SAMPLES:
            print("FIFO trop plein, vidage...")
            imu.flushFIFO()

    # Traitement de l'interruption watermark
    if interrupt_occurred:
        interrupt_occurred = False
        int_counter += 1
        print("Interruption n°{}!".format(int_counter))

        rslt, status = imu.getInterruptStatus()
        if rslt != BMI2_OK:
            print("Erreur getInterruptStatus:", rslt)
        elif status & BMI2_FWM_INT_STATUS_MASK:
            rslt, n_read, fifo_data = imu.getFIFOData(NUM_SAMPLES)
            if rslt != BMI2_OK:
                print("Erreur getFIFOData:", rslt)
            elif n_read != NUM_SAMPLES:
                print("Nombre de trames inattendu:", n_read)
            else:
                print("  {} trames lues:".format(n_read))
                for i, s in enumerate(fifo_data):
                    print("  [{}] Acc(g) X:{:.3f} Y:{:.3f} Z:{:.3f}"
                          "  Gyr(°/s) X:{:.2f} Y:{:.2f} Z:{:.2f}".format(
                              i, s.accelX, s.accelY, s.accelZ,
                              s.gyroX, s.gyroY, s.gyroZ))

                # Affiche la dernière trame sur le LCD
                s = fifo_data[-1]
                lcd.fill("#EEF5FF")
                lcd.text("FIFO #{:03d}".format(int_counter), 10, 5, '#000055', font=medium)
                lcd.text("Acc (g)", 10, 25, '#000000', font=medium)
                lcd.text("X:{:6.3f}".format(s.accelX), 10, 42, '#FF0000', font=medium)
                lcd.text("Y:{:6.3f}".format(s.accelY), 10, 58, '#00AA00', font=medium)
                lcd.text("Z:{:6.3f}".format(s.accelZ), 10, 74, '#0000FF', font=medium)
                lcd.text("Gyr (d/s)", 10, 95, '#000000', font=medium)
                lcd.text("X:{:7.2f}".format(s.gyroX), 10, 112, '#FF6600', font=medium)
                lcd.text("Y:{:7.2f}".format(s.gyroY), 10, 128, '#AA00FF', font=medium)
                lcd.text("Z:{:7.2f}".format(s.gyroZ), 10, 144, '#00AAFF', font=medium)
                lcd.update()
        else:
            print("Interruption inconnue (status=0x{:04X})".format(status))

    utime.sleep_ms(20)
