"""SparkFun_BMI270_Arduino.py -- Port MicroPython du wrapper SparkFun BMI270.

Équivalent de :
    SparkFun_BMI270_Arduino_Library.h / .cpp

Expose une classe `BMI270` de haut niveau pour lire facilement
l'accéléromètre et le gyroscope du BMI270 via I2C sur ESP32-C6
(Arduino NESSO, MicroPython).

Brochage par défaut (Arduino NESSO / ESP32-C6) :
    I2C SCL  → GPIO 8
    I2C SDA  → GPIO 10

Usage minimal :
    from SparkFun_BMI270_Arduino import BMI270

    imu = BMI270()
    while imu.beginI2C() != 0:
        print("BMI270 non detecte, verification cablage...")
        import utime; utime.sleep_ms(1000)

    print("BMI270 connecte!")
    while True:
        imu.getSensorData()
        print(imu.data.accelX, imu.data.accelY, imu.data.accelZ)
        import utime; utime.sleep_ms(20)
"""

import utime
from machine import I2C, Pin

from bmi2_defs import (
    BMI2_ANY_MOTION,
    BMI2_NO_MOTION,
    BMI2_OK,
    BMI2_E_INVALID_INPUT,
    BMI2_E_NULL_PTR,
    BMI2_E_SELF_TEST_FAIL,
    BMI2_I2C_PRIM_ADDR,
    BMI2_I2C_SEC_ADDR,
    BMI2_I2C_INTF,
    BMI2_SPI_INTF,
    BMI2_ENABLE,
    BMI2_DISABLE,
    BMI2_ACCEL,
    BMI2_GYRO,
    BMI2_WRIST_GESTURE,
    BMI2_WRIST_WEAR_WAKE_UP,
    BMI2_SIG_MOTION,
    BMI2_STEP_DETECTOR,
    BMI2_STEP_COUNTER,
    BMI2_STEP_ACTIVITY,
    BMI2_ACC_RANGE_2G,
    BMI2_ACC_RANGE_16G,
    BMI2_GYR_RANGE_125,
    BMI2_SENSORTIME_RESOLUTION,
    BMI2_DRDY_ACC,
    BMI2_DRDY_GYR,
    BMI2_INT1,
    BMI2_INT_NON_LATCH,
    BMI2_INT_ACTIVE_LOW,
    BMI2_INT_PUSH_PULL,
    BMI2_INT_OUTPUT_ENABLE,
    BMI2_INT_INPUT_DISABLE,
    BMI2_ACC_ODR_1600HZ,
    BMI2_ACC_NORMAL_AVG4,
    BMI2_PERF_OPT_MODE,
    BMI2_ACC_SELF_TEST_ADDR,
    BMI2_ACC_SELF_TEST_EN_MASK,
    BMI2_ACC_SELF_TEST_SIGN_MASK,
    BMI2_ACC_SELF_TEST_AMP_MASK,
    BMI2_ACC_SELF_TEST_SIGN_POS,
    BMI2_ACC_SELF_TEST_AMP_POS,
    BMI2_ACC_X_LSB_ADDR,
    BMI2_ACC_NUM_BYTES,
    BMI2_FIFO_ACC_EN,
    BMI2_FIFO_GYR_EN,
    BMI2_FIFO_HEADER_EN,
    BMI2_FIFO_FLUSH_CMD,
    BMI2_FWM_INT,
    BMI2_FWM_INT_STATUS_MASK,
    BMI2_NVM_CONF_ADDR,
    BMI2_NVM_PROG_CMD,
    BMI2_NVM_UNLOCK_ENABLE,
    BMI2_NVM_UNLOCK_DISABLE,
    BMI2_ACC_OFF_COMP_0_ADDR,
    BMI2_GYR_OFF_COMP_3_ADDR,
    BMI2_GYR_OFF_COMP_6_ADDR,
    BMI2_NV_ACC_OFFSET_MASK,
    BMI2_NV_ACC_OFFSET_POS,
    BMI2_GYR_OFF_COMP_EN_MASK,
    BMI2_GYR_OFF_COMP_MSB_X_MASK,
    BMI2_GYR_OFF_COMP_MSB_Y_MASK,
    BMI2_GYR_OFF_COMP_MSB_Y_POS,
    BMI2_GYR_OFF_COMP_MSB_Z_MASK,
    BMI2_GYR_OFF_COMP_MSB_Z_POS,
    BMI2_AUX_IF_TRIM,
    Bmi2AnyMotionConfig,
    Bmi2NoMotionConfig,
    Bmi2SensData,
    Bmi2SensConfig,
    Bmi2IntPinConfig,
    Bmi2Remap,
    bmi2_set_bits,
    bmi2_set_bit_pos0,
)
from bmi2 import (
    Bmi2Dev,
    bmi2_get_regs,
    bmi2_set_regs,
    bmi2_get_sensor_data,
    bmi2_get_sensor_config,
    bmi2_set_int_pin_config,
    bmi2_map_feat_int,
    bmi2_map_data_int,
    bmi2_get_int_status,
    bmi2_get_status,
    bmi2_set_remap_axes,
    bmi2_set_adv_power_save,
    bmi2_soft_reset,
    bmi2_set_command_register,
    bmi2_set_fifo_config,
    bmi2_get_fifo_length,
    bmi2_read_fifo_data,
    bmi2_set_fifo_down_sample,
    bmi2_set_fifo_filter_data,
    bmi2_set_fifo_self_wake_up,
    bmi2_set_fifo_wm,
)
from bmi270 import (
    _set_any_motion_config,
    _set_no_motion_config,
    bmi270_init,
    bmi270_sensor_enable,
    bmi270_sensor_disable,
    bmi270_set_sensor_config,
    bmi270_get_wrist_gesture,
    bmi270_get_step_count,
    bmi270_get_step_activity,
    bmi270_set_step_count_watermark,
    bmi270_reset_step_count,
    BMI270_ANY_MOT_STATUS_MASK,
    BMI270_NO_MOT_STATUS_MASK,
    BMI270_WRIST_GEST_STATUS_MASK,
    BMI270_WRIST_WAKE_UP_STATUS_MASK,
    BMI270_SIG_MOT_STATUS_MASK,
    BMI270_STEP_CNT_STATUS_MASK,
    BMI270_STEP_ACT_STATUS_MASK,
)

# ---------------------------------------------------------------------------
# Facteurs de sous-échantillonnage FIFO (définis dans SparkFun_BMI270_Arduino_Library.h)
# ---------------------------------------------------------------------------
BMI2_FIFO_DOWN_SAMPLE_1   = 0
BMI2_FIFO_DOWN_SAMPLE_2   = 1
BMI2_FIFO_DOWN_SAMPLE_4   = 2
BMI2_FIFO_DOWN_SAMPLE_8   = 3
BMI2_FIFO_DOWN_SAMPLE_16  = 4
BMI2_FIFO_DOWN_SAMPLE_32  = 5
BMI2_FIFO_DOWN_SAMPLE_64  = 6
BMI2_FIFO_DOWN_SAMPLE_128 = 7

# ---------------------------------------------------------------------------
# Constantes de direction de gravité pour performAccelOffsetCalibration
# ---------------------------------------------------------------------------
BMI2_GRAVITY_X    = 0x01
BMI2_GRAVITY_Y    = 0x02
BMI2_GRAVITY_Z    = 0x04
BMI2_GRAVITY_POS  = 0x08
BMI2_GRAVITY_POS_X = BMI2_GRAVITY_X | BMI2_GRAVITY_POS   # 0x09
BMI2_GRAVITY_POS_Y = BMI2_GRAVITY_Y | BMI2_GRAVITY_POS   # 0x0A
BMI2_GRAVITY_POS_Z = BMI2_GRAVITY_Z | BMI2_GRAVITY_POS   # 0x0C
BMI2_GRAVITY_NEG_X = BMI2_GRAVITY_X                       # 0x01
BMI2_GRAVITY_NEG_Y = BMI2_GRAVITY_Y                       # 0x02
BMI2_GRAVITY_NEG_Z = BMI2_GRAVITY_Z                       # 0x04

# ---------------------------------------------------------------------------
# Constantes d'offset pour les sources d'interruption feature
# ---------------------------------------------------------------------------
BMI2_FEATURE_DATA_OFFSET    = 128
BMI2_ANY_MOTION_INT         = 128 + 4   # BMI2_ANY_MOTION (4) + BMI2_FEATURE_DATA_OFFSET = 132
BMI2_NO_MOTION_INT          = 128 + 5   # BMI2_NO_MOTION  (5) + BMI2_FEATURE_DATA_OFFSET = 133
BMI2_SIG_MOTION_INT         = 128 + 3   # BMI2_SIG_MOTION (3) + BMI2_FEATURE_DATA_OFFSET = 131
BMI2_STEP_COUNTER_INT       = 128 + 7   # BMI2_STEP_COUNTER (7) + BMI2_FEATURE_DATA_OFFSET = 135
BMI2_STEP_ACTIVITY_INT      = 128 + 8   # BMI2_STEP_ACTIVITY (8) + BMI2_FEATURE_DATA_OFFSET = 136
BMI2_WRIST_GESTURE_INT      = 128 + 19  # BMI2_WRIST_GESTURE (19) = 147
BMI2_WRIST_WEAR_WAKE_UP_INT = 128 + 20  # BMI2_WRIST_WEAR_WAKE_UP (20) = 148

# Codes d'activité step (step_activity_output)
BMI2_STEP_ACTIVITY_STILL   = 0
BMI2_STEP_ACTIVITY_WALKING = 1
BMI2_STEP_ACTIVITY_RUNNING = 2

# Codes de geste de poignet (wrist_gesture_output)
BMI2_WRIST_GESTURE_UNKNOWN      = 0
BMI2_WRIST_GESTURE_ARM_DOWN     = 1
BMI2_WRIST_GESTURE_ARM_UP       = 2
BMI2_WRIST_GESTURE_SHAKE_JIGGLE = 3
BMI2_WRIST_GESTURE_FLICK_IN     = 4
BMI2_WRIST_GESTURE_FLICK_OUT    = 5

# Constantes de remapping d'axes pour Bmi2Remap.x/y/z
# Valeur = axe physique (bits 0-2) | signe (bit 3)
BMI2_AXIS_POS_X = 0x01   # +X
BMI2_AXIS_POS_Y = 0x02   # +Y
BMI2_AXIS_POS_Z = 0x04   # +Z
BMI2_AXIS_NEG_X = 0x09   # -X (0x01 | 0x08)
BMI2_AXIS_NEG_Y = 0x0A   # -Y (0x02 | 0x08)
BMI2_AXIS_NEG_Z = 0x0C   # -Z (0x04 | 0x08)

# ---------------------------------------------------------------------------
# Brochage par défaut (Arduino NESSO / ESP32-C6)
# ---------------------------------------------------------------------------
# i2c = I2C(0, scl=Pin(8), sda=Pin(10))
_DEFAULT_SCL_PIN  = 8
_DEFAULT_SDA_PIN  = 10
_DEFAULT_I2C_FREQ = 400_000   # 400 kHz


# ---------------------------------------------------------------------------
# Configuration FIFO (équivalent BMI270_FIFOConfig)
# ---------------------------------------------------------------------------

class BMI270_FIFOConfig:
    """Paramètres de configuration du buffer FIFO du BMI270.

    Attributs :
        flags          : combinaison de BMI2_FIFO_ACC_EN et/ou BMI2_FIFO_GYR_EN.
        watermark      : nombre de trames avant déclenchement de l'interruption.
        accelDownSample : facteur de sous-échantillonnage accel (BMI2_FIFO_DOWN_SAMPLE_*).
        gyroDownSample  : facteur de sous-échantillonnage gyro.
        accelFilter    : BMI2_ENABLE pour données filtrées, BMI2_DISABLE pour brutes.
        gyroFilter     : idem pour le gyroscope.
        selfWakeUp     : BMI2_ENABLE pour autoriser la lecture FIFO en low-power.
    """
    __slots__ = ('flags', 'watermark', 'accelDownSample', 'gyroDownSample',
                 'accelFilter', 'gyroFilter', 'selfWakeUp')

    def __init__(self):
        self.flags           = BMI2_FIFO_ACC_EN | BMI2_FIFO_GYR_EN
        self.watermark       = 5
        self.accelDownSample = BMI2_FIFO_DOWN_SAMPLE_1
        self.gyroDownSample  = BMI2_FIFO_DOWN_SAMPLE_1
        self.accelFilter     = BMI2_ENABLE
        self.gyroFilter      = BMI2_ENABLE
        self.selfWakeUp      = BMI2_ENABLE


# ---------------------------------------------------------------------------
# Struct de données exposée à l'utilisateur (équivalent BMI270_SensorData)
# ---------------------------------------------------------------------------

class BMI270_SensorData:
    """Données brutes converties en unités physiques.

    Attributs :
        accelX/Y/Z      : accélération en g.
        gyroX/Y/Z       : vitesse angulaire en °/s.
        sensorTimeMillis: horodatage capteur en ms.
    """
    __slots__ = ('accelX', 'accelY', 'accelZ',
                 'gyroX',  'gyroY',  'gyroZ',
                 'sensorTimeMillis')

    def __init__(self):
        self.accelX = 0.0
        self.accelY = 0.0
        self.accelZ = 0.0
        self.gyroX  = 0.0
        self.gyroY  = 0.0
        self.gyroZ  = 0.0
        self.sensorTimeMillis = 0


# ---------------------------------------------------------------------------
# Classe principale
# ---------------------------------------------------------------------------

class BMI270:
    """Wrapper haut niveau pour le capteur BMI270.

    Correspond à la classe `BMI270` de SparkFun_BMI270_Arduino_Library.
    """

    def __init__(self,_i2c=None):
        self._dev  = Bmi2Dev()
        self._raw  = Bmi2SensData()
        self.data  = BMI270_SensorData()
        self._raw_to_gs      = 0.0
        self._raw_to_deg_sec = 0.0
        self._i2c  = _i2c
        self._i2c_addr = BMI2_I2C_PRIM_ADDR
        # FIFO state
        self._fifo_flags          = 0
        self._fifo_bytes_per_frame = 0

    # ------------------------------------------------------------------
    # Initialisation
    # ------------------------------------------------------------------

    def beginI2C(self, address=BMI2_I2C_PRIM_ADDR,
                 scl_pin=_DEFAULT_SCL_PIN,
                 sda_pin=_DEFAULT_SDA_PIN,
                 freq=_DEFAULT_I2C_FREQ):
        """Initialise la communication I2C et le capteur.

        Args:
            address : adresse I2C du BMI270 (0x68 ou 0x69).
            scl_pin : numéro GPIO pour SCL (défaut 8 sur NESSO).
            sda_pin : numéro GPIO pour SDA (défaut 10 sur NESSO).
            freq    : fréquence I2C en Hz (défaut 400 000).

        Returns:
            int8_t -- BMI2_OK (0) en cas de succès, code négatif sinon.
        """
        if address not in (BMI2_I2C_PRIM_ADDR, BMI2_I2C_SEC_ADDR):
            return BMI2_E_INVALID_INPUT

        self._i2c_addr = address

        # Instanciation du bus I2C MicroPython
        # i2c = I2C(0, scl=Pin(8), sda=Pin(10))
        if self._i2c is None:
            self._i2c = I2C(0,
                            scl=Pin(scl_pin),
                            sda=Pin(sda_pin),
                            freq=freq)
    
        # Interface de communication pour bmi2_dev
        self._dev.intf = BMI2_I2C_INTF

        # Longueur maximale d'un transfert (burst)
        self._dev.read_write_len = 32

        # Callbacks I2C ─ bmi2.py les appelle via dev.read / dev.write / dev.delay_us
        self._dev.read     = self._read_registers_i2c
        self._dev.write    = self._write_registers_i2c
        self._dev.delay_us = self._us_delay

        return self._begin()

    def beginSPI(self, cs_pin, freq=100_000, spi_id=1,
                 sck_pin=None, mosi_pin=None, miso_pin=None):
        """Initialise la communication SPI et le capteur.

        cs_pin  : numéro GPIO du chip select (CS).
        freq    : fréquence SPI en Hz (défaut 100 000).
        spi_id  : identifiant du bus SPI MicroPython (défaut 1).
        sck_pin, mosi_pin, miso_pin : broches SCK/MOSI/MISO (optionnel sur ESP32).

        Returns:
            int8_t -- BMI2_OK (0) en cas de succès, code négatif sinon.
        """
        from machine import SPI, Pin as _Pin

        self._cs = _Pin(cs_pin, _Pin.OUT, value=1)

        spi_kwargs = {'baudrate': freq, 'polarity': 0, 'phase': 0}
        if sck_pin is not None:
            spi_kwargs['sck']  = _Pin(sck_pin)
        if mosi_pin is not None:
            spi_kwargs['mosi'] = _Pin(mosi_pin)
        if miso_pin is not None:
            spi_kwargs['miso'] = _Pin(miso_pin)
        self._spi = SPI(spi_id, **spi_kwargs)

        self._dev.intf      = BMI2_SPI_INTF
        self._dev.dummy_byte = 1   # BMI270 SPI insère 1 octet muet sur les lectures
        self._dev.read_write_len = 32

        self._dev.read     = self._read_registers_spi
        self._dev.write    = self._write_registers_spi
        self._dev.delay_us = self._us_delay

        return self._begin()

    # ------------------------------------------------------------------
    # Lecture de données
    # ------------------------------------------------------------------

    def getSensorData(self):
        """Lit l'accéléromètre + gyroscope et met à jour self.data.

        Returns:
            int8_t -- BMI2_OK (0) en cas de succès.
        """
        rslt = bmi2_get_sensor_data(self._raw, self._dev)
        if rslt != BMI2_OK:
            return rslt

        self._convert_raw_data()
        return BMI2_OK

    # ------------------------------------------------------------------
    # Gestion des features et interruptions
    # ------------------------------------------------------------------

    def enableFeature(self, feature):
        """Active une feature (BMI2_ANY_MOTION, BMI2_NO_MOTION, …)."""
        return bmi270_sensor_enable([feature], self._dev)

    def enableFeatures(self, features):
        """Active une liste de features."""
        return bmi270_sensor_enable(features, self._dev)

    def disableFeature(self, feature):
        """Désactive une feature."""
        return bmi270_sensor_disable([feature], self._dev)

    def disableFeatures(self, features):
        """Désactive une liste de features."""
        return bmi270_sensor_disable(features, self._dev)

    def setConfig(self, cfg):
        """Configure un capteur ou une feature (Bmi2SensConfig)."""
        return bmi270_set_sensor_config([cfg], self._dev)

    def setConfigs(self, cfgs):
        """Configure une liste de capteurs/features."""
        return bmi270_set_sensor_config(cfgs, self._dev)

    def mapInterruptToPin(self, interruptSource, pin):
        """Mappe une source d'interruption sur une broche INT1/INT2.

        interruptSource : BMI2_ANY_MOTION_INT, BMI2_NO_MOTION_INT, …
                          (valeurs >= BMI2_FEATURE_DATA_OFFSET → feature int)
        pin             : BMI2_INT1, BMI2_INT2, BMI2_INT_BOTH, BMI2_INT_NONE
        """
        if interruptSource >= BMI2_FEATURE_DATA_OFFSET:
            feat_type = interruptSource - BMI2_FEATURE_DATA_OFFSET
            return bmi2_map_feat_int(feat_type, pin, self._dev)
        else:
            return bmi2_map_data_int(interruptSource, pin, self._dev)

    def setInterruptPinConfig(self, config):
        """Configure une broche d'interruption (Bmi2IntPinConfig)."""
        return bmi2_set_int_pin_config(config, self._dev)

    def getInterruptStatus(self):
        """Lit le statut des interruptions.

        Returns:
            (rslt, status_uint16) — byte bas = feature ints, byte haut = data ints.
        """
        return bmi2_get_int_status(self._dev)

    # ------------------------------------------------------------------
    # ODR / mode de puissance
    # ------------------------------------------------------------------

    def setAccelODR(self, odr):
        """Modifie l'ODR (output data rate) de l'accéléromètre.

        odr : constante BMI2_ACC_ODR_* (ex. BMI2_ACC_ODR_25HZ).
        """
        from bmi2 import bmi2_get_sensor_config as _bmi2_get_cfg
        cfg = Bmi2SensConfig()
        cfg.type = BMI2_ACCEL
        err = _bmi2_get_cfg([cfg], self._dev)
        if err != BMI2_OK:
            return err
        cfg.cfg.acc.odr = odr
        return bmi270_set_sensor_config([cfg], self._dev)

    def setGyroODR(self, odr):
        """Modifie l'ODR du gyroscope.

        odr : constante BMI2_GYR_ODR_* (ex. BMI2_GYR_ODR_25HZ).
        """
        from bmi2 import bmi2_get_sensor_config as _bmi2_get_cfg
        cfg = Bmi2SensConfig()
        cfg.type = BMI2_GYRO
        err = _bmi2_get_cfg([cfg], self._dev)
        if err != BMI2_OK:
            return err
        cfg.cfg.gyr.odr = odr
        return bmi270_set_sensor_config([cfg], self._dev)

    def setAccelPowerMode(self, mode):
        """Modifie le mode de filtrage de l'accéléromètre.

        mode : BMI2_POWER_OPT_MODE (0) ou BMI2_PERF_OPT_MODE (1).
        """
        from bmi2 import bmi2_get_sensor_config as _bmi2_get_cfg
        cfg = Bmi2SensConfig()
        cfg.type = BMI2_ACCEL
        err = _bmi2_get_cfg([cfg], self._dev)
        if err != BMI2_OK:
            return err
        cfg.cfg.acc.filter_perf = mode
        return bmi270_set_sensor_config([cfg], self._dev)

    def getWristGesture(self):
        """Lit le geste de poignet détecté.

        Returns:
            (rslt, gesture) — gesture est un entier parmi :
                BMI2_WRIST_GESTURE_UNKNOWN (0)
                BMI2_WRIST_GESTURE_ARM_DOWN (1)
                BMI2_WRIST_GESTURE_ARM_UP (2)
                BMI2_WRIST_GESTURE_SHAKE_JIGGLE (3)
                BMI2_WRIST_GESTURE_FLICK_IN (4)
                BMI2_WRIST_GESTURE_FLICK_OUT (5)
        """
        return bmi270_get_wrist_gesture(self._dev)

    def setGyroPowerMode(self, filterMode, noiseMode):
        """Modifie le mode de filtrage et de bruit du gyroscope.

        filterMode : BMI2_POWER_OPT_MODE (0) ou BMI2_PERF_OPT_MODE (1).
        noiseMode  : BMI2_POWER_OPT_MODE (0) ou BMI2_PERF_OPT_MODE (1).
        """
        from bmi2 import bmi2_get_sensor_config as _bmi2_get_cfg
        cfg = Bmi2SensConfig()
        cfg.type = BMI2_GYRO
        err = _bmi2_get_cfg([cfg], self._dev)
        if err != BMI2_OK:
            return err
        cfg.cfg.gyr.filter_perf = filterMode
        cfg.cfg.gyr.noise_perf  = noiseMode
        return bmi270_set_sensor_config([cfg], self._dev)

    def getStatus(self):
        """Lit le registre STATUS du capteur.

        Returns:
            (rslt, status_byte) — bits utiles :
                BMI2_DRDY_ACC (0x80) — accéléromètre prêt
                BMI2_DRDY_GYR (0x40) — gyroscope prêt
                BMI2_DRDY_AUX (0x20) — auxiliaire prêt
                BMI2_CMD_RDY  (0x10) — commande traitée
        """
        return bmi2_get_status(self._dev)

    def enableAdvancedPowerSave(self, enable=True):
        """Active ou désactive le mode advanced power save (APS).

        enable : True pour activer, False pour désactiver.
        Returns: int8_t code d'erreur.
        """
        val = BMI2_ENABLE if enable else BMI2_DISABLE
        return bmi2_set_adv_power_save(val, self._dev)

    def disableAdvancedPowerSave(self):
        """Désactive le mode advanced power save."""
        return bmi2_set_adv_power_save(BMI2_DISABLE, self._dev)

    def remapAxes(self, axes):
        """Reconfigure le mapping des axes X/Y/Z du capteur.

        axes : instance de Bmi2Remap avec .x, .y, .z contenant
               des constantes BMI2_AXIS_POS_X/Y/Z ou BMI2_AXIS_NEG_X/Y/Z.

        Exemple :
            axes = Bmi2Remap()
            axes.x = BMI2_AXIS_POS_Y
            axes.y = BMI2_AXIS_NEG_Z
            axes.z = BMI2_AXIS_POS_X
            imu.remapAxes(axes)

        Returns: int8_t code d'erreur.
        """
        return bmi2_set_remap_axes(axes, self._dev)

    def selfTest(self):
        """Effectue le self-test matériel de l'accéléromètre (±16g).

        Séquence :
          1. Configure accel à 1600 Hz, 16g, mode perf
          2. Mesure polarité positive puis négative
          3. Vérifie les différences min : X>16g, Y<-15g, Z>10g (en mg)
          4. Reset soft après le test

        Returns: BMI2_OK (0) si succès, BMI2_E_SELF_TEST_FAIL (-16) sinon.
        """
        dev = self._dev

        def _signed16(v):
            return v - 0x10000 if v >= 0x8000 else v

        def _write_st(byte_val):
            bmi2_set_regs(BMI2_ACC_SELF_TEST_ADDR, bytes([byte_val]), 1, dev)

        def _read_st():
            rslt, d = bmi2_get_regs(BMI2_ACC_SELF_TEST_ADDR, 1, dev)
            return rslt, d[0] if rslt == BMI2_OK else 0

        # -- 1. Prépare l'accéléromètre --
        bmi270_sensor_enable([BMI2_ACCEL], dev)
        dev.delay_us(1000)

        # Amplitude haute pour le self-test
        rslt, b = _read_st()
        if rslt != BMI2_OK:
            return rslt
        b = bmi2_set_bits(b, BMI2_ACC_SELF_TEST_AMP_MASK, BMI2_ACC_SELF_TEST_AMP_POS, BMI2_ENABLE)
        _write_st(b)

        # ODR 1600 Hz, BWP NORMAL, range 16g, perf mode
        cfg = Bmi2SensConfig()
        cfg.type = BMI2_ACCEL
        bmi2_get_sensor_config([cfg], dev)
        cfg.cfg.acc.odr         = BMI2_ACC_ODR_1600HZ
        cfg.cfg.acc.bwp         = BMI2_ACC_NORMAL_AVG4
        cfg.cfg.acc.filter_perf = BMI2_PERF_OPT_MODE
        cfg.cfg.acc.range       = BMI2_ACC_RANGE_16G
        bmi270_set_sensor_config([cfg], dev)
        dev.delay_us(3000)

        # -- 2. Polarité positive (sign=0) --
        rslt, b = _read_st()
        if rslt != BMI2_OK:
            return rslt
        b = bmi2_set_bit_pos0(b, BMI2_ACC_SELF_TEST_EN_MASK, BMI2_ENABLE)
        b = bmi2_set_bits(b, BMI2_ACC_SELF_TEST_SIGN_MASK, BMI2_ACC_SELF_TEST_SIGN_POS, 0)
        _write_st(b)
        dev.delay_us(51000)

        rslt, raw = bmi2_get_regs(BMI2_ACC_X_LSB_ADDR, BMI2_ACC_NUM_BYTES, dev)
        if rslt != BMI2_OK:
            return rslt
        px = _signed16(raw[0] | (raw[1] << 8))
        py = _signed16(raw[2] | (raw[3] << 8))
        pz = _signed16(raw[4] | (raw[5] << 8))

        # -- 3. Polarité négative (sign=1) --
        rslt, b = _read_st()
        if rslt != BMI2_OK:
            return rslt
        b = bmi2_set_bits(b, BMI2_ACC_SELF_TEST_SIGN_MASK, BMI2_ACC_SELF_TEST_SIGN_POS, 1)
        _write_st(b)
        dev.delay_us(51000)

        rslt, raw = bmi2_get_regs(BMI2_ACC_X_LSB_ADDR, BMI2_ACC_NUM_BYTES, dev)
        if rslt != BMI2_OK:
            return rslt
        nx = _signed16(raw[0] | (raw[1] << 8))
        ny = _signed16(raw[2] | (raw[3] << 8))
        nz = _signed16(raw[4] | (raw[5] << 8))

        # -- 4. Désactive le self-test --
        _write_st(0x00)

        # -- 5. Calcule les différences en mg (résolution 16 bits, ±16g) --
        # lsb_per_g = 2^16 / (2*16) = 2048  →  mg = (raw_diff / 2048) * 1000
        lsb_per_g = 2048
        dx_mg = ((px - nx) * 1000) // lsb_per_g
        dy_mg = ((py - ny) * 1000) // lsb_per_g
        dz_mg = ((pz - nz) * 1000) // lsb_per_g

        # -- 6. Validation (seuils du datasheet BMI270) --
        if dx_mg > 16000 and dy_mg < -15000 and dz_mg > 10000:
            st_rslt = BMI2_OK
        else:
            st_rslt = BMI2_E_SELF_TEST_FAIL

        # -- 7. Reset capteur --
        bmi2_soft_reset(dev)
        return st_rslt

    def setStepCountWatermark(self, watermark):
        """Configure le seuil watermark du step counter (N × 20 pas).

        watermark : 0..1023, chaque unité = 20 pas (1 = interruption/20 pas).
        Returns: int8_t code d'erreur.
        """
        return bmi270_set_step_count_watermark(watermark, self._dev)

    def getStepCount(self):
        """Lit le nombre total de pas depuis la page d'output.

        Returns: (rslt, count) — count : entier non signé.
        """
        return bmi270_get_step_count(self._dev)

    def getStepActivity(self):
        """Lit l'activité de marche courante.

        Returns:
            (rslt, activity) :
                BMI2_STEP_ACTIVITY_STILL   (0) — immobile
                BMI2_STEP_ACTIVITY_WALKING (1) — marche
                BMI2_STEP_ACTIVITY_RUNNING (2) — course
        """
        return bmi270_get_step_activity(self._dev)

    def resetStepCount(self):
        """Remet à zéro le compteur de pas.

        Returns: int8_t code d'erreur.
        """
        return bmi270_reset_step_count(self._dev)

    # ------------------------------------------------------------------
    # FIFO
    # ------------------------------------------------------------------

    def setFIFOConfig(self, config):
        """Configure le buffer FIFO (BMI270_FIFOConfig).

        config : instance de BMI270_FIFOConfig.
        Returns: int8_t code d'erreur.
        """
        dev = self._dev

        # Calcul du nombre d'octets par trame
        bpf = 0
        bpf += 6 if (config.flags & BMI2_FIFO_ACC_EN) else 0
        bpf += 6 if (config.flags & BMI2_FIFO_GYR_EN) else 0
        bpf += 1 if (config.flags & BMI2_FIFO_HEADER_EN) else 0
        self._fifo_bytes_per_frame = bpf if bpf > 0 else 12
        self._fifo_flags = config.flags

        # Active les flags demandés, désactive les autres
        rslt = bmi2_set_fifo_config(config.flags, BMI2_ENABLE, dev)
        if rslt != BMI2_OK:
            return rslt
        disable_flags = (~config.flags) & 0xFFFF
        rslt = bmi2_set_fifo_config(disable_flags, BMI2_DISABLE, dev)
        if rslt != BMI2_OK:
            return rslt

        rslt = bmi2_set_fifo_down_sample(BMI2_ACCEL, config.accelDownSample, dev)
        if rslt != BMI2_OK:
            return rslt
        rslt = bmi2_set_fifo_down_sample(BMI2_GYRO, config.gyroDownSample, dev)
        if rslt != BMI2_OK:
            return rslt

        rslt = bmi2_set_fifo_filter_data(BMI2_ACCEL, config.accelFilter, dev)
        if rslt != BMI2_OK:
            return rslt
        rslt = bmi2_set_fifo_filter_data(BMI2_GYRO, config.gyroFilter, dev)
        if rslt != BMI2_OK:
            return rslt

        rslt = bmi2_set_fifo_self_wake_up(config.selfWakeUp, dev)
        if rslt != BMI2_OK:
            return rslt

        wm_bytes = config.watermark * self._fifo_bytes_per_frame
        return bmi2_set_fifo_wm(wm_bytes, dev)

    def getFIFOLength(self):
        """Retourne le nombre de trames actuellement dans le FIFO.

        Returns: (rslt, num_samples).
        """
        rslt, length_bytes = bmi2_get_fifo_length(self._dev)
        if rslt != BMI2_OK:
            return rslt, 0
        bpf = self._fifo_bytes_per_frame if self._fifo_bytes_per_frame > 0 else 12
        return BMI2_OK, length_bytes // bpf

    def getFIFOData(self, num_samples):
        """Lit des trames du buffer FIFO et les convertit.

        num_samples : nombre maximum de trames à lire.

        Returns:
            (rslt, samples_read, list_of_BMI270_SensorData).
        """
        dev = self._dev
        bpf = self._fifo_bytes_per_frame if self._fifo_bytes_per_frame > 0 else 12

        rslt, fifo_bytes = bmi2_get_fifo_length(dev)
        if rslt != BMI2_OK:
            return rslt, 0, []

        num_frames = fifo_bytes // bpf
        if num_samples > num_frames:
            num_samples = num_frames
        if num_samples == 0:
            return BMI2_OK, 0, []

        rslt, raw = bmi2_read_fifo_data(fifo_bytes, dev)
        if rslt != BMI2_OK:
            return rslt, 0, []

        def _s16(v):
            return v - 0x10000 if v >= 0x8000 else v

        s_acc = self._raw_to_gs
        s_gyr = self._raw_to_deg_sec
        samples = []

        for i in range(num_samples):
            base = i * bpf
            sample = BMI270_SensorData()
            off = base

            if self._fifo_flags & BMI2_FIFO_ACC_EN:
                sample.accelX = _s16(raw[off] | (raw[off+1] << 8)) * s_acc
                sample.accelY = _s16(raw[off+2] | (raw[off+3] << 8)) * s_acc
                sample.accelZ = _s16(raw[off+4] | (raw[off+5] << 8)) * s_acc
                off += 6

            if self._fifo_flags & BMI2_FIFO_GYR_EN:
                sample.gyroX = _s16(raw[off] | (raw[off+1] << 8)) * s_gyr
                sample.gyroY = _s16(raw[off+2] | (raw[off+3] << 8)) * s_gyr
                sample.gyroZ = _s16(raw[off+4] | (raw[off+5] << 8)) * s_gyr

            samples.append(sample)

        return BMI2_OK, len(samples), samples

    def flushFIFO(self):
        """Vide le buffer FIFO.

        Returns: int8_t code d'erreur.
        """
        return bmi2_set_command_register(BMI2_FIFO_FLUSH_CMD, self._dev)

    # ------------------------------------------------------------------
    # Calibration et NVM
    # ------------------------------------------------------------------

    def performAccelOffsetCalibration(self, gravity_dir):
        """Calibration FOC (Fast Offset Calibration) de l'accéléromètre.

        Le capteur doit rester immobile, un axe aligné avec la gravité.

        gravity_dir : constante BMI2_GRAVITY_POS_X/Y/Z ou BMI2_GRAVITY_NEG_X/Y/Z.

        Returns: int8_t code d'erreur.
        """
        dev = self._dev

        # Sauvegarde config actuelle
        cfg = Bmi2SensConfig()
        cfg.type = BMI2_ACCEL
        bmi2_get_sensor_config([cfg], dev)
        saved_range = cfg.cfg.acc.range

        # Passage en ±2g pour la calibration
        cfg.cfg.acc.range = BMI2_ACC_RANGE_2G
        bmi270_set_sensor_config([cfg], dev)
        dev.delay_us(5000)

        # Valeurs attendues (16-bit, ±2g → 1g = 16384 LSB)
        lsb_per_g = 16384
        pos = bool(gravity_dir & BMI2_GRAVITY_POS)
        g_x = (lsb_per_g if pos else -lsb_per_g) if (gravity_dir & BMI2_GRAVITY_X) else 0
        g_y = (lsb_per_g if pos else -lsb_per_g) if (gravity_dir & BMI2_GRAVITY_Y) else 0
        g_z = (lsb_per_g if pos else -lsb_per_g) if (gravity_dir & BMI2_GRAVITY_Z) else 0

        # Lecture de 128 échantillons (~50Hz)
        n = 128
        sx = sy = sz = 0
        for _ in range(n):
            rslt = bmi2_get_sensor_data(self._raw, dev)
            if rslt != BMI2_OK:
                return rslt
            sx += self._raw.acc.x
            sy += self._raw.acc.y
            sz += self._raw.acc.z
            dev.delay_us(20000)

        avg_x = sx // n
        avg_y = sy // n
        avg_z = sz // n

        # Conversion en registres d'offset (résolution 0.0039g = 1/256g)
        # à ±2g : 1 LSB offset_reg = 1/256g, 1 LSB raw16 = 1/16384g → scale=64
        scale = 64
        def _clamp(v):
            return max(-128, min(127, v))

        off_x = _clamp(-(avg_x - g_x) // scale)
        off_y = _clamp(-(avg_y - g_y) // scale)
        off_z = _clamp(-(avg_z - g_z) // scale)

        data = bytes([off_x & 0xFF, off_y & 0xFF, off_z & 0xFF])
        rslt = bmi2_set_regs(BMI2_ACC_OFF_COMP_0_ADDR, data, 3, dev)
        if rslt != BMI2_OK:
            return rslt

        # Active la compensation d'offset accéléromètre (bit BMI2_NV_ACC_OFFSET_MASK)
        rslt, reg = bmi2_get_regs(BMI2_GYR_OFF_COMP_6_ADDR, 1, dev)
        if rslt != BMI2_OK:
            return rslt
        new_reg = bmi2_set_bits(reg[0], BMI2_NV_ACC_OFFSET_MASK,
                                BMI2_NV_ACC_OFFSET_POS, BMI2_ENABLE)
        rslt = bmi2_set_regs(BMI2_GYR_OFF_COMP_6_ADDR, bytes([new_reg]), 1, dev)
        if rslt != BMI2_OK:
            return rslt

        # Restaure la plage originale
        cfg.cfg.acc.range = saved_range
        bmi270_set_sensor_config([cfg], dev)
        return BMI2_OK

    def performGyroOffsetCalibration(self):
        """Calibration FOC du gyroscope (doit être immobile).

        Returns: int8_t code d'erreur.
        """
        dev = self._dev

        # Lecture de 128 échantillons
        n = 128
        sx = sy = sz = 0
        for _ in range(n):
            rslt = bmi2_get_sensor_data(self._raw, dev)
            if rslt != BMI2_OK:
                return rslt
            sx += self._raw.gyr.x
            sy += self._raw.gyr.y
            sz += self._raw.gyr.z
            dev.delay_us(5000)   # 5ms ≈ 200Hz

        avg_x = sx // n
        avg_y = sy // n
        avg_z = sz // n

        # Registres gyro offset : 10-bit signé, résolution ~0.061°/s
        # Scaling : raw16 à ±2000°/s → 1LSB = 1/16 °/s
        # offset reg 10-bit : 1LSB ≈ 0.061°/s ≈ 1/16 °/s
        # Pour simplifier : diviseur ~270 (empirique pour ±2000°/s)
        scale = 270
        def _clamp10(v):
            return max(-512, min(511, v))

        off_x = _clamp10(-avg_x // scale)
        off_y = _clamp10(-avg_y // scale)
        off_z = _clamp10(-avg_z // scale)

        # LSBs (8 bits bas)
        data = bytes([off_x & 0xFF, off_y & 0xFF, off_z & 0xFF])
        rslt = bmi2_set_regs(BMI2_GYR_OFF_COMP_3_ADDR, data, 3, dev)
        if rslt != BMI2_OK:
            return rslt

        # MSBs (2 bits hauts par axe) + activation compensation
        msb = (((off_x >> 8) & 0x03) |
               (((off_y >> 8) & 0x03) << BMI2_GYR_OFF_COMP_MSB_Y_POS) |
               (((off_z >> 8) & 0x03) << BMI2_GYR_OFF_COMP_MSB_Z_POS))
        rslt, reg = bmi2_get_regs(BMI2_GYR_OFF_COMP_6_ADDR, 1, dev)
        if rslt != BMI2_OK:
            return rslt
        new_reg = (reg[0] & ~(BMI2_GYR_OFF_COMP_MSB_X_MASK |
                               BMI2_GYR_OFF_COMP_MSB_Y_MASK |
                               BMI2_GYR_OFF_COMP_MSB_Z_MASK)) | msb
        new_reg = new_reg | BMI2_GYR_OFF_COMP_EN_MASK
        return bmi2_set_regs(BMI2_GYR_OFF_COMP_6_ADDR, bytes([new_reg & 0xFF]), 1, dev)

    def performComponentRetrim(self):
        """Effectue le réétalonnage composant (CRT) du gyroscope.

        Note: le CRT complet (bmi2_do_crt) nécessite un fichier config CRT
        spécifique et une machine d'état matérielle complexe. Cette implémentation
        MicroPython effectue uniquement la séquence de préparation basique.

        Returns: BMI2_OK (la gyroscope reste actif après appel).
        """
        # Le CRT matériel n'est pas implémenté dans ce portage MicroPython.
        # Il faudrait charger le fichier bmi270_crt_config (≈2Ko) et piloter
        # le CRT_GYRO_SELF_TEST feature via les pages feature.
        raise NotImplementedError(
            "Le CRT nécessite bmi2_do_crt() qui dépend d'un fichier config CRT "
            "séparé, non inclus dans ce portage."
        )

    def saveNVM(self):
        """Sauvegarde les valeurs de calibration dans la NVM du capteur.

        ATTENTION : la NVM du BMI270 ne supporte que 14 cycles d'écriture au TOTAL!

        Returns: int8_t code d'erreur.
        """
        dev = self._dev

        # Déverrouille la NVM
        rslt = bmi2_set_regs(BMI2_NVM_CONF_ADDR,
                              bytes([BMI2_NVM_UNLOCK_ENABLE]), 1, dev)
        if rslt != BMI2_OK:
            return rslt

        # Déclenche la programmation NVM
        rslt = bmi2_set_command_register(BMI2_NVM_PROG_CMD, dev)
        if rslt != BMI2_OK:
            return rslt

        # Attend la fin de la programmation (~14ms)
        dev.delay_us(14000)

        # Reverrouille la NVM
        bmi2_set_regs(BMI2_NVM_CONF_ADDR,
                      bytes([BMI2_NVM_UNLOCK_DISABLE]), 1, dev)
        return BMI2_OK

    # ------------------------------------------------------------------
    # Interface I2C auxiliaire (stub — nécessite le driver bmi2_aux)
    # ------------------------------------------------------------------

    def setAuxPullUps(self, pull_up_value):
        """Configure les résistances de pull-up de l'interface I2C auxiliaire.

        pull_up_value : BMI2_ASDA_PUPSEL_OFF/40K/10K/2K.
        Returns: int8_t code d'erreur.
        """
        return bmi2_set_regs(BMI2_AUX_IF_TRIM, bytes([pull_up_value]), 1, self._dev)

    def readAux(self, addr, num_bytes):
        """Lit des données depuis le capteur auxiliaire (mode manuel).

        Note: bmi2_read_aux_man_mode n'est pas implémenté dans ce portage.
        Pour une implémentation complète, voir bmi2.c Bosch (section auxiliary).

        Returns: int8_t code d'erreur.
        """
        print("readAux: interface AUX non entièrement implementee en MicroPython")
        return BMI2_OK

    def writeAux(self, addr, data):
        """Écrit un ou plusieurs octets vers le capteur auxiliaire (mode manuel).

        addr : adresse registre cible.
        data : octet (int) ou bytes à écrire.

        Returns: int8_t code d'erreur.
        """
        print("writeAux: interface AUX non entièrement implementee en MicroPython")
        return BMI2_OK

    # ------------------------------------------------------------------
    # Implémentation interne
    # ------------------------------------------------------------------

    def _begin(self):
        """Initialisation commune (après configuration de l'interface)."""
        # Initialise le BMI270 (charge le firmware, vérifie chip-id)
        print("Initialisation du BMI270...")
        err = bmi270_init(self._dev)
        print("BMI270 init result:", err)
        if err != BMI2_OK:
            return err

        # Active accéléromètre et gyroscope
        print("Activation des capteurs...")
        err = bmi270_sensor_enable([BMI2_ACCEL, BMI2_GYRO], self._dev)
        print("Sensor enable result:", err)
        if err != BMI2_OK:
            return err

        # Récupère les configs pour connaître les plages (range)
        cfg_acc = Bmi2SensConfig()
        cfg_acc.type = BMI2_ACCEL
        cfg_gyr = Bmi2SensConfig()
        cfg_gyr.type = BMI2_GYRO
        print("Récupération des configs capteurs...")
        err = bmi2_get_sensor_config([cfg_acc, cfg_gyr], self._dev)
        print("Get sensor config result:", err)
        if err != BMI2_OK:
            return err

        self._raw_to_gs      = self._convert_raw_to_gs_scalar(cfg_acc.cfg.acc.range)
        self._raw_to_deg_sec = self._convert_raw_to_deg_sec_scalar(cfg_gyr.cfg.gyr.range)

        return BMI2_OK

    # ------------------------------------------------------------------
    # Conversion unités
    # ------------------------------------------------------------------

    @staticmethod
    def _convert_raw_to_gs_scalar(acc_range):
        """Scalaire raw → g's.

        acc_range | pleine échelle
        ----------|---------------
            0     |  ±2 g
            1     |  ±4 g
            2     |  ±8 g
            3     | ±16 g
        """
        return (2 << acc_range) / 32768.0

    @staticmethod
    def _convert_raw_to_deg_sec_scalar(gyr_range):
        """Scalaire raw → °/s.

        gyr_range (BMI2_GYR_RANGE_xxx) | pleine échelle
        --------------------------------|---------------
              0                         | ±2000 °/s
              1                         | ±1000 °/s
              2                         |  ±500 °/s
              3                         |  ±250 °/s
              4                         |  ±125 °/s
        """
        return (125 * (1 << (BMI2_GYR_RANGE_125 - gyr_range))) / 32768.0

    def _convert_raw_data(self):
        """Convertit self._raw (Bmi2SensData) → self.data (BMI270_SensorData)."""
        s = self._raw_to_gs
        self.data.accelX = self._raw.acc.x * s
        self.data.accelY = self._raw.acc.y * s
        self.data.accelZ = self._raw.acc.z * s

        s = self._raw_to_deg_sec
        self.data.gyroX = self._raw.gyr.x * s
        self.data.gyroY = self._raw.gyr.y * s
        self.data.gyroZ = self._raw.gyr.z * s

        # Horodatage capteur en ms (résolution 39.0625 µs par tick)
        self.data.sensorTimeMillis = (
            self._raw.sens_time * 1000.0 * BMI2_SENSORTIME_RESOLUTION
        )

    # ------------------------------------------------------------------
    # Callbacks I2C pour bmi2_dev
    # ------------------------------------------------------------------

    def _read_registers_i2c(self, reg_addr, length):
        """Lit `length` octets à partir du registre `reg_addr` via I2C.

        Signature attendue par bmi2.py :
            dev.read(reg_addr, length) -> bytes

        Returns:
            bytes de longueur `length`, ou b'' en cas d'erreur.
        """
        try:
            return self._i2c.readfrom_mem(self._i2c_addr, reg_addr, length)
        except OSError:
            return b''

    def _write_registers_i2c(self, reg_addr, data):
        """Écrit `data` (bytes / bytearray) dans le registre `reg_addr` via I2C.

        Signature attendue par bmi2.py :
            dev.write(reg_addr, data: bytes) -> None (ou int pour le code de retour)
        """
        try:
            self._i2c.writeto_mem(self._i2c_addr, reg_addr, data)
            return 0   # BMI2_INTF_RET_SUCCESS
        except OSError:
            return -1

    # ------------------------------------------------------------------
    # Callbacks SPI pour bmi2_dev
    # ------------------------------------------------------------------

    def _read_registers_spi(self, reg_addr, length):
        """Lit `length` octets depuis le registre `reg_addr` via SPI.

        Pour SPI, bmi2_get_regs ajoute le dummy_byte (1) au length.
        Ce callback reçoit donc length = requested + dummy_byte et renvoie
        les octets bruts (bmi2_get_regs retire ensuite l'octet muet).
        """
        try:
            buf = bytearray(length)
            self._cs(0)
            self._spi.write(bytes([reg_addr | 0x80]))
            self._spi.readinto(buf)
            self._cs(1)
            return bytes(buf)
        except Exception:
            return b''

    def _write_registers_spi(self, reg_addr, data):
        """Écrit `data` dans le registre `reg_addr` via SPI."""
        try:
            self._cs(0)
            self._spi.write(bytes([reg_addr & 0x7F]) + bytes(data))
            self._cs(1)
            return 0
        except Exception:
            return -1

    @staticmethod
    def _us_delay(period_us, _=None):
        """Délai en microsecondes.

        Signature attendue par bmi2.py :
            dev.delay_us(period_us, intf_ptr)
        """
        utime.sleep_us(period_us)

    # ------------------------------------------------------------------
    # AddOns
    # ------------------------------------------------------------------
    def configureAnyMotion(self,duration=5,threshold=170,select_x=BMI2_ENABLE,select_y=BMI2_ENABLE,select_z=BMI2_ENABLE):
        # Configure any-motion : 100 ms (duration=5), seuil 83 mg (threshold=170), axes XYZ
        # duration  : en unités de 20 ms → 5 × 20 ms = 100 ms
        # threshold : en unités de 0.4883 mg → 170 × 0.4883 mg ≈ 83 mg
        config = Bmi2AnyMotionConfig()
        config.duration = duration
        config.threshold = threshold
        config.select_x = select_x
        config.select_y = select_y
        config.select_z = select_z
        return _set_any_motion_config(config, self._dev)
        

    def configureNoMotion(self,duration=5,threshold=170,select_x=BMI2_ENABLE,select_y=BMI2_ENABLE,select_z=BMI2_ENABLE):
        # Configure no-motion : 100 ms (duration=5), seuil 83 mg (threshold=170), axes XYZ
        #  Set the "no motion" config. Default values:
        # 
        # .duration  - 5 = 100ms
        # .threshold - 144 = 70mg
        # .select_x  - Enabled
        # .select_y  - Enabled
        # .select_z  - Enabled
        config = Bmi2NoMotionConfig()
        config.duration = duration
        config.threshold = threshold
        config.select_x = select_x
        config.select_y = select_y
        config.select_z = select_z
        rslt = _set_no_motion_config(config, self._dev)
       
        return rslt

    def get_major_axis(self):
        self.getSensorData()
        ax, ay, az = self.data.accelX, self.data.accelY, self.data.accelZ
        max_val = max(abs(ax), abs(ay), abs(az))
        if max_val == abs(ax):
            return BMI2_AXIS_POS_X if ax > 0 else BMI2_AXIS_NEG_X
        elif max_val == abs(ay):
            return BMI2_AXIS_POS_Y if ay > 0 else BMI2_AXIS_NEG_Y
        elif max_val == abs(az):
            return BMI2_AXIS_POS_Z if az > 0 else BMI2_AXIS_NEG_Z
        
    def configureInterrupts(self,interrupts):
        """
        install your handler
        int_pin = Pin(INTERRUPT_PIN, Pin.IN)
        int_pin.irq(trigger=Pin.IRQ_FALLING, handler=interrupt_handler)
        """
        for i in interrupts:
            assert i in [BMI2_ANY_MOTION_INT,BMI2_NO_MOTION_INT,
                         BMI2_SIG_MOTION_INT,BMI2_STEP_COUNTER_INT,
                         BMI2_STEP_ACTIVITY_INT,BMI2_WRIST_GESTURE_INT,
                         BMI2_WRIST_WEAR_WAKE_UP_INT, ]

            rslt = self.mapInterruptToPin(i, BMI2_INT1)
            if rslt != BMI2_OK:
                raise Exception("Failed to map interrupt to pin")


        intPinConfig = Bmi2IntPinConfig()
        intPinConfig.pin_type = BMI2_INT1
        intPinConfig.int_latch = BMI2_INT_NON_LATCH
        intPinConfig.pin_cfg[0].lvl       = BMI2_INT_ACTIVE_LOW
        intPinConfig.pin_cfg[0].od        = BMI2_INT_PUSH_PULL
        intPinConfig.pin_cfg[0].output_en = BMI2_INT_OUTPUT_ENABLE
        intPinConfig.pin_cfg[0].input_en  = BMI2_INT_INPUT_DISABLE
        rslt = self.setInterruptPinConfig(intPinConfig)
        if rslt != BMI2_OK:
            raise Exception("Failed to configure interrupt pin")
        #return rslt

        

