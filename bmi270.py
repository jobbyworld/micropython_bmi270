"""bmi270.py -- Port MicroPython du driver Bosch BMI270 (bmi270.h + bmi270.c).

Ce module est le pendant de `bmi270.c` / `bmi270.h`. Il:
  - fournit le blob du firmware en tant que constante `bytes` (à copier depuis
    le fichier C d'origine -- voir commentaire ci-dessous),
  - expose `bmi270_init(dev)` qui charge le firmware et initialise le capteur,
  - expose `bmi270_sensor_enable` / `bmi270_sensor_disable` qui délèguent à
    `bmi2.py` les opérations sur le registre PWR_CTRL.

Seules les fonctions utiles pour l'Example01 (init + enable accel+gyro) sont
implémentées dans un premier temps.
"""

from bmi2_defs import (
    BMI2_OK, BMI2_E_NULL_PTR, BMI2_E_INVALID_SENSOR, BMI2_SPI_INTF,
    BMI2_ENABLE, BMI2_DISABLE,
    BMI2_ACCEL, BMI2_GYRO, BMI2_AUX, BMI2_TEMP,
    BMI2_ANY_MOTION, BMI2_NO_MOTION,
    BMI2_WRIST_GESTURE, BMI2_WRIST_WEAR_WAKE_UP,
    BMI2_ACCEL_SENS_SEL, BMI2_GYRO_SENS_SEL,
    BMI2_ANY_MOT_SEL, BMI2_NO_MOT_SEL, BMI2_MAIN_SENSORS,
    BMI2_WRIST_GEST_SEL, BMI2_WRIST_WEAR_WAKE_UP_SEL,
    BMI2_WRIST_GEST_FEAT_EN_MASK, BMI2_WRIST_GEST_FEAT_EN_POS,
    BMI2_WRIST_WEAR_WAKE_UP_FEAT_EN_MASK, BMI2_WRIST_WEAR_WAKE_UP_FEAT_EN_POS,
    BMI2_PAGE_0, BMI2_PAGE_1, BMI2_PAGE_2, BMI2_PAGE_3,
    BMI2_PAGE_4, BMI2_PAGE_5, BMI2_PAGE_6, BMI2_PAGE_7,
    BMI2_ANY_MOT_FEAT_EN_OFFSET, BMI2_NO_MOT_FEAT_EN_OFFSET,
    BMI2_ANY_NO_MOT_EN_MASK, BMI2_ANY_NO_MOT_EN_POS,
    BMI2_ANY_NO_MOT_DUR_MASK, BMI2_ANY_NO_MOT_X_SEL_MASK,
    BMI2_ANY_NO_MOT_Y_SEL_MASK, BMI2_ANY_NO_MOT_Z_SEL_MASK,
    BMI2_ANY_NO_MOT_THRES_MASK,
    BMI2_ANY_NO_MOT_X_SEL_POS, BMI2_ANY_NO_MOT_Y_SEL_POS,
    BMI2_ANY_NO_MOT_Z_SEL_POS,
    BMI2_FEAT_SIZE_IN_BYTES, BMI2_FEATURES_REG_ADDR,
    Bmi2FeatureConfig, Bmi2MapInt, Bmi2SensConfig,
    Bmi2AnyMotionConfig, Bmi2NoMotionConfig,
    BMI2_SIG_MOTION, BMI2_SIG_MOTION_SEL,
    BMI2_SIG_MOT_FEAT_EN_OFFSET, BMI2_SIG_MOT_FEAT_EN_MASK,
    BMI2_STEP_DETECTOR, BMI2_STEP_COUNTER, BMI2_STEP_ACTIVITY,
    BMI2_STEP_DETECT_SEL, BMI2_STEP_COUNT_SEL, BMI2_STEP_ACT_SEL,
    BMI2_STEP_COUNT_FEAT_EN_OFFSET,
    BMI2_STEP_DET_FEAT_EN_MASK, BMI2_STEP_COUNT_FEAT_EN_MASK, BMI2_STEP_ACT_FEAT_EN_MASK,
    BMI2_STEP_DET_FEAT_EN_POS, BMI2_STEP_COUNT_FEAT_EN_POS, BMI2_STEP_ACT_FEAT_EN_POS,
    BMI2_STEP_COUNT_WM_LEVEL_MASK, BMI2_STEP_COUNT_RST_CNT_MASK,
    bmi2_set_bits, bmi2_set_bit_pos0,
)
from bmi2 import (
    bmi2_sec_init,
    _sensor_enable,
    _sensor_disable,
    bmi2_set_sensor_config,
    bmi2_get_sensor_config,
    bmi2_get_feat_config,
    bmi2_extract_input_feat_config,
    bmi2_set_adv_power_save,
    bmi2_set_regs,
)

# ---------------------------------------------------------------------------
# Constantes spécifiques BMI270
# ---------------------------------------------------------------------------
BMI270_CHIP_ID        = 0x24
BMI270_MAX_PAGE_NUM   = 8
BMI270_MAX_FEAT_IN    = 17
BMI270_MAX_FEAT_OUT   = 7
BMI270_MAX_INT_MAP    = 8

# Feature start addresses within pages
BMI270_CONFIG_ID_STRT_ADDR          = 0x00
BMI270_MAX_BURST_LEN_STRT_ADDR      = 0x02
BMI270_CRT_GYRO_SELF_TEST_STRT_ADDR = 0x03
BMI270_ABORT_STRT_ADDR              = 0x03
BMI270_AXIS_MAP_STRT_ADDR           = 0x04
BMI270_GYRO_SELF_OFF_STRT_ADDR      = 0x05
BMI270_NVM_PROG_PREP_STRT_ADDR      = 0x05
BMI270_GYRO_GAIN_UPDATE_STRT_ADDR   = 0x06
BMI270_ANY_MOT_STRT_ADDR            = 0x0C
BMI270_NO_MOT_STRT_ADDR             = 0x00
BMI270_SIG_MOT_STRT_ADDR            = 0x04
BMI270_STEP_CNT_1_STRT_ADDR         = 0x00
BMI270_STEP_CNT_4_STRT_ADDR         = 0x02
BMI270_WRIST_GEST_STRT_ADDR         = 0x06
BMI270_WRIST_WEAR_WAKE_UP_STRT_ADDR = 0x00
BMI270_STEP_CNT_OUT_STRT_ADDR       = 0x00
BMI270_STEP_ACT_OUT_STRT_ADDR       = 0x04
BMI270_WRIST_GEST_OUT_STRT_ADDR     = 0x06
BMI270_GYR_USER_GAIN_OUT_STRT_ADDR  = 0x08
BMI270_GYRO_CROSS_SENSE_STRT_ADDR   = 0x0C
BMI270_NVM_VFRM_OUT_STRT_ADDR       = 0x0E

# Interrupt status masks (INT_STATUS_0, byte 0)
BMI270_SIG_MOT_STATUS_MASK       = 0x01
BMI270_STEP_CNT_STATUS_MASK      = 0x02
BMI270_STEP_ACT_STATUS_MASK      = 0x04
BMI270_WRIST_WAKE_UP_STATUS_MASK = 0x08
BMI270_WRIST_GEST_STATUS_MASK    = 0x10
BMI270_NO_MOT_STATUS_MASK        = 0x20
BMI270_ANY_MOT_STATUS_MASK       = 0x40

# Interrupt mapping masks (INT1_MAP_FEAT / INT2_MAP_FEAT)
BMI270_INT_SIG_MOT_MASK           = 0x01
BMI270_INT_STEP_COUNTER_MASK      = 0x02
BMI270_INT_STEP_DETECTOR_MASK     = 0x02
BMI270_INT_STEP_ACT_MASK          = 0x04
BMI270_INT_WRIST_WEAR_WAKEUP_MASK = 0x08
BMI270_INT_WRIST_GEST_MASK        = 0x10
BMI270_INT_NO_MOT_MASK            = 0x20
BMI270_INT_ANY_MOT_MASK           = 0x40

# ---------------------------------------------------------------------------
# Blob firmware (bmi270_config_file)
# ---------------------------------------------------------------------------
# IMPORTANT : copier-coller ici le contenu de la variable
#   const uint8_t bmi270_config_file[] = { ... };
# depuis le fichier source C :
#   src/bmi270_api/bmi270.c
# en remplaçant les accolades C par un littéral bytes Python, par exemple :
#
#   BMI270_CONFIG_FILE = bytes([0xc8, 0x2e, 0x00, 0x2e, ...])
#
# Le blob fait environ 8 Ko -- il doit rester en flash (objet bytes)
# et n'est chargé en RAM que pendant le transfert DMA.
#
# Le placeholder ci-dessous lèvera une ValueError à l'exécution pour
# signaler que le blob n'a pas encore été rempli.
BMI270_CONFIG_FILE = "/firmware/bmi270_config.bin"

# ---------------------------------------------------------------------------
# Initialisation BMI270
# ---------------------------------------------------------------------------

# ---------------------------------------------------------------------------
# Tables de features et de mapping d'interruptions (BMI270-spécifiques)
# ---------------------------------------------------------------------------

def _make_feat_in():
    """Construit la table BMI270_FEAT_IN (équivalent de bmi270_feat_in[] en C)."""
    from bmi2_defs import (
        BMI2_CONFIG_ID, BMI2_MAX_BURST_LEN, BMI2_CRT_GYRO_SELF_TEST,
        BMI2_ABORT_CRT_GYRO_SELF_TEST, BMI2_AXIS_MAP, BMI2_GYRO_SELF_OFF,
        BMI2_NVM_PROG_PREP, BMI2_GYRO_GAIN_UPDATE, BMI2_SIG_MOTION,
        BMI2_STEP_COUNTER_PARAMS, BMI2_STEP_DETECTOR, BMI2_STEP_COUNTER,
        BMI2_STEP_ACTIVITY, BMI2_WRIST_GESTURE, BMI2_WRIST_WEAR_WAKE_UP,
    )
    data = [
        (BMI2_CONFIG_ID,                BMI2_PAGE_1, BMI270_CONFIG_ID_STRT_ADDR),
        (BMI2_MAX_BURST_LEN,            BMI2_PAGE_1, BMI270_MAX_BURST_LEN_STRT_ADDR),
        (BMI2_CRT_GYRO_SELF_TEST,       BMI2_PAGE_1, BMI270_CRT_GYRO_SELF_TEST_STRT_ADDR),
        (BMI2_ABORT_CRT_GYRO_SELF_TEST, BMI2_PAGE_1, BMI270_ABORT_STRT_ADDR),
        (BMI2_AXIS_MAP,                 BMI2_PAGE_1, BMI270_AXIS_MAP_STRT_ADDR),
        (BMI2_GYRO_SELF_OFF,            BMI2_PAGE_1, BMI270_GYRO_SELF_OFF_STRT_ADDR),
        (BMI2_NVM_PROG_PREP,            BMI2_PAGE_1, BMI270_NVM_PROG_PREP_STRT_ADDR),
        (BMI2_GYRO_GAIN_UPDATE,         BMI2_PAGE_1, BMI270_GYRO_GAIN_UPDATE_STRT_ADDR),
        (BMI2_ANY_MOTION,               BMI2_PAGE_1, BMI270_ANY_MOT_STRT_ADDR),
        (BMI2_NO_MOTION,                BMI2_PAGE_2, BMI270_NO_MOT_STRT_ADDR),
        (BMI2_SIG_MOTION,               BMI2_PAGE_2, BMI270_SIG_MOT_STRT_ADDR),
        (BMI2_STEP_COUNTER_PARAMS,      BMI2_PAGE_3, BMI270_STEP_CNT_1_STRT_ADDR),
        (BMI2_STEP_DETECTOR,            BMI2_PAGE_6, BMI270_STEP_CNT_4_STRT_ADDR),
        (BMI2_STEP_COUNTER,             BMI2_PAGE_6, BMI270_STEP_CNT_4_STRT_ADDR),
        (BMI2_STEP_ACTIVITY,            BMI2_PAGE_6, BMI270_STEP_CNT_4_STRT_ADDR),
        (BMI2_WRIST_GESTURE,            BMI2_PAGE_6, BMI270_WRIST_GEST_STRT_ADDR),
        (BMI2_WRIST_WEAR_WAKE_UP,       BMI2_PAGE_7, BMI270_WRIST_WEAR_WAKE_UP_STRT_ADDR),
    ]
    result = []
    for t, p, s in data:
        fc = Bmi2FeatureConfig()
        fc.type = t; fc.page = p; fc.start_addr = s
        result.append(fc)
    return result


def _make_map_int():
    """Construit la table BMI270_MAP_INT (équivalent de bmi270_map_int[] en C)."""
    from bmi2_defs import (
        BMI2_SIG_MOTION, BMI2_STEP_COUNTER, BMI2_STEP_DETECTOR,
        BMI2_STEP_ACTIVITY, BMI2_WRIST_GESTURE, BMI2_WRIST_WEAR_WAKE_UP,
    )
    data = [
        (BMI2_SIG_MOTION,         BMI270_INT_SIG_MOT_MASK),
        (BMI2_STEP_COUNTER,       BMI270_INT_STEP_COUNTER_MASK),
        (BMI2_STEP_DETECTOR,      BMI270_INT_STEP_DETECTOR_MASK),
        (BMI2_STEP_ACTIVITY,      BMI270_INT_STEP_ACT_MASK),
        (BMI2_WRIST_GESTURE,      BMI270_INT_WRIST_GEST_MASK),
        (BMI2_WRIST_WEAR_WAKE_UP, BMI270_INT_WRIST_WEAR_WAKEUP_MASK),
        (BMI2_ANY_MOTION,         BMI270_INT_ANY_MOT_MASK),
        (BMI2_NO_MOTION,          BMI270_INT_NO_MOT_MASK),
    ]
    result = []
    for t, m in data:
        mi = Bmi2MapInt()
        mi.type = t; mi.sens_map_int = m
        result.append(mi)
    return result


BMI270_FEAT_IN = _make_feat_in()
BMI270_MAP_INT = _make_map_int()


# ---------------------------------------------------------------------------
# Initialisation
# ---------------------------------------------------------------------------

def bmi270_init(dev):
    """Initialise le BMI270.

    Équivalent de `bmi270_init()` dans bmi270.c :
      1. Configure les champs variant du device (chip_id, config_size,
         variant_feature, dummy_byte, config_file_ptr).
      2. Appelle `bmi2_sec_init()` qui vérifie le chip-id, effectue un
         soft-reset et charge le firmware.
      3. Peuple les méta-données features dans dev.

    Args:
        dev: instance de Bmi2Dev (voir bmi2.py).

    Returns:
        int8_t -- BMI2_OK (0) en cas de succès, code d'erreur négatif sinon.
    """
    if dev is None:
        return BMI2_E_NULL_PTR

    if BMI270_CONFIG_FILE is None:
        raise ValueError(
            "bmi270.py : BMI270_CONFIG_FILE est None.\n"
            "Assignez un chemin de fichier (str) ou un blob bytes à BMI270_CONFIG_FILE."
        )

    # Chip ID attendu par bmi2_sec_init pour la vérification
    dev.chip_id = BMI270_CHIP_ID

    # Taille du firmware : depuis le système de fichiers ou depuis le blob en mémoire
    if isinstance(BMI270_CONFIG_FILE, str):
        import os
        dev.config_size = os.stat(BMI270_CONFIG_FILE)[6]   # st_size
    else:
        dev.config_size = len(BMI270_CONFIG_FILE)

    # Variante : active gyro cross-sensitivity (bit 0) et CRT RTOSK (bit 1)
    # BMI2_GYRO_CROSS_SENS_ENABLE = 0x0001, BMI2_CRT_RTOSK_ENABLE = 0x0002
    dev.variant_feature = 0x0001 | 0x0002

    # Un octet dummy supplémentaire est lu lors d'une lecture SPI
    if dev.intf == BMI2_SPI_INTF:
        dev.dummy_byte = 1
    else:
        dev.dummy_byte = 0

    # Pointeur vers le blob firmware (bytes)
    dev.config_file_ptr = BMI270_CONFIG_FILE

    # Initialisation générique BMI2 (soft-reset + chargement firmware)
    rslt = bmi2_sec_init(dev)
    if rslt != BMI2_OK:
        return rslt

    # Méta-données features (tables Python simplifiées -- non utilisées pour
    # l'Example01, mais nécessaires si l'on veut activer des features avancées)
    dev.page_max     = BMI270_MAX_PAGE_NUM
    dev.input_sens   = BMI270_MAX_FEAT_IN
    dev.out_sens     = BMI270_MAX_FEAT_OUT
    dev.sens_int_map = BMI270_MAX_INT_MAP

    # Note : bmi2_get_gyro_cross_sense() est intentionnellement omis ici.
    # dev.variant_feature = 0 désactive la compensation cross-sens dans
    # bmi2_get_sensor_data(); on peut l'activer plus tard si nécessaire.
    dev.variant_feature = 0

    # Injecte les tables de features et de mapping d'interruptions
    dev.feat_config = BMI270_FEAT_IN
    dev.map_int     = BMI270_MAP_INT

    return BMI2_OK


# ---------------------------------------------------------------------------
# Activation / désactivation de capteurs
# ---------------------------------------------------------------------------

def _bmi270_select_sensor(sens_list):
    """Construit un bitfield sensor_sel incluant capteurs principaux et features."""
    sensor_sel = 0
    for s in sens_list:
        if   s == BMI2_ACCEL:            sensor_sel |= BMI2_ACCEL_SENS_SEL
        elif s == BMI2_GYRO:             sensor_sel |= BMI2_GYRO_SENS_SEL
        elif s == BMI2_AUX:              sensor_sel |= (1 << BMI2_AUX)
        elif s == BMI2_TEMP:             sensor_sel |= (1 << BMI2_TEMP)
        elif s == BMI2_ANY_MOTION:       sensor_sel |= BMI2_ANY_MOT_SEL
        elif s == BMI2_NO_MOTION:        sensor_sel |= BMI2_NO_MOT_SEL
        elif s == BMI2_WRIST_GESTURE:    sensor_sel |= BMI2_WRIST_GEST_SEL
        elif s == BMI2_WRIST_WEAR_WAKE_UP: sensor_sel |= BMI2_WRIST_WEAR_WAKE_UP_SEL
        elif s == BMI2_SIG_MOTION:       sensor_sel |= BMI2_SIG_MOTION_SEL
        elif s == BMI2_STEP_DETECTOR:    sensor_sel |= BMI2_STEP_DETECT_SEL
        elif s == BMI2_STEP_COUNTER:     sensor_sel |= BMI2_STEP_COUNT_SEL
        elif s == BMI2_STEP_ACTIVITY:    sensor_sel |= BMI2_STEP_ACT_SEL
        else:                            return BMI2_E_INVALID_SENSOR, 0
    return BMI2_OK, sensor_sel


def _set_any_motion(enable, dev):
    """Active ou désactive la feature any-motion dans les pages feature."""
    fc = Bmi2FeatureConfig()
    if not bmi2_extract_input_feat_config(fc, BMI2_ANY_MOTION, dev):
        return BMI2_E_INVALID_SENSOR
    rslt, feat_config = bmi2_get_feat_config(fc.page, dev)
    if rslt != BMI2_OK:
        return rslt
    idx = fc.start_addr + BMI2_ANY_MOT_FEAT_EN_OFFSET
    feat_config[idx] = bmi2_set_bits(feat_config[idx],
                                     BMI2_ANY_NO_MOT_EN_MASK,
                                     BMI2_ANY_NO_MOT_EN_POS, enable)
    rslt = bmi2_set_regs(BMI2_FEATURES_REG_ADDR, feat_config,
                         BMI2_FEAT_SIZE_IN_BYTES, dev)
    if rslt == BMI2_OK:
        if enable == BMI2_ENABLE:
            dev.sens_en_stat |= BMI2_ANY_MOT_SEL
        else:
            dev.sens_en_stat &= ~BMI2_ANY_MOT_SEL
    return rslt


def _set_no_motion(enable, dev):
    """Active ou désactive la feature no-motion dans les pages feature."""
    fc = Bmi2FeatureConfig()
    if not bmi2_extract_input_feat_config(fc, BMI2_NO_MOTION, dev):
        return BMI2_E_INVALID_SENSOR
    rslt, feat_config = bmi2_get_feat_config(fc.page, dev)
    if rslt != BMI2_OK:
        return rslt
    idx = fc.start_addr + BMI2_NO_MOT_FEAT_EN_OFFSET
    feat_config[idx] = bmi2_set_bits(feat_config[idx],
                                     BMI2_ANY_NO_MOT_EN_MASK,
                                     BMI2_ANY_NO_MOT_EN_POS, enable)
    rslt = bmi2_set_regs(BMI2_FEATURES_REG_ADDR, feat_config,
                         BMI2_FEAT_SIZE_IN_BYTES, dev)
    if rslt == BMI2_OK:
        if enable == BMI2_ENABLE:
            dev.sens_en_stat |= BMI2_NO_MOT_SEL
        else:
            dev.sens_en_stat &= ~BMI2_NO_MOT_SEL
    return rslt


def _set_wrist_gesture(enable, dev):
    """Active ou désactive la feature wrist gesture dans les pages feature."""
    fc = Bmi2FeatureConfig()
    if not bmi2_extract_input_feat_config(fc, BMI2_WRIST_GESTURE, dev):
        return BMI2_E_INVALID_SENSOR
    rslt, feat_config = bmi2_get_feat_config(fc.page, dev)
    if rslt != BMI2_OK:
        return rslt
    idx = fc.start_addr
    feat_config[idx] = bmi2_set_bits(feat_config[idx],
                                     BMI2_WRIST_GEST_FEAT_EN_MASK,
                                     BMI2_WRIST_GEST_FEAT_EN_POS, enable)
    rslt = bmi2_set_regs(BMI2_FEATURES_REG_ADDR, feat_config,
                         BMI2_FEAT_SIZE_IN_BYTES, dev)
    if rslt == BMI2_OK:
        if enable == BMI2_ENABLE:
            dev.sens_en_stat |= BMI2_WRIST_GEST_SEL
        else:
            dev.sens_en_stat &= ~BMI2_WRIST_GEST_SEL
    return rslt


def _set_wrist_wear_wake_up(enable, dev):
    """Active ou désactive la feature wrist wear wake-up dans les pages feature."""
    fc = Bmi2FeatureConfig()
    if not bmi2_extract_input_feat_config(fc, BMI2_WRIST_WEAR_WAKE_UP, dev):
        return BMI2_E_INVALID_SENSOR
    rslt, feat_config = bmi2_get_feat_config(fc.page, dev)
    if rslt != BMI2_OK:
        return rslt
    idx = fc.start_addr
    feat_config[idx] = bmi2_set_bits(feat_config[idx],
                                     BMI2_WRIST_WEAR_WAKE_UP_FEAT_EN_MASK,
                                     BMI2_WRIST_WEAR_WAKE_UP_FEAT_EN_POS, enable)
    rslt = bmi2_set_regs(BMI2_FEATURES_REG_ADDR, feat_config,
                         BMI2_FEAT_SIZE_IN_BYTES, dev)
    if rslt == BMI2_OK:
        if enable == BMI2_ENABLE:
            dev.sens_en_stat |= BMI2_WRIST_WEAR_WAKE_UP_SEL
        else:
            dev.sens_en_stat &= ~BMI2_WRIST_WEAR_WAKE_UP_SEL
    return rslt


def _set_sig_motion(enable, dev):
    """Active ou désactive la feature significant-motion dans les pages feature."""
    fc = Bmi2FeatureConfig()
    if not bmi2_extract_input_feat_config(fc, BMI2_SIG_MOTION, dev):
        return BMI2_E_INVALID_SENSOR
    rslt, feat_config = bmi2_get_feat_config(fc.page, dev)
    if rslt != BMI2_OK:
        return rslt
    idx = fc.start_addr + BMI2_SIG_MOT_FEAT_EN_OFFSET
    feat_config[idx] = bmi2_set_bit_pos0(feat_config[idx], BMI2_SIG_MOT_FEAT_EN_MASK, enable)
    rslt = bmi2_set_regs(BMI2_FEATURES_REG_ADDR, feat_config, BMI2_FEAT_SIZE_IN_BYTES, dev)
    if rslt == BMI2_OK:
        if enable == BMI2_ENABLE:
            dev.sens_en_stat |= BMI2_SIG_MOTION_SEL
        else:
            dev.sens_en_stat &= ~BMI2_SIG_MOTION_SEL
    return rslt


def _set_step_detector(enable, dev):
    """Active ou désactive la feature step-detector dans les pages feature."""
    fc = Bmi2FeatureConfig()
    if not bmi2_extract_input_feat_config(fc, BMI2_STEP_DETECTOR, dev):
        return BMI2_E_INVALID_SENSOR
    rslt, feat_config = bmi2_get_feat_config(fc.page, dev)
    if rslt != BMI2_OK:
        return rslt
    idx = fc.start_addr + BMI2_STEP_COUNT_FEAT_EN_OFFSET
    feat_config[idx] = bmi2_set_bits(feat_config[idx],
                                     BMI2_STEP_DET_FEAT_EN_MASK,
                                     BMI2_STEP_DET_FEAT_EN_POS, enable)
    rslt = bmi2_set_regs(BMI2_FEATURES_REG_ADDR, feat_config, BMI2_FEAT_SIZE_IN_BYTES, dev)
    if rslt == BMI2_OK:
        if enable == BMI2_ENABLE:
            dev.sens_en_stat |= BMI2_STEP_DETECT_SEL
        else:
            dev.sens_en_stat &= ~BMI2_STEP_DETECT_SEL
    return rslt


def _set_step_counter(enable, dev):
    """Active ou désactive la feature step-counter dans les pages feature."""
    fc = Bmi2FeatureConfig()
    if not bmi2_extract_input_feat_config(fc, BMI2_STEP_COUNTER, dev):
        return BMI2_E_INVALID_SENSOR
    rslt, feat_config = bmi2_get_feat_config(fc.page, dev)
    if rslt != BMI2_OK:
        return rslt
    idx = fc.start_addr + BMI2_STEP_COUNT_FEAT_EN_OFFSET
    feat_config[idx] = bmi2_set_bits(feat_config[idx],
                                     BMI2_STEP_COUNT_FEAT_EN_MASK,
                                     BMI2_STEP_COUNT_FEAT_EN_POS, enable)
    rslt = bmi2_set_regs(BMI2_FEATURES_REG_ADDR, feat_config, BMI2_FEAT_SIZE_IN_BYTES, dev)
    if rslt == BMI2_OK:
        if enable == BMI2_ENABLE:
            dev.sens_en_stat |= BMI2_STEP_COUNT_SEL
        else:
            dev.sens_en_stat &= ~BMI2_STEP_COUNT_SEL
    return rslt


def _set_step_activity(enable, dev):
    """Active ou désactive la feature step-activity dans les pages feature."""
    fc = Bmi2FeatureConfig()
    if not bmi2_extract_input_feat_config(fc, BMI2_STEP_ACTIVITY, dev):
        return BMI2_E_INVALID_SENSOR
    rslt, feat_config = bmi2_get_feat_config(fc.page, dev)
    if rslt != BMI2_OK:
        return rslt
    idx = fc.start_addr + BMI2_STEP_COUNT_FEAT_EN_OFFSET
    feat_config[idx] = bmi2_set_bits(feat_config[idx],
                                     BMI2_STEP_ACT_FEAT_EN_MASK,
                                     BMI2_STEP_ACT_FEAT_EN_POS, enable)
    rslt = bmi2_set_regs(BMI2_FEATURES_REG_ADDR, feat_config, BMI2_FEAT_SIZE_IN_BYTES, dev)
    if rslt == BMI2_OK:
        if enable == BMI2_ENABLE:
            dev.sens_en_stat |= BMI2_STEP_ACT_SEL
        else:
            dev.sens_en_stat &= ~BMI2_STEP_ACT_SEL
    return rslt


def _enable_sensor_features(sensor_sel, dev):
    """Active les features sélectionnées (any-motion, no-motion, wrist gesture…)."""
    rslt = BMI2_OK
    if sensor_sel & BMI2_ANY_MOT_SEL:
        rslt = _set_any_motion(BMI2_ENABLE, dev)
    if rslt == BMI2_OK and (sensor_sel & BMI2_NO_MOT_SEL):
        rslt = _set_no_motion(BMI2_ENABLE, dev)
    if rslt == BMI2_OK and (sensor_sel & BMI2_WRIST_GEST_SEL):
        rslt = _set_wrist_gesture(BMI2_ENABLE, dev)
    if rslt == BMI2_OK and (sensor_sel & BMI2_WRIST_WEAR_WAKE_UP_SEL):
        rslt = _set_wrist_wear_wake_up(BMI2_ENABLE, dev)
    if rslt == BMI2_OK and (sensor_sel & BMI2_SIG_MOTION_SEL):
        rslt = _set_sig_motion(BMI2_ENABLE, dev)
    if rslt == BMI2_OK and (sensor_sel & BMI2_STEP_DETECT_SEL):
        rslt = _set_step_detector(BMI2_ENABLE, dev)
    if rslt == BMI2_OK and (sensor_sel & BMI2_STEP_COUNT_SEL):
        rslt = _set_step_counter(BMI2_ENABLE, dev)
    if rslt == BMI2_OK and (sensor_sel & BMI2_STEP_ACT_SEL):
        rslt = _set_step_activity(BMI2_ENABLE, dev)
    return rslt


def _disable_sensor_features(sensor_sel, dev):
    """Désactive les features sélectionnées."""
    rslt = BMI2_OK
    if sensor_sel & BMI2_ANY_MOT_SEL:
        rslt = _set_any_motion(BMI2_DISABLE, dev)
    if rslt == BMI2_OK and (sensor_sel & BMI2_NO_MOT_SEL):
        rslt = _set_no_motion(BMI2_DISABLE, dev)
    if rslt == BMI2_OK and (sensor_sel & BMI2_WRIST_GEST_SEL):
        rslt = _set_wrist_gesture(BMI2_DISABLE, dev)
    if rslt == BMI2_OK and (sensor_sel & BMI2_WRIST_WEAR_WAKE_UP_SEL):
        rslt = _set_wrist_wear_wake_up(BMI2_DISABLE, dev)
    if rslt == BMI2_OK and (sensor_sel & BMI2_SIG_MOTION_SEL):
        rslt = _set_sig_motion(BMI2_DISABLE, dev)
    if rslt == BMI2_OK and (sensor_sel & BMI2_STEP_DETECT_SEL):
        rslt = _set_step_detector(BMI2_DISABLE, dev)
    if rslt == BMI2_OK and (sensor_sel & BMI2_STEP_COUNT_SEL):
        rslt = _set_step_counter(BMI2_DISABLE, dev)
    if rslt == BMI2_OK and (sensor_sel & BMI2_STEP_ACT_SEL):
        rslt = _set_step_activity(BMI2_DISABLE, dev)
    return rslt


def bmi270_sensor_enable(sens_list, dev):
    """Active les capteurs/features listés dans `sens_list`.

    Gère à la fois les capteurs principaux (accel/gyro via PWR_CTRL) et
    les features avancées (any-motion, no-motion via pages feature).

    Args:
        sens_list: liste/tuple de constantes BMI2_ACCEL, BMI2_GYRO,
                   BMI2_ANY_MOTION, BMI2_NO_MOTION, etc.
        dev: instance de Bmi2Dev.

    Returns:
        int8_t code d'erreur.
    """
    if dev is None or sens_list is None:
        return BMI2_E_NULL_PTR
    rslt, sensor_sel = _bmi270_select_sensor(sens_list)
    if rslt != BMI2_OK:
        return rslt

    # Capteurs principaux (accel/gyro) via PWR_CTRL
    rslt = _sensor_enable(sensor_sel & BMI2_MAIN_SENSORS, dev)
    if rslt != BMI2_OK:
        return rslt

    # Features avancées via pages feature (avec gestion APS)
    feat_sel = sensor_sel & ~BMI2_MAIN_SENSORS
    if feat_sel:
        aps = dev.aps_status
        if aps == BMI2_ENABLE:
            rslt = bmi2_set_adv_power_save(BMI2_DISABLE, dev)
        if rslt == BMI2_OK:
            rslt = _enable_sensor_features(feat_sel, dev)
            if aps == BMI2_ENABLE and rslt == BMI2_OK:
                rslt = bmi2_set_adv_power_save(BMI2_ENABLE, dev)
    return rslt


def bmi270_sensor_disable(sens_list, dev):
    """Désactive les capteurs/features listés dans `sens_list`.

    Args:
        sens_list: liste/tuple de constantes BMI2_*.
        dev: instance de Bmi2Dev.

    Returns:
        int8_t code d'erreur.
    """
    if dev is None or sens_list is None:
        return BMI2_E_NULL_PTR

    rslt, sensor_sel = _bmi270_select_sensor(sens_list)
    if rslt != BMI2_OK:
        return rslt

    # Capteurs principaux via PWR_CTRL
    rslt = _sensor_disable(sensor_sel & BMI2_MAIN_SENSORS, dev)
    if rslt != BMI2_OK:
        return rslt

    # Features avancées via pages feature (avec gestion APS)
    feat_sel = sensor_sel & ~BMI2_MAIN_SENSORS
    if feat_sel:
        aps = dev.aps_status
        if aps == BMI2_ENABLE:
            rslt = bmi2_set_adv_power_save(BMI2_DISABLE, dev)
        if rslt == BMI2_OK:
            rslt = _disable_sensor_features(feat_sel, dev)
            if aps == BMI2_ENABLE and rslt == BMI2_OK:
                rslt = bmi2_set_adv_power_save(BMI2_ENABLE, dev)
    return rslt


# ---------------------------------------------------------------------------
# Configuration capteur (délégation à bmi2.py)
# ---------------------------------------------------------------------------

def _set_any_motion_config(config, dev):
    """Écrit la configuration any-motion dans les pages feature.

    config : instance de Bmi2AnyMotionConfig.
    """
    fc = Bmi2FeatureConfig()
    if not bmi2_extract_input_feat_config(fc, BMI2_ANY_MOTION, dev):
        return BMI2_E_INVALID_SENSOR
    rslt, feat_config = bmi2_get_feat_config(fc.page, dev)
    if rslt != BMI2_OK:
        return rslt
    sa = fc.start_addr
    # Word 0 : duration + axis select
    w0 = feat_config[sa] | (feat_config[sa + 1] << 8)
    w0 = bmi2_set_bit_pos0(w0, BMI2_ANY_NO_MOT_DUR_MASK, config.duration)
    w0 = bmi2_set_bits(w0, BMI2_ANY_NO_MOT_X_SEL_MASK, BMI2_ANY_NO_MOT_X_SEL_POS, config.select_x)
    w0 = bmi2_set_bits(w0, BMI2_ANY_NO_MOT_Y_SEL_MASK, BMI2_ANY_NO_MOT_Y_SEL_POS, config.select_y)
    w0 = bmi2_set_bits(w0, BMI2_ANY_NO_MOT_Z_SEL_MASK, BMI2_ANY_NO_MOT_Z_SEL_POS, config.select_z)
    feat_config[sa]     = w0 & 0xFF
    feat_config[sa + 1] = (w0 >> 8) & 0xFF
    # Word 1 : threshold
    w1 = feat_config[sa + 2] | (feat_config[sa + 3] << 8)
    w1 = bmi2_set_bit_pos0(w1, BMI2_ANY_NO_MOT_THRES_MASK, config.threshold)
    feat_config[sa + 2] = w1 & 0xFF
    feat_config[sa + 3] = (w1 >> 8) & 0xFF
    return bmi2_set_regs(BMI2_FEATURES_REG_ADDR, feat_config, BMI2_FEAT_SIZE_IN_BYTES, dev)


def _set_no_motion_config(config, dev):
    """Écrit la configuration no-motion dans les pages feature.

    config : instance de Bmi2NoMotionConfig.
    """
    fc = Bmi2FeatureConfig()
    if not bmi2_extract_input_feat_config(fc, BMI2_NO_MOTION, dev):
        return BMI2_E_INVALID_SENSOR
    rslt, feat_config = bmi2_get_feat_config(fc.page, dev)
    if rslt != BMI2_OK:
        return rslt
    sa = fc.start_addr
    # Word 0 : duration + axis select
    w0 = feat_config[sa] | (feat_config[sa + 1] << 8)
    w0 = bmi2_set_bit_pos0(w0, BMI2_ANY_NO_MOT_DUR_MASK, config.duration)
    w0 = bmi2_set_bits(w0, BMI2_ANY_NO_MOT_X_SEL_MASK, BMI2_ANY_NO_MOT_X_SEL_POS, config.select_x)
    w0 = bmi2_set_bits(w0, BMI2_ANY_NO_MOT_Y_SEL_MASK, BMI2_ANY_NO_MOT_Y_SEL_POS, config.select_y)
    w0 = bmi2_set_bits(w0, BMI2_ANY_NO_MOT_Z_SEL_MASK, BMI2_ANY_NO_MOT_Z_SEL_POS, config.select_z)
    feat_config[sa]     = w0 & 0xFF
    feat_config[sa + 1] = (w0 >> 8) & 0xFF
    # Word 1 : threshold
    w1 = feat_config[sa + 2] | (feat_config[sa + 3] << 8)
    w1 = bmi2_set_bit_pos0(w1, BMI2_ANY_NO_MOT_THRES_MASK, config.threshold)
    feat_config[sa + 2] = w1 & 0xFF
    feat_config[sa + 3] = (w1 >> 8) & 0xFF
    return bmi2_set_regs(BMI2_FEATURES_REG_ADDR, feat_config, BMI2_FEAT_SIZE_IN_BYTES, dev)


def bmi270_set_sensor_config(sens_cfg_list, dev):
    """Configure les capteurs de la liste.

    Gère BMI2_ANY_MOTION et BMI2_NO_MOTION via les pages feature.
    Délègue le reste à bmi2_set_sensor_config.

    Args:
        sens_cfg_list: liste de Bmi2SensConfig.
        dev: instance de Bmi2Dev.

    Returns:
        int8_t code d'erreur.
    """
    rslt = BMI2_OK
    for cfg in sens_cfg_list:
        if cfg.type == BMI2_ANY_MOTION:
            rslt = _set_any_motion_config(cfg.cfg.any_motion, dev)
        elif cfg.type == BMI2_NO_MOTION:
            rslt = _set_no_motion_config(cfg.cfg.no_motion, dev)
        else:
            rslt = bmi2_set_sensor_config([cfg], dev)
        if rslt != BMI2_OK:
            break
    return rslt


def bmi270_get_sensor_config(sens_cfg_list, dev):
    """Lit la configuration des capteurs de la liste.

    Args:
        sens_cfg_list: liste de Bmi2SensConfig (type doit être pré-rempli).
        dev: instance de Bmi2Dev.

    Returns:
        int8_t code d'erreur.
    """
    return bmi2_get_sensor_config(sens_cfg_list, dev)


def bmi270_get_wrist_gesture(dev):
    """Lit le geste de poignet détecté depuis la page d'output (PAGE_0).

    Équivalent de get_wrist_gest_status() dans bmi270.c.

    Returns:
        (rslt, gesture_byte) — gesture_byte est un entier :
            0 = inconnu
            1 = bras vers le bas (ARM_DOWN)
            2 = bras vers le haut (ARM_UP)
            3 = secousse (SHAKE_JIGGLE)
            4 = flick vers l'intérieur (FLICK_IN)
            5 = flick vers l'extérieur (FLICK_OUT)
    """
    rslt, feat_config = bmi2_get_feat_config(BMI2_PAGE_0, dev)
    if rslt != BMI2_OK:
        return rslt, 0
    return BMI2_OK, feat_config[BMI270_WRIST_GEST_OUT_STRT_ADDR]


def bmi270_get_step_count(dev):
    """Lit le nombre de pas depuis la page d'output (PAGE_0, octets 0..3).

    Returns:
        (rslt, count) — count : entier 32 bits non signé.
    """
    rslt, feat_config = bmi2_get_feat_config(BMI2_PAGE_0, dev)
    if rslt != BMI2_OK:
        return rslt, 0
    sa = BMI270_STEP_CNT_OUT_STRT_ADDR
    count = (feat_config[sa]
             | (feat_config[sa + 1] << 8)
             | (feat_config[sa + 2] << 16)
             | (feat_config[sa + 3] << 24))
    return BMI2_OK, count


def bmi270_get_step_activity(dev):
    """Lit l'activité de marche depuis la page d'output (PAGE_0, octet 4).

    Returns:
        (rslt, activity) — activity : 0 = Immobile, 1 = Marche, 2 = Course.
    """
    rslt, feat_config = bmi2_get_feat_config(BMI2_PAGE_0, dev)
    if rslt != BMI2_OK:
        return rslt, 0
    return BMI2_OK, feat_config[BMI270_STEP_ACT_OUT_STRT_ADDR]


def bmi270_set_step_count_watermark(watermark, dev):
    """Configure le seuil watermark du step counter (N × 20 pas par interruption).

    Args:
        watermark: valeur 0..1023 (1 = interruption toutes les 20 pas).
        dev: instance de Bmi2Dev.

    Returns:
        int8_t code d'erreur.
    """
    fc = Bmi2FeatureConfig()
    if not bmi2_extract_input_feat_config(fc, BMI2_STEP_COUNTER, dev):
        return BMI2_E_INVALID_SENSOR
    rslt, feat_config = bmi2_get_feat_config(fc.page, dev)
    if rslt != BMI2_OK:
        return rslt
    sa = fc.start_addr
    w0 = feat_config[sa] | (feat_config[sa + 1] << 8)
    w0 = (w0 & ~BMI2_STEP_COUNT_WM_LEVEL_MASK) | (watermark & BMI2_STEP_COUNT_WM_LEVEL_MASK)
    feat_config[sa]     = w0 & 0xFF
    feat_config[sa + 1] = (w0 >> 8) & 0xFF
    return bmi2_set_regs(BMI2_FEATURES_REG_ADDR, feat_config, BMI2_FEAT_SIZE_IN_BYTES, dev)


def bmi270_reset_step_count(dev):
    """Remet à zéro le compteur de pas.

    Returns:
        int8_t code d'erreur.
    """
    fc = Bmi2FeatureConfig()
    if not bmi2_extract_input_feat_config(fc, BMI2_STEP_COUNTER, dev):
        return BMI2_E_INVALID_SENSOR
    rslt, feat_config = bmi2_get_feat_config(fc.page, dev)
    if rslt != BMI2_OK:
        return rslt
    sa = fc.start_addr
    w0 = feat_config[sa] | (feat_config[sa + 1] << 8)
    w0 |= BMI2_STEP_COUNT_RST_CNT_MASK
    feat_config[sa]     = w0 & 0xFF
    feat_config[sa + 1] = (w0 >> 8) & 0xFF
    return bmi2_set_regs(BMI2_FEATURES_REG_ADDR, feat_config, BMI2_FEAT_SIZE_IN_BYTES, dev)
