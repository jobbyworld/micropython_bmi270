"""bmi270_legacy.py -- Port MicroPython du driver Bosch BMI270 variante Legacy.

Source : bmi270_legacy.h + bmi270_legacy.c

9 fonctions publiques : bmi270_legacy_init, sensor_enable, sensor_disable,
set_sensor_config, get_sensor_config, get_feature_data,
update_gyro_user_gain, read_gyro_user_gain, map_feat_int.
"""

import os

from bmi2_defs import (
    BMI2_OK, BMI2_E_NULL_PTR, BMI2_E_INVALID_SENSOR,
    BMI2_SPI_INTF, BMI2_ENABLE, BMI2_DISABLE,
    BMI2_ACCEL, BMI2_GYRO, BMI2_AUX,
    BMI2_ACCEL_SENS_SEL, BMI2_GYRO_SENS_SEL, BMI2_AUX_SENS_SEL,
    BMI2_ANY_MOTION, BMI2_NO_MOTION,
    BMI2_ANY_MOT_SEL, BMI2_NO_MOT_SEL,
    BMI2_SIG_MOTION, BMI2_SIG_MOTION_SEL,
    BMI2_STEP_DETECTOR, BMI2_STEP_COUNTER, BMI2_STEP_ACTIVITY,
    BMI2_STEP_DETECT_SEL, BMI2_STEP_COUNT_SEL, BMI2_STEP_ACT_SEL,
    BMI2_GYRO_GAIN_UPDATE, BMI2_GYRO_GAIN_UPDATE_SEL,
    BMI2_HIGH_G, BMI2_HIGH_G_SEL,
    BMI2_LOW_G, BMI2_LOW_G_SEL,
    BMI2_FLAT, BMI2_FLAT_SEL,
    BMI2_ORIENTATION, BMI2_ORIENT_SEL,
    BMI2_SINGLE_TAP, BMI2_DOUBLE_TAP, BMI2_TRIPLE_TAP,
    BMI2_SINGLE_TAP_SEL, BMI2_DOUBLE_TAP_SEL, BMI2_TRIPLE_TAP_SEL,
    BMI2_GYRO_CROSS_SENS_ENABLE, BMI2_CRT_RTOSK_ENABLE,
    BMI2_CONFIG_ID, BMI2_MAX_BURST_LEN, BMI2_CRT_GYRO_SELF_TEST,
    BMI2_ABORT_CRT_GYRO_SELF_TEST, BMI2_AXIS_MAP, BMI2_GYRO_SELF_OFF,
    BMI2_NVM_PROG_PREP, BMI2_STEP_COUNTER_PARAMS,
    BMI2_ANY_MOT_FEAT_EN_OFFSET, BMI2_NO_MOT_FEAT_EN_OFFSET,
    BMI2_ANY_NO_MOT_EN_MASK, BMI2_ANY_NO_MOT_EN_POS,
    BMI2_SIG_MOT_FEAT_EN_OFFSET, BMI2_SIG_MOT_FEAT_EN_MASK,
    BMI2_STEP_COUNT_FEAT_EN_OFFSET,
    BMI2_STEP_DET_FEAT_EN_MASK, BMI2_STEP_COUNT_FEAT_EN_MASK, BMI2_STEP_ACT_FEAT_EN_MASK,
    BMI2_STEP_DET_FEAT_EN_POS, BMI2_STEP_COUNT_FEAT_EN_POS, BMI2_STEP_ACT_FEAT_EN_POS,
    BMI2_PAGE_0, BMI2_PAGE_1, BMI2_PAGE_2, BMI2_PAGE_3,
    BMI2_PAGE_4, BMI2_PAGE_5, BMI2_PAGE_6,
    BMI2_FEAT_SIZE_IN_BYTES, BMI2_FEATURES_REG_ADDR,
    Bmi2FeatureConfig, Bmi2MapInt,
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
    bmi2_set_regs,
    bmi2_map_feat_int,
)

# ---------------------------------------------------------------------------
# Constantes spécifiques BMI270_LEGACY
# ---------------------------------------------------------------------------
BMI270_LEGACY_CHIP_ID      = 0x24
BMI270_LEGACY_MAX_PAGE_NUM = 8
BMI270_LEGACY_MAX_FEAT_IN  = 23
BMI270_LEGACY_MAX_FEAT_OUT = 8
BMI270_LEGACY_MAX_INT_MAP  = 14

# Feature input start addresses
BMI270_LEGACY_CONFIG_ID_STRT_ADDR            = 0x00
BMI270_LEGACY_MAX_BURST_LEN_STRT_ADDR        = 0x02
BMI270_LEGACY_CRT_GYRO_SELF_TEST_STRT_ADDR   = 0x03
BMI270_LEGACY_ABORT_STRT_ADDR                = 0x03
BMI270_LEGACY_AXIS_MAP_STRT_ADDR             = 0x04
BMI270_LEGACY_GYRO_SELF_OFF_STRT_ADDR        = 0x05
BMI270_LEGACY_NVM_PROG_PREP_STRT_ADDR        = 0x05
BMI270_LEGACY_ANY_MOT_STRT_ADDR              = 0x06
BMI270_LEGACY_NO_MOT_STRT_ADDR               = 0x0A
BMI270_LEGACY_ORIENT_STRT_ADDR               = 0x00
BMI270_LEGACY_HIGH_G_STRT_ADDR               = 0x04
BMI270_LEGACY_LOW_G_STRT_ADDR                = 0x0A
BMI270_LEGACY_FLAT_STRT_ADDR                 = 0x00
BMI270_LEGACY_SIG_MOT_STRT_ADDR              = 0x04
BMI270_LEGACY_STEP_COUNT_STRT_ADDR           = 0x00
BMI270_LEGACY_GYRO_USERGAIN_UPDATE_STRT_ADDR = 0x04
BMI270_LEGACY_TAP_DETECT_1_STRT_ADDR         = 0x00
BMI270_LEGACY_TAP_DETECT_2_STRT_ADDR         = 0x00

# Feature output start addresses (page 0)
BMI270_LEGACY_STEP_CNT_OUT_STRT_ADDR       = 0x00
BMI270_LEGACY_STEP_ACT_OUT_STRT_ADDR       = 0x04
BMI270_LEGACY_ORIENT_HIGH_G_OUT_STRT_ADDR  = 0x06
BMI270_LEGACY_GYR_USER_GAIN_OUT_STRT_ADDR  = 0x08
BMI270_LEGACY_GYRO_CROSS_SENSE_STRT_ADDR   = 0x0C
BMI270_LEGACY_NVM_VFRM_OUT_STRT_ADDR       = 0x0E

# Interrupt mapping masks
BMI270_LEGACY_INT_SIG_MOT_MASK       = 0x01
BMI270_LEGACY_INT_STEP_COUNTER_MASK  = 0x02
BMI270_LEGACY_INT_STEP_DETECTOR_MASK = 0x02
BMI270_LEGACY_INT_STEP_ACTIVITY_MASK = 0x02
BMI270_LEGACY_INT_HIGH_G_MASK        = 0x04
BMI270_LEGACY_INT_LOW_G_MASK         = 0x04
BMI270_LEGACY_INT_TAP_MASK           = 0x08
BMI270_LEGACY_INT_FLAT_MASK          = 0x10
BMI270_LEGACY_INT_NO_MOT_MASK        = 0x20
BMI270_LEGACY_INT_ANY_MOT_MASK       = 0x40
BMI270_LEGACY_INT_ORIENT_MASK        = 0x80
BMI270_LEGACY_INT_SINGLE_TAP_MASK    = 0x20
BMI270_LEGACY_INT_DOUBLE_TAP_MASK    = 0x40
BMI270_LEGACY_INT_TRIPLE_TAP_MASK    = 0x80

# ---------------------------------------------------------------------------
# Firmware
# ---------------------------------------------------------------------------
BMI270_LEGACY_CONFIG_FILE = "/firmware/bmi270_config_legacy.bin"

# ---------------------------------------------------------------------------
# Tables de features
# ---------------------------------------------------------------------------

def _make_feat_in():
    from bmi2_defs import BMI2_TAP_DETECTOR_1, BMI2_TAP_DETECTOR_2
    data = [
        (BMI2_CONFIG_ID,               BMI2_PAGE_1, BMI270_LEGACY_CONFIG_ID_STRT_ADDR),
        (BMI2_MAX_BURST_LEN,           BMI2_PAGE_1, BMI270_LEGACY_MAX_BURST_LEN_STRT_ADDR),
        (BMI2_CRT_GYRO_SELF_TEST,      BMI2_PAGE_1, BMI270_LEGACY_CRT_GYRO_SELF_TEST_STRT_ADDR),
        (BMI2_ABORT_CRT_GYRO_SELF_TEST, BMI2_PAGE_1, BMI270_LEGACY_ABORT_STRT_ADDR),
        (BMI2_AXIS_MAP,                BMI2_PAGE_1, BMI270_LEGACY_AXIS_MAP_STRT_ADDR),
        (BMI2_GYRO_SELF_OFF,           BMI2_PAGE_1, BMI270_LEGACY_GYRO_SELF_OFF_STRT_ADDR),
        (BMI2_NVM_PROG_PREP,           BMI2_PAGE_1, BMI270_LEGACY_NVM_PROG_PREP_STRT_ADDR),
        (BMI2_ANY_MOTION,              BMI2_PAGE_1, BMI270_LEGACY_ANY_MOT_STRT_ADDR),
        (BMI2_NO_MOTION,               BMI2_PAGE_1, BMI270_LEGACY_NO_MOT_STRT_ADDR),
        (BMI2_ORIENTATION,             BMI2_PAGE_2, BMI270_LEGACY_ORIENT_STRT_ADDR),
        (BMI2_HIGH_G,                  BMI2_PAGE_2, BMI270_LEGACY_HIGH_G_STRT_ADDR),
        (BMI2_LOW_G,                   BMI2_PAGE_2, BMI270_LEGACY_LOW_G_STRT_ADDR),
        (BMI2_FLAT,                    BMI2_PAGE_3, BMI270_LEGACY_FLAT_STRT_ADDR),
        (BMI2_SIG_MOTION,              BMI2_PAGE_3, BMI270_LEGACY_SIG_MOT_STRT_ADDR),
        (BMI2_STEP_DETECTOR,           BMI2_PAGE_4, BMI270_LEGACY_STEP_COUNT_STRT_ADDR),
        (BMI2_STEP_COUNTER,            BMI2_PAGE_4, BMI270_LEGACY_STEP_COUNT_STRT_ADDR),
        (BMI2_STEP_ACTIVITY,           BMI2_PAGE_4, BMI270_LEGACY_STEP_COUNT_STRT_ADDR),
        (BMI2_GYRO_GAIN_UPDATE,        BMI2_PAGE_4, BMI270_LEGACY_GYRO_USERGAIN_UPDATE_STRT_ADDR),
        (BMI2_SINGLE_TAP,              BMI2_PAGE_5, BMI270_LEGACY_TAP_DETECT_1_STRT_ADDR),
        (BMI2_DOUBLE_TAP,              BMI2_PAGE_5, BMI270_LEGACY_TAP_DETECT_1_STRT_ADDR),
        (BMI2_TRIPLE_TAP,              BMI2_PAGE_5, BMI270_LEGACY_TAP_DETECT_1_STRT_ADDR),
        (BMI2_TAP_DETECTOR_1,          BMI2_PAGE_5, BMI270_LEGACY_TAP_DETECT_1_STRT_ADDR),
        (BMI2_TAP_DETECTOR_2,          BMI2_PAGE_6, BMI270_LEGACY_TAP_DETECT_2_STRT_ADDR),
    ]
    result = []
    for t, p, s in data:
        fc = Bmi2FeatureConfig()
        fc.type = t; fc.page = p; fc.start_addr = s
        result.append(fc)
    return result


def _make_map_int():
    from bmi2_defs import BMI2_TAP
    data = [
        (BMI2_SIG_MOTION,    BMI270_LEGACY_INT_SIG_MOT_MASK),
        (BMI2_STEP_COUNTER,  BMI270_LEGACY_INT_STEP_COUNTER_MASK),
        (BMI2_STEP_DETECTOR, BMI270_LEGACY_INT_STEP_DETECTOR_MASK),
        (BMI2_STEP_ACTIVITY, BMI270_LEGACY_INT_STEP_ACTIVITY_MASK),
        (BMI2_HIGH_G,        BMI270_LEGACY_INT_HIGH_G_MASK),
        (BMI2_LOW_G,         BMI270_LEGACY_INT_LOW_G_MASK),
        (BMI2_TAP,           BMI270_LEGACY_INT_TAP_MASK),
        (BMI2_FLAT,          BMI270_LEGACY_INT_FLAT_MASK),
        (BMI2_NO_MOTION,     BMI270_LEGACY_INT_NO_MOT_MASK),
        (BMI2_ANY_MOTION,    BMI270_LEGACY_INT_ANY_MOT_MASK),
        (BMI2_ORIENTATION,   BMI270_LEGACY_INT_ORIENT_MASK),
        (BMI2_SINGLE_TAP,    BMI270_LEGACY_INT_SINGLE_TAP_MASK),
        (BMI2_DOUBLE_TAP,    BMI270_LEGACY_INT_DOUBLE_TAP_MASK),
        (BMI2_TRIPLE_TAP,    BMI270_LEGACY_INT_TRIPLE_TAP_MASK),
    ]
    result = []
    for t, m in data:
        mi = Bmi2MapInt()
        mi.type = t; mi.sens_map_int = m
        result.append(mi)
    return result


BMI270_LEGACY_FEAT_IN = _make_feat_in()
BMI270_LEGACY_MAP_INT = _make_map_int()


# ---------------------------------------------------------------------------
# Initialisation
# ---------------------------------------------------------------------------

def bmi270_legacy_init(dev):
    """Initialise le BMI270 en variante Legacy.

    Returns:
        int8_t -- BMI2_OK ou code erreur.
    """
    if dev is None:
        return BMI2_E_NULL_PTR

    dev.chip_id = BMI270_LEGACY_CHIP_ID

    if isinstance(BMI270_LEGACY_CONFIG_FILE, str):
        dev.config_size = os.stat(BMI270_LEGACY_CONFIG_FILE)[6]
    else:
        dev.config_size = len(BMI270_LEGACY_CONFIG_FILE)

    dev.variant_feature = BMI2_CRT_RTOSK_ENABLE | BMI2_GYRO_CROSS_SENS_ENABLE
    dev.dummy_byte = 1 if dev.intf == BMI2_SPI_INTF else 0
    dev.config_file_ptr = BMI270_LEGACY_CONFIG_FILE

    rslt = bmi2_sec_init(dev)
    if rslt != BMI2_OK:
        return rslt

    dev.page_max     = BMI270_LEGACY_MAX_PAGE_NUM
    dev.input_sens   = BMI270_LEGACY_MAX_FEAT_IN
    dev.out_sens     = BMI270_LEGACY_MAX_FEAT_OUT
    dev.sens_int_map = BMI270_LEGACY_MAX_INT_MAP
    dev.variant_feature = 0
    dev.feat_config  = BMI270_LEGACY_FEAT_IN
    dev.map_int      = BMI270_LEGACY_MAP_INT

    return BMI2_OK


# ---------------------------------------------------------------------------
# Activation / désactivation
# ---------------------------------------------------------------------------

def _legacy_select_sensor(sens_list):
    sensor_sel = 0
    for s in sens_list:
        if   s == BMI2_ACCEL:        sensor_sel |= BMI2_ACCEL_SENS_SEL
        elif s == BMI2_GYRO:         sensor_sel |= BMI2_GYRO_SENS_SEL
        elif s == BMI2_AUX:          sensor_sel |= BMI2_AUX_SENS_SEL
        elif s == BMI2_ANY_MOTION:   sensor_sel |= BMI2_ANY_MOT_SEL
        elif s == BMI2_NO_MOTION:    sensor_sel |= BMI2_NO_MOT_SEL
        elif s == BMI2_SIG_MOTION:   sensor_sel |= BMI2_SIG_MOTION_SEL
        elif s == BMI2_STEP_DETECTOR: sensor_sel |= BMI2_STEP_DETECT_SEL
        elif s == BMI2_STEP_COUNTER: sensor_sel |= BMI2_STEP_COUNT_SEL
        elif s == BMI2_STEP_ACTIVITY: sensor_sel |= BMI2_STEP_ACT_SEL
        elif s == BMI2_GYRO_GAIN_UPDATE: sensor_sel |= BMI2_GYRO_GAIN_UPDATE_SEL
        elif s == BMI2_HIGH_G:       sensor_sel |= BMI2_HIGH_G_SEL
        elif s == BMI2_LOW_G:        sensor_sel |= BMI2_LOW_G_SEL
        elif s == BMI2_FLAT:         sensor_sel |= BMI2_FLAT_SEL
        elif s == BMI2_ORIENTATION:  sensor_sel |= BMI2_ORIENT_SEL
        elif s == BMI2_SINGLE_TAP:   sensor_sel |= BMI2_SINGLE_TAP_SEL
        elif s == BMI2_DOUBLE_TAP:   sensor_sel |= BMI2_DOUBLE_TAP_SEL
        elif s == BMI2_TRIPLE_TAP:   sensor_sel |= BMI2_TRIPLE_TAP_SEL
        else:
            return BMI2_E_INVALID_SENSOR, 0
    return BMI2_OK, sensor_sel


def _set_any_motion(enable, dev):
    fc = Bmi2FeatureConfig()
    if not bmi2_extract_input_feat_config(fc, BMI2_ANY_MOTION, dev):
        return BMI2_E_INVALID_SENSOR
    rslt, feat_config = bmi2_get_feat_config(fc.page, dev)
    if rslt != BMI2_OK:
        return rslt
    idx = fc.start_addr + BMI2_ANY_MOT_FEAT_EN_OFFSET
    feat_config[idx] = bmi2_set_bits(feat_config[idx], BMI2_ANY_NO_MOT_EN_MASK,
                                     BMI2_ANY_NO_MOT_EN_POS, enable)
    rslt = bmi2_set_regs(BMI2_FEATURES_REG_ADDR, feat_config, BMI2_FEAT_SIZE_IN_BYTES, dev)
    if rslt == BMI2_OK:
        if enable == BMI2_ENABLE:
            dev.sens_en_stat |= BMI2_ANY_MOT_SEL
        else:
            dev.sens_en_stat &= ~BMI2_ANY_MOT_SEL
    return rslt


def _set_no_motion(enable, dev):
    fc = Bmi2FeatureConfig()
    if not bmi2_extract_input_feat_config(fc, BMI2_NO_MOTION, dev):
        return BMI2_E_INVALID_SENSOR
    rslt, feat_config = bmi2_get_feat_config(fc.page, dev)
    if rslt != BMI2_OK:
        return rslt
    idx = fc.start_addr + BMI2_NO_MOT_FEAT_EN_OFFSET
    feat_config[idx] = bmi2_set_bits(feat_config[idx], BMI2_ANY_NO_MOT_EN_MASK,
                                     BMI2_ANY_NO_MOT_EN_POS, enable)
    rslt = bmi2_set_regs(BMI2_FEATURES_REG_ADDR, feat_config, BMI2_FEAT_SIZE_IN_BYTES, dev)
    if rslt == BMI2_OK:
        if enable == BMI2_ENABLE:
            dev.sens_en_stat |= BMI2_NO_MOT_SEL
        else:
            dev.sens_en_stat &= ~BMI2_NO_MOT_SEL
    return rslt


def _set_sig_motion(enable, dev):
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


def _set_step_features(enable, dev):
    fc = Bmi2FeatureConfig()
    if not bmi2_extract_input_feat_config(fc, BMI2_STEP_DETECTOR, dev):
        return BMI2_E_INVALID_SENSOR
    rslt, feat_config = bmi2_get_feat_config(fc.page, dev)
    if rslt != BMI2_OK:
        return rslt
    idx = fc.start_addr + BMI2_STEP_COUNT_FEAT_EN_OFFSET
    feat_config[idx] = bmi2_set_bits(feat_config[idx], BMI2_STEP_DET_FEAT_EN_MASK,
                                     BMI2_STEP_DET_FEAT_EN_POS, enable)
    feat_config[idx] = bmi2_set_bits(feat_config[idx], BMI2_STEP_COUNT_FEAT_EN_MASK,
                                     BMI2_STEP_COUNT_FEAT_EN_POS, enable)
    feat_config[idx] = bmi2_set_bits(feat_config[idx], BMI2_STEP_ACT_FEAT_EN_MASK,
                                     BMI2_STEP_ACT_FEAT_EN_POS, enable)
    return bmi2_set_regs(BMI2_FEATURES_REG_ADDR, feat_config, BMI2_FEAT_SIZE_IN_BYTES, dev)


def bmi270_legacy_sensor_enable(sens_list, dev):
    """Active les capteurs/features listés.

    Returns:
        int8_t -- BMI2_OK ou code erreur.
    """
    if dev is None:
        return BMI2_E_NULL_PTR

    rslt, sensor_sel = _legacy_select_sensor(sens_list)
    if rslt != BMI2_OK:
        return rslt

    for s in sens_list:
        if s == BMI2_ANY_MOTION:
            rslt = _set_any_motion(BMI2_ENABLE, dev)
        elif s == BMI2_NO_MOTION:
            rslt = _set_no_motion(BMI2_ENABLE, dev)
        elif s == BMI2_SIG_MOTION:
            rslt = _set_sig_motion(BMI2_ENABLE, dev)
        elif s in (BMI2_STEP_DETECTOR, BMI2_STEP_COUNTER, BMI2_STEP_ACTIVITY):
            rslt = _set_step_features(BMI2_ENABLE, dev)
        if rslt != BMI2_OK:
            return rslt

    return _sensor_enable(sensor_sel, dev)


def bmi270_legacy_sensor_disable(sens_list, dev):
    """Désactive les capteurs/features listés.

    Returns:
        int8_t -- BMI2_OK ou code erreur.
    """
    if dev is None:
        return BMI2_E_NULL_PTR

    rslt, sensor_sel = _legacy_select_sensor(sens_list)
    if rslt != BMI2_OK:
        return rslt

    for s in sens_list:
        if s == BMI2_ANY_MOTION:
            rslt = _set_any_motion(BMI2_DISABLE, dev)
        elif s == BMI2_NO_MOTION:
            rslt = _set_no_motion(BMI2_DISABLE, dev)
        elif s == BMI2_SIG_MOTION:
            rslt = _set_sig_motion(BMI2_DISABLE, dev)
        elif s in (BMI2_STEP_DETECTOR, BMI2_STEP_COUNTER, BMI2_STEP_ACTIVITY):
            rslt = _set_step_features(BMI2_DISABLE, dev)
        if rslt != BMI2_OK:
            return rslt

    return _sensor_disable(sensor_sel, dev)


# ---------------------------------------------------------------------------
# Configuration capteurs
# ---------------------------------------------------------------------------

def bmi270_legacy_set_sensor_config(sens_cfg_list, dev):
    """Configure les capteurs/features.

    Returns:
        int8_t -- BMI2_OK ou code erreur.
    """
    if dev is None:
        return BMI2_E_NULL_PTR
    return bmi2_set_sensor_config(sens_cfg_list, dev)


def bmi270_legacy_get_sensor_config(sens_cfg_list, dev):
    """Lit la configuration des capteurs/features.

    Returns:
        int8_t -- BMI2_OK ou code erreur.
    """
    if dev is None:
        return BMI2_E_NULL_PTR
    return bmi2_get_sensor_config(sens_cfg_list, dev)


# ---------------------------------------------------------------------------
# Données de features
# ---------------------------------------------------------------------------

def bmi270_legacy_get_feature_data(feat_data_list, dev):
    """Lit les données de sortie des features depuis la page 0.

    feat_data_list : liste de Bmi2FeatSensorData avec .type renseigné.

    Returns:
        int8_t -- BMI2_OK ou code erreur.
    """
    if dev is None:
        return BMI2_E_NULL_PTR

    rslt, page0 = bmi2_get_feat_config(BMI2_PAGE_0, dev)
    if rslt != BMI2_OK:
        return rslt

    for fd in feat_data_list:
        if fd.type == BMI2_STEP_COUNTER:
            idx = BMI270_LEGACY_STEP_CNT_OUT_STRT_ADDR
            fd.sens_data.step_counter_output = (
                page0[idx] | (page0[idx+1] << 8) |
                (page0[idx+2] << 16) | (page0[idx+3] << 24)
            )
        elif fd.type == BMI2_STEP_ACTIVITY:
            idx = BMI270_LEGACY_STEP_ACT_OUT_STRT_ADDR
            fd.sens_data.activity_output = page0[idx] & 0x03
        elif fd.type in (BMI2_ORIENTATION, BMI2_HIGH_G):
            idx = BMI270_LEGACY_ORIENT_HIGH_G_OUT_STRT_ADDR
            fd.sens_data.activity_output = page0[idx]
        elif fd.type == BMI2_GYRO_GAIN_UPDATE:
            idx = BMI270_LEGACY_GYR_USER_GAIN_OUT_STRT_ADDR
            fd.sens_data.gyro_user_gain_status = page0[idx]
        else:
            return BMI2_E_INVALID_SENSOR

    return BMI2_OK


# ---------------------------------------------------------------------------
# Gain utilisateur gyroscope (fonctions spécifiques à la variante legacy)
# ---------------------------------------------------------------------------

# Constantes pour le champ GYRO_GAIN_UPDATE (depuis bmi2_defs.h)
_BMI2_GYR_USER_GAIN_X_MASK  = 0x7F
_BMI2_GYR_USER_GAIN_Y_MASK  = 0x7F
_BMI2_GYR_USER_GAIN_Z_MASK  = 0x7F


def bmi270_legacy_update_gyro_user_gain(user_gain, dev):
    """Met à jour le gain utilisateur du gyroscope.

    user_gain : objet Bmi2GyroUserGainConfig avec ratio_x, ratio_y, ratio_z.

    Returns:
        int8_t -- BMI2_OK ou code erreur.
    """
    if dev is None:
        return BMI2_E_NULL_PTR

    fc = Bmi2FeatureConfig()
    if not bmi2_extract_input_feat_config(fc, BMI2_GYRO_GAIN_UPDATE, dev):
        return BMI2_E_INVALID_SENSOR

    rslt, feat_config = bmi2_get_feat_config(fc.page, dev)
    if rslt != BMI2_OK:
        return rslt

    idx = fc.start_addr
    feat_config[idx]   = user_gain.ratio_x & _BMI2_GYR_USER_GAIN_X_MASK
    feat_config[idx+1] = user_gain.ratio_y & _BMI2_GYR_USER_GAIN_Y_MASK
    feat_config[idx+2] = user_gain.ratio_z & _BMI2_GYR_USER_GAIN_Z_MASK

    return bmi2_set_regs(BMI2_FEATURES_REG_ADDR, feat_config, BMI2_FEAT_SIZE_IN_BYTES, dev)


def bmi270_legacy_read_gyro_user_gain(user_gain, dev):
    """Lit le gain utilisateur du gyroscope depuis la page de sortie.

    user_gain : objet Bmi2GyroUserGainData mis à jour en place.

    Returns:
        int8_t -- BMI2_OK ou code erreur.
    """
    if dev is None:
        return BMI2_E_NULL_PTR

    rslt, page0 = bmi2_get_feat_config(BMI2_PAGE_0, dev)
    if rslt != BMI2_OK:
        return rslt

    idx = BMI270_LEGACY_GYR_USER_GAIN_OUT_STRT_ADDR
    user_gain.ratio_x = page0[idx]   & _BMI2_GYR_USER_GAIN_X_MASK
    user_gain.ratio_y = page0[idx+1] & _BMI2_GYR_USER_GAIN_Y_MASK
    user_gain.ratio_z = page0[idx+2] & _BMI2_GYR_USER_GAIN_Z_MASK
    return BMI2_OK


# ---------------------------------------------------------------------------
# Mapping d'interruptions
# ---------------------------------------------------------------------------

def bmi270_legacy_map_feat_int(type_val, hw_int_pin, dev):
    """Mappe une feature sur une broche d'interruption matérielle.

    Returns:
        int8_t -- BMI2_OK ou code erreur.
    """
    if dev is None:
        return BMI2_E_NULL_PTR
    return bmi2_map_feat_int(type_val, hw_int_pin, dev)
