"""bmi270_context.py -- Port MicroPython du driver Bosch BMI270 variante Context.

Source : bmi270_context.h + bmi270_context.c

10 fonctions publiques : bmi270_context_init, sensor_enable, sensor_disable,
set_sensor_config, get_sensor_config, get_feature_data, get_act_recg_sett,
set_act_recg_sett, get_act_recog_output, map_feat_int.
"""

import os

from bmi2_defs import (
    BMI2_OK, BMI2_E_NULL_PTR, BMI2_E_INVALID_SENSOR,
    BMI2_SPI_INTF, BMI2_ENABLE, BMI2_DISABLE,
    BMI2_ACCEL, BMI2_GYRO, BMI2_AUX,
    BMI2_ACCEL_SENS_SEL, BMI2_GYRO_SENS_SEL, BMI2_AUX_SENS_SEL,
    BMI2_STEP_DETECTOR, BMI2_STEP_COUNTER,
    BMI2_STEP_DETECT_SEL, BMI2_STEP_COUNT_SEL,
    BMI2_GYRO_GAIN_UPDATE, BMI2_GYRO_GAIN_UPDATE_SEL,
    BMI2_ACTIVITY_RECOGNITION, BMI2_ACTIVITY_RECOGNITION_SEL,
    BMI2_ACTIVITY_RECOGNITION_SETTINGS,
    BMI2_ACTIVITY_RECOG_EN_MASK,
    BMI2_CONFIG_ID, BMI2_MAX_BURST_LEN, BMI2_CRT_GYRO_SELF_TEST,
    BMI2_ABORT_CRT_GYRO_SELF_TEST, BMI2_NVM_PROG_PREP,
    BMI2_STEP_COUNTER_PARAMS,
    BMI2_GYRO_CROSS_SENS_ENABLE, BMI2_CRT_RTOSK_ENABLE,
    BMI2_PAGE_0, BMI2_PAGE_1, BMI2_PAGE_4, BMI2_PAGE_5,
    BMI2_FEAT_SIZE_IN_BYTES, BMI2_FEATURES_REG_ADDR,
    Bmi2FeatureConfig, Bmi2MapInt, Bmi2ActRecogOutput,
    bmi2_set_bit_pos0,
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
    bmi2_get_int_status,
)

# ---------------------------------------------------------------------------
# Constantes spécifiques BMI270_CONTEXT
# ---------------------------------------------------------------------------
BMI270_CONTEXT_CHIP_ID      = 0x24
BMI270_CONTEXT_MAX_PAGE_NUM = 8
BMI270_CONTEXT_MAX_FEAT_IN  = 10
BMI270_CONTEXT_MAX_FEAT_OUT = 5
BMI270_C_MAX_INT_MAP        = 2

# Feature input start addresses
BMI270_CONTEXT_CONFIG_ID_STRT_ADDR          = 0x06
BMI270_CONTEXT_STEP_CNT_1_STRT_ADDR         = 0x00
BMI270_CONTEXT_STEP_CNT_4_STRT_ADDR         = 0x02
BMI270_CONTEXT_MAX_BURST_LEN_STRT_ADDR      = 0x08
BMI270_CONTEXT_CRT_GYRO_SELF_TEST_STRT_ADDR = 0x09
BMI270_CONTEXT_ABORT_STRT_ADDR              = 0x09
BMI270_CONTEXT_NVM_PROG_PREP_STRT_ADDR      = 0x0A
BMI270_CONTEXT_ACT_RGN_SETT_STRT_ADDR       = 0x00
BMI270_CONTEXT_ACT_RGN_STRT_ADDR            = 0x0A

# Feature output start addresses (page 0)
BMI270_CONTEXT_STEP_CNT_OUT_STRT_ADDR     = 0x00
BMI270_CONTEXT_GYR_USER_GAIN_OUT_STRT_ADDR = 0x04
BMI270_CONTEXT_GYRO_CROSS_SENSE_STRT_ADDR  = 0x0C
BMI270_CONTEXT_NVM_VFRM_OUT_STRT_ADDR      = 0x0E

# Interrupt mapping masks
BMI270_C_INT_STEP_COUNTER_MASK  = 0x01
BMI270_C_INT_STEP_DETECTOR_MASK = 0x01

# ---------------------------------------------------------------------------
# Firmware
# ---------------------------------------------------------------------------
BMI270_CONTEXT_CONFIG_FILE = "/firmware/bmi270_config_context.bin"

# ---------------------------------------------------------------------------
# Tables de features
# ---------------------------------------------------------------------------

def _make_feat_in():
    data = [
        (BMI2_CONFIG_ID,                  BMI2_PAGE_4, BMI270_CONTEXT_CONFIG_ID_STRT_ADDR),
        (BMI2_STEP_COUNTER_PARAMS,        BMI2_PAGE_1, BMI270_CONTEXT_STEP_CNT_1_STRT_ADDR),
        (BMI2_STEP_DETECTOR,              BMI2_PAGE_4, BMI270_CONTEXT_STEP_CNT_4_STRT_ADDR),
        (BMI2_STEP_COUNTER,               BMI2_PAGE_4, BMI270_CONTEXT_STEP_CNT_4_STRT_ADDR),
        (BMI2_NVM_PROG_PREP,              BMI2_PAGE_4, BMI270_CONTEXT_NVM_PROG_PREP_STRT_ADDR),
        (BMI2_MAX_BURST_LEN,              BMI2_PAGE_4, BMI270_CONTEXT_MAX_BURST_LEN_STRT_ADDR),
        (BMI2_CRT_GYRO_SELF_TEST,         BMI2_PAGE_4, BMI270_CONTEXT_CRT_GYRO_SELF_TEST_STRT_ADDR),
        (BMI2_ABORT_CRT_GYRO_SELF_TEST,   BMI2_PAGE_4, BMI270_CONTEXT_ABORT_STRT_ADDR),
        (BMI2_ACTIVITY_RECOGNITION_SETTINGS, BMI2_PAGE_5, BMI270_CONTEXT_ACT_RGN_SETT_STRT_ADDR),
        (BMI2_ACTIVITY_RECOGNITION,       BMI2_PAGE_5, BMI270_CONTEXT_ACT_RGN_STRT_ADDR),
    ]
    result = []
    for t, p, s in data:
        fc = Bmi2FeatureConfig()
        fc.type = t; fc.page = p; fc.start_addr = s
        result.append(fc)
    return result


def _make_map_int():
    data = [
        (BMI2_STEP_COUNTER,  BMI270_C_INT_STEP_COUNTER_MASK),
        (BMI2_STEP_DETECTOR, BMI270_C_INT_STEP_DETECTOR_MASK),
    ]
    result = []
    for t, m in data:
        mi = Bmi2MapInt()
        mi.type = t; mi.sens_map_int = m
        result.append(mi)
    return result


BMI270_CONTEXT_FEAT_IN = _make_feat_in()
BMI270_C_MAP_INT       = _make_map_int()


# ---------------------------------------------------------------------------
# Initialisation
# ---------------------------------------------------------------------------

def bmi270_context_init(dev):
    """Initialise le BMI270 en variante Context.

    Returns:
        int8_t -- BMI2_OK ou code erreur.
    """
    if dev is None:
        return BMI2_E_NULL_PTR

    dev.chip_id = BMI270_CONTEXT_CHIP_ID

    if isinstance(BMI270_CONTEXT_CONFIG_FILE, str):
        dev.config_size = os.stat(BMI270_CONTEXT_CONFIG_FILE)[6]
    else:
        dev.config_size = len(BMI270_CONTEXT_CONFIG_FILE)

    dev.variant_feature = BMI2_CRT_RTOSK_ENABLE | BMI2_GYRO_CROSS_SENS_ENABLE
    dev.dummy_byte = 1 if dev.intf == BMI2_SPI_INTF else 0
    dev.config_file_ptr = BMI270_CONTEXT_CONFIG_FILE

    rslt = bmi2_sec_init(dev)
    if rslt != BMI2_OK:
        return rslt

    dev.page_max     = BMI270_CONTEXT_MAX_PAGE_NUM
    dev.input_sens   = BMI270_CONTEXT_MAX_FEAT_IN
    dev.out_sens     = BMI270_CONTEXT_MAX_FEAT_OUT
    dev.sens_int_map = BMI270_C_MAX_INT_MAP
    dev.variant_feature = 0
    dev.feat_config  = BMI270_CONTEXT_FEAT_IN
    dev.map_int      = BMI270_C_MAP_INT

    return BMI2_OK


# ---------------------------------------------------------------------------
# Activation / désactivation
# ---------------------------------------------------------------------------

def _context_select_sensor(sens_list):
    sensor_sel = 0
    for s in sens_list:
        if   s == BMI2_ACCEL:               sensor_sel |= BMI2_ACCEL_SENS_SEL
        elif s == BMI2_GYRO:                sensor_sel |= BMI2_GYRO_SENS_SEL
        elif s == BMI2_AUX:                 sensor_sel |= BMI2_AUX_SENS_SEL
        elif s == BMI2_STEP_DETECTOR:       sensor_sel |= BMI2_STEP_DETECT_SEL
        elif s == BMI2_STEP_COUNTER:        sensor_sel |= BMI2_STEP_COUNT_SEL
        elif s == BMI2_GYRO_GAIN_UPDATE:    sensor_sel |= BMI2_GYRO_GAIN_UPDATE_SEL
        elif s == BMI2_ACTIVITY_RECOGNITION: sensor_sel |= BMI2_ACTIVITY_RECOGNITION_SEL
        else:
            return BMI2_E_INVALID_SENSOR, 0
    return BMI2_OK, sensor_sel


def _set_activity_recognition(enable, dev):
    """Active ou désactive la feature activity recognition."""
    fc = Bmi2FeatureConfig()
    if not bmi2_extract_input_feat_config(fc, BMI2_ACTIVITY_RECOGNITION, dev):
        return BMI2_E_INVALID_SENSOR
    rslt, feat_config = bmi2_get_feat_config(fc.page, dev)
    if rslt != BMI2_OK:
        return rslt
    idx = fc.start_addr
    feat_config[idx] = bmi2_set_bit_pos0(feat_config[idx], BMI2_ACTIVITY_RECOG_EN_MASK, enable)
    rslt = bmi2_set_regs(BMI2_FEATURES_REG_ADDR, feat_config, BMI2_FEAT_SIZE_IN_BYTES, dev)
    if rslt == BMI2_OK:
        if enable == BMI2_ENABLE:
            dev.sens_en_stat |= BMI2_ACTIVITY_RECOGNITION_SEL
        else:
            dev.sens_en_stat &= ~BMI2_ACTIVITY_RECOGNITION_SEL
    return rslt


def bmi270_context_sensor_enable(sens_list, dev):
    """Active les capteurs/features listés.

    Returns:
        int8_t -- BMI2_OK ou code erreur.
    """
    if dev is None:
        return BMI2_E_NULL_PTR

    rslt, sensor_sel = _context_select_sensor(sens_list)
    if rslt != BMI2_OK:
        return rslt

    for s in sens_list:
        if s == BMI2_ACTIVITY_RECOGNITION:
            rslt = _set_activity_recognition(BMI2_ENABLE, dev)
            if rslt != BMI2_OK:
                return rslt

    return _sensor_enable(sensor_sel, dev)


def bmi270_context_sensor_disable(sens_list, dev):
    """Désactive les capteurs/features listés.

    Returns:
        int8_t -- BMI2_OK ou code erreur.
    """
    if dev is None:
        return BMI2_E_NULL_PTR

    rslt, sensor_sel = _context_select_sensor(sens_list)
    if rslt != BMI2_OK:
        return rslt

    for s in sens_list:
        if s == BMI2_ACTIVITY_RECOGNITION:
            rslt = _set_activity_recognition(BMI2_DISABLE, dev)
            if rslt != BMI2_OK:
                return rslt

    return _sensor_disable(sensor_sel, dev)


# ---------------------------------------------------------------------------
# Configuration capteurs
# ---------------------------------------------------------------------------

def bmi270_context_set_sensor_config(sens_cfg_list, dev):
    """Configure les capteurs/features.

    Returns:
        int8_t -- BMI2_OK ou code erreur.
    """
    if dev is None:
        return BMI2_E_NULL_PTR
    return bmi2_set_sensor_config(sens_cfg_list, dev)


def bmi270_context_get_sensor_config(sens_cfg_list, dev):
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

def bmi270_context_get_feature_data(feat_data_list, dev):
    """Lit les données de sortie des features (step counter, etc.).

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
            idx = BMI270_CONTEXT_STEP_CNT_OUT_STRT_ADDR
            fd.sens_data.step_counter_output = (
                page0[idx] | (page0[idx+1] << 8) |
                (page0[idx+2] << 16) | (page0[idx+3] << 24)
            )
        elif fd.type == BMI2_GYRO_GAIN_UPDATE:
            idx = BMI270_CONTEXT_GYR_USER_GAIN_OUT_STRT_ADDR
            fd.sens_data.gyro_user_gain_status = page0[idx]
        else:
            return BMI2_E_INVALID_SENSOR

    return BMI2_OK


# ---------------------------------------------------------------------------
# Reconnaissance d'activité
# ---------------------------------------------------------------------------

def bmi270_context_get_act_recg_sett(sett, dev):
    """Lit les paramètres de reconnaissance d'activité.

    sett : objet avec les champs min_segment_conf, etc. (mis à jour en place).

    Returns:
        int8_t -- BMI2_OK ou code erreur.
    """
    if dev is None:
        return BMI2_E_NULL_PTR

    fc = Bmi2FeatureConfig()
    if not bmi2_extract_input_feat_config(fc, BMI2_ACTIVITY_RECOGNITION_SETTINGS, dev):
        return BMI2_E_INVALID_SENSOR

    rslt, feat_config = bmi2_get_feat_config(fc.page, dev)
    if rslt != BMI2_OK:
        return rslt

    idx = fc.start_addr
    # Les paramètres sont stockés en mots de 2 octets little-endian
    sett.postproc_timeout = feat_config[idx] | (feat_config[idx+1] << 8); idx += 2
    sett.min_segment_conf = feat_config[idx] | (feat_config[idx+1] << 8); idx += 2
    return BMI2_OK


def bmi270_context_set_act_recg_sett(sett, dev):
    """Écrit les paramètres de reconnaissance d'activité.

    Returns:
        int8_t -- BMI2_OK ou code erreur.
    """
    if dev is None:
        return BMI2_E_NULL_PTR

    fc = Bmi2FeatureConfig()
    if not bmi2_extract_input_feat_config(fc, BMI2_ACTIVITY_RECOGNITION_SETTINGS, dev):
        return BMI2_E_INVALID_SENSOR

    rslt, feat_config = bmi2_get_feat_config(fc.page, dev)
    if rslt != BMI2_OK:
        return rslt

    idx = fc.start_addr
    feat_config[idx]   = sett.postproc_timeout & 0xFF
    feat_config[idx+1] = (sett.postproc_timeout >> 8) & 0xFF
    feat_config[idx+2] = sett.min_segment_conf & 0xFF
    feat_config[idx+3] = (sett.min_segment_conf >> 8) & 0xFF

    return bmi2_set_regs(BMI2_FEATURES_REG_ADDR, feat_config, BMI2_FEAT_SIZE_IN_BYTES, dev)


def bmi270_context_get_act_recog_output(act_recog_list, dev):
    """Lit les sorties de reconnaissance d'activité (depuis la page feature out).

    act_recog_list : liste de Bmi2ActRecogOutput à remplir.

    Returns:
        int8_t -- BMI2_OK ou code erreur.
    """
    if dev is None:
        return BMI2_E_NULL_PTR

    rslt, page0 = bmi2_get_feat_config(BMI2_PAGE_0, dev)
    if rslt != BMI2_OK:
        return rslt

    for out in act_recog_list:
        idx = BMI270_CONTEXT_STEP_CNT_OUT_STRT_ADDR
        out.time_stamp = (page0[idx] | (page0[idx+1] << 8) |
                          (page0[idx+2] << 16) | (page0[idx+3] << 24))
        out.curr_act = page0[idx+4] & 0x0F
        out.prev_act = (page0[idx+4] >> 4) & 0x0F

    return BMI2_OK


# ---------------------------------------------------------------------------
# Mapping d'interruptions
# ---------------------------------------------------------------------------

def bmi270_context_map_feat_int(type_val, hw_int_pin, dev):
    """Mappe une feature sur une broche d'interruption matérielle.

    Returns:
        int8_t -- BMI2_OK ou code erreur.
    """
    if dev is None:
        return BMI2_E_NULL_PTR
    return bmi2_map_feat_int(type_val, hw_int_pin, dev)
