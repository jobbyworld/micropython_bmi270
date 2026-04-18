"""bmi2_ois.py -- Port MicroPython du driver OIS Bosch BMI2 (bmi2_ois.h + bmi2_ois.c).

Interface OIS (Optical Image Stabilization) sur bus SPI secondaire.
5 fonctions publiques : bmi2_ois_get_regs, bmi2_ois_set_regs,
bmi2_ois_set_config, bmi2_ois_get_config, bmi2_ois_read_data.
"""

# ---------------------------------------------------------------------------
# Constantes
# ---------------------------------------------------------------------------
BMI2_OIS_OK              = 0
BMI2_OIS_E_NULL_PTR      = -1
BMI2_OIS_E_COM_FAIL      = -2
BMI2_OIS_E_INVALID_SENSOR = -8

BMI2_OIS_SPI_RD_MASK = 0x80
BMI2_OIS_SPI_WR_MASK = 0x7F
BMI2_OIS_ACC_GYR_NUM_BYTES = 6

BMI2_OIS_ACCEL = 0x01
BMI2_OIS_GYRO  = 0x02

BMI2_OIS_CONFIG_ADDR    = 0x40
BMI2_OIS_ACC_X_LSB_ADDR = 0x0C
BMI2_OIS_GYR_X_LSB_ADDR = 0x12

BMI2_OIS_GYR_EN_MASK = 0x40
BMI2_OIS_GYR_EN_POS  = 6
BMI2_OIS_ACC_EN_MASK = 0x80
BMI2_OIS_ACC_EN_POS  = 7

BMI2_OIS_LP_FILTER_EN_POS      = 0
BMI2_OIS_LP_FILTER_EN_MASK     = 0x01
BMI2_OIS_LP_FILTER_CONFIG_POS  = 1
BMI2_OIS_LP_FILTER_CONFIG_MASK = 0x06
BMI2_OIS_LP_FILTER_MUTE_POS    = 5
BMI2_OIS_LP_FILTER_MUTE_MASK   = 0x20


# ---------------------------------------------------------------------------
# Classes
# ---------------------------------------------------------------------------

class Bmi2OisSensAxesData:
    """Données d'axes capteur OIS (struct bmi2_ois_sens_axes_data)."""
    def __init__(self):
        self.x = 0
        self.y = 0
        self.z = 0


class Bmi2OisDev:
    """Device OIS (struct bmi2_ois_dev).

    Attributs à renseigner avant utilisation :
        ois_read(reg_addr, length) -> bytes
        ois_write(reg_addr, data: bytes)
        ois_delay_us(period_us)
    """
    def __init__(self):
        self.ois_read     = None
        self.ois_write    = None
        self.ois_delay_us = None
        self.lp_filter_en     = 0
        self.lp_filter_config = 0
        self.lp_filter_mute   = 0
        self.acc_en   = 0
        self.gyr_en   = 0
        self.acc_data = Bmi2OisSensAxesData()
        self.gyr_data = Bmi2OisSensAxesData()
        self.intf_ptr  = None
        self.intf_rslt = 0
        # Compensation cross-axe gyroscope (lue depuis le registre NV du BMI270 principal)
        self.cross_sens_zx = 0


# ---------------------------------------------------------------------------
# Fonctions internes
# ---------------------------------------------------------------------------

def _to_signed_16(val):
    return val - 0x10000 if val >= 0x8000 else val


# ---------------------------------------------------------------------------
# Fonctions publiques
# ---------------------------------------------------------------------------

def bmi2_ois_get_regs(ois_reg_addr, length, ois_dev):
    """Lit length octets depuis ois_reg_addr via l'interface OIS SPI.

    Returns:
        (int, bytes | None) -- (BMI2_OIS_OK, données) ou (code_erreur, None).
    """
    if ois_dev is None or ois_dev.ois_read is None:
        return BMI2_OIS_E_NULL_PTR, None

    reg_addr = ois_reg_addr | BMI2_OIS_SPI_RD_MASK
    try:
        data = ois_dev.ois_read(reg_addr, length + 1)  # +1 octet dummy SPI
        ois_dev.intf_rslt = 0
    except Exception:
        ois_dev.intf_rslt = -1
        return BMI2_OIS_E_COM_FAIL, None

    if len(data) < length + 1:
        return BMI2_OIS_E_COM_FAIL, None
    return BMI2_OIS_OK, data[1:]  # supprime l'octet dummy


def bmi2_ois_set_regs(ois_reg_addr, data, ois_dev):
    """Écrit data vers ois_reg_addr via l'interface OIS SPI.

    Returns:
        int -- BMI2_OIS_OK ou code erreur.
    """
    if ois_dev is None or ois_dev.ois_write is None:
        return BMI2_OIS_E_NULL_PTR

    reg_addr = ois_reg_addr & BMI2_OIS_SPI_WR_MASK
    try:
        ois_dev.ois_write(reg_addr, data)
        ois_dev.intf_rslt = 0
    except Exception:
        ois_dev.intf_rslt = -1
        return BMI2_OIS_E_COM_FAIL
    return BMI2_OIS_OK


def bmi2_ois_set_config(ois_dev):
    """Configure l'interface OIS (activation acc/gyro, filtre LP).

    Les champs acc_en, gyr_en, lp_filter_en, lp_filter_config, lp_filter_mute
    de ois_dev sont lus et écrits dans le registre BMI2_OIS_CONFIG_ADDR.

    Returns:
        int -- BMI2_OIS_OK ou code erreur.
    """
    if ois_dev is None:
        return BMI2_OIS_E_NULL_PTR

    rslt, data = bmi2_ois_get_regs(BMI2_OIS_CONFIG_ADDR, 1, ois_dev)
    if rslt != BMI2_OIS_OK:
        return rslt

    reg = data[0]
    reg = (reg & ~BMI2_OIS_GYR_EN_MASK) | ((ois_dev.gyr_en << BMI2_OIS_GYR_EN_POS) & BMI2_OIS_GYR_EN_MASK)
    reg = (reg & ~BMI2_OIS_ACC_EN_MASK) | ((ois_dev.acc_en << BMI2_OIS_ACC_EN_POS) & BMI2_OIS_ACC_EN_MASK)
    reg = (reg & ~BMI2_OIS_LP_FILTER_EN_MASK)     | ((ois_dev.lp_filter_en     << BMI2_OIS_LP_FILTER_EN_POS)     & BMI2_OIS_LP_FILTER_EN_MASK)
    reg = (reg & ~BMI2_OIS_LP_FILTER_CONFIG_MASK) | ((ois_dev.lp_filter_config << BMI2_OIS_LP_FILTER_CONFIG_POS) & BMI2_OIS_LP_FILTER_CONFIG_MASK)
    reg = (reg & ~BMI2_OIS_LP_FILTER_MUTE_MASK)   | ((ois_dev.lp_filter_mute   << BMI2_OIS_LP_FILTER_MUTE_POS)   & BMI2_OIS_LP_FILTER_MUTE_MASK)

    return bmi2_ois_set_regs(BMI2_OIS_CONFIG_ADDR, bytes([reg & 0xFF]), ois_dev)


def bmi2_ois_get_config(ois_dev):
    """Lit la configuration OIS et met à jour les champs de ois_dev.

    Returns:
        int -- BMI2_OIS_OK ou code erreur.
    """
    if ois_dev is None:
        return BMI2_OIS_E_NULL_PTR

    rslt, data = bmi2_ois_get_regs(BMI2_OIS_CONFIG_ADDR, 1, ois_dev)
    if rslt != BMI2_OIS_OK:
        return rslt

    reg = data[0]
    ois_dev.gyr_en          = (reg & BMI2_OIS_GYR_EN_MASK)          >> BMI2_OIS_GYR_EN_POS
    ois_dev.acc_en          = (reg & BMI2_OIS_ACC_EN_MASK)          >> BMI2_OIS_ACC_EN_POS
    ois_dev.lp_filter_en    = (reg & BMI2_OIS_LP_FILTER_EN_MASK)    >> BMI2_OIS_LP_FILTER_EN_POS
    ois_dev.lp_filter_config = (reg & BMI2_OIS_LP_FILTER_CONFIG_MASK) >> BMI2_OIS_LP_FILTER_CONFIG_POS
    ois_dev.lp_filter_mute  = (reg & BMI2_OIS_LP_FILTER_MUTE_MASK)  >> BMI2_OIS_LP_FILTER_MUTE_POS
    return BMI2_OIS_OK


def bmi2_ois_read_data(sens_sel, ois_dev):
    """Lit les données acc/gyro via l'interface OIS SPI secondaire.

    sens_sel : BMI2_OIS_ACCEL, BMI2_OIS_GYRO, ou les deux OR-és.
    Les données lues sont stockées dans ois_dev.acc_data et ois_dev.gyr_data.

    Returns:
        int -- BMI2_OIS_OK ou code erreur.
    """
    if ois_dev is None:
        return BMI2_OIS_E_NULL_PTR
    if not (sens_sel & (BMI2_OIS_ACCEL | BMI2_OIS_GYRO)):
        return BMI2_OIS_E_INVALID_SENSOR

    if sens_sel & BMI2_OIS_ACCEL:
        rslt, data = bmi2_ois_get_regs(BMI2_OIS_ACC_X_LSB_ADDR,
                                        BMI2_OIS_ACC_GYR_NUM_BYTES, ois_dev)
        if rslt != BMI2_OIS_OK:
            return rslt
        ois_dev.acc_data.x = _to_signed_16(data[0] | (data[1] << 8))
        ois_dev.acc_data.y = _to_signed_16(data[2] | (data[3] << 8))
        ois_dev.acc_data.z = _to_signed_16(data[4] | (data[5] << 8))

    if sens_sel & BMI2_OIS_GYRO:
        rslt, data = bmi2_ois_get_regs(BMI2_OIS_GYR_X_LSB_ADDR,
                                        BMI2_OIS_ACC_GYR_NUM_BYTES, ois_dev)
        if rslt != BMI2_OIS_OK:
            return rslt
        gx = _to_signed_16(data[0] | (data[1] << 8))
        gy = _to_signed_16(data[2] | (data[3] << 8))
        gz = _to_signed_16(data[4] | (data[5] << 8))
        # Compensation cross-axe ZX (lue depuis les NV params du BMI270 principal)
        ois_dev.gyr_data.x = gx - (ois_dev.cross_sens_zx * gz) // 512
        ois_dev.gyr_data.y = gy
        ois_dev.gyr_data.z = gz

    return BMI2_OIS_OK
