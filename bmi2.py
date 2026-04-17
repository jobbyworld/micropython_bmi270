"""bmi2.py -- Port MicroPython du driver Bosch BMI2 (bmi2.h + bmi2.c).

Ce module implemente le sous-ensemble des fonctions necessaires a la lecture
basique de l'accelerometre et du gyroscope (cf. Example01_BasicReadingsI2C).

Les fonctions non necessaires a ce cas d'usage (FIFO, AUX, self-test, FOC,
CRT, NVM, OIS, features avancees) ne sont pas implementees. Elles peuvent
etre ajoutees ulterieurement en suivant la meme convention de conversion.

Conventions de conversion (voir CLAUDE.md a la racine du depot) :
- struct bmi2_dev -> class Bmi2Dev (dans ce fichier)
- struct bmi2_sens_data/sens_axes_data/sens_config/accel_config/gyro_config
  etc. -> classes dans bmi2_defs.py
- #define BMI2_xxx -> constantes dans bmi2_defs.py
- Macros BMI2_SET_BITS / BMI2_GET_BITS -> fonctions bmi2_set_bits / bmi2_get_bits
  dans bmi2_defs.py (avec mask et pos passes explicitement)
- Pointeurs de fonctions dev->read/write/delay_us -> attributs callables (None
  par defaut) assignes par le driver haut niveau.
"""

from bmi2_defs import (
    # Codes de retour
    BMI2_OK, BMI2_E_NULL_PTR, BMI2_E_COM_FAIL, BMI2_E_DEV_NOT_FOUND,
    BMI2_E_INVALID_SENSOR, BMI2_E_SET_APS_FAIL, BMI2_E_CONFIG_LOAD,
    BMI2_E_INVALID_PAGE, BMI2_E_INVALID_INT_PIN, BMI2_E_INVALID_INPUT,
    # Interfaces
    BMI2_I2C_INTF, BMI2_SPI_INTF,
    # Constantes generales
    BMI2_ENABLE, BMI2_DISABLE,
    BMI2_INTF_RET_SUCCESS,
    BMI2_POWER_SAVE_MODE_DELAY_IN_US, BMI2_NORMAL_MODE_DELAY_IN_US,
    BMI2_INTERNAL_STATUS_READ_DELAY_MS,
    # Adresses registres
    BMI2_CHIP_ID_ADDR, BMI2_STATUS_ADDR,
    BMI2_ACC_CONF_ADDR, BMI2_GYR_CONF_ADDR,
    BMI2_INIT_CTRL_ADDR, BMI2_INIT_ADDR_0, BMI2_INIT_DATA_ADDR,
    BMI2_INTERNAL_STATUS_ADDR,
    BMI2_PWR_CONF_ADDR, BMI2_PWR_CTRL_ADDR, BMI2_CMD_REG_ADDR,
    # Adresses registres feature / interruption
    BMI2_FEAT_PAGE_ADDR, BMI2_FEATURES_REG_ADDR,
    BMI2_INT1_IO_CTRL_ADDR, BMI2_INT_STATUS_0_ADDR,
    BMI2_INT1_MAP_FEAT_ADDR, BMI2_INT2_MAP_FEAT_ADDR, BMI2_INT_MAP_DATA_ADDR,
    # Commandes
    BMI2_SOFT_RESET_CMD,
    # SPI masques
    BMI2_SPI_RD_MASK, BMI2_SPI_WR_MASK,
    # Config load
    BMI2_CONFIG_LOAD_SUCCESS, BMI2_CONFIG_LOAD_STATUS_MASK,
    # Masques / positions - power configuration
    BMI2_ADV_POW_EN_MASK,
    # Masques / positions - init ctrl
    BMI2_CONF_LOAD_EN_MASK,
    # Masques / positions - PWR_CTRL
    BMI2_AUX_EN_MASK,
    BMI2_GYR_EN_MASK, BMI2_GYR_EN_POS,
    BMI2_ACC_EN_MASK, BMI2_ACC_EN_POS,
    BMI2_TEMP_EN_MASK, BMI2_TEMP_EN_POS,
    # Masques / positions - ACC_CONF / ACC_RANGE
    BMI2_ACC_ODR_MASK,
    BMI2_ACC_BW_PARAM_MASK, BMI2_ACC_BW_PARAM_POS,
    BMI2_ACC_FILTER_PERF_MODE_MASK, BMI2_ACC_FILTER_PERF_MODE_POS,
    BMI2_ACC_RANGE_MASK,
    # Masques / positions - GYR_CONF / GYR_RANGE
    BMI2_GYR_ODR_MASK,
    BMI2_GYR_BW_PARAM_MASK, BMI2_GYR_BW_PARAM_POS,
    BMI2_GYR_NOISE_PERF_MODE_MASK, BMI2_GYR_NOISE_PERF_MODE_POS,
    BMI2_GYR_FILTER_PERF_MODE_MASK, BMI2_GYR_FILTER_PERF_MODE_POS,
    BMI2_GYR_RANGE_MASK,
    BMI2_GYR_OIS_RANGE_MASK, BMI2_GYR_OIS_RANGE_POS,
    # Selection / types de capteurs
    BMI2_ACCEL, BMI2_GYRO, BMI2_AUX, BMI2_TEMP,
    BMI2_ACCEL_SENS_SEL, BMI2_GYRO_SENS_SEL, BMI2_AUX_SENS_SEL,
    BMI2_TEMP_SENS_SEL, BMI2_MAIN_SENSORS,
    # Tailles
    BMI2_ACC_GYR_AUX_SENSORTIME_NUM_BYTES,
    BMI2_AUX_NUM_BYTES, BMI2_AUX_START_INDEX,
    BMI2_ACC_START_INDEX, BMI2_GYR_START_INDEX,
    BMI2_MAX_LEN, BMI2_FEAT_SIZE_IN_BYTES,
    # Remap
    BMI2_MAP_X_AXIS, BMI2_MAP_Y_AXIS, BMI2_MAP_Z_AXIS,
    BMI2_MAP_POSITIVE, BMI2_MAP_NEGATIVE,
    BMI2_POS_SIGN, BMI2_NEG_SIGN,
    BMI2_AXIS_MAP,
    BMI2_AXIS_MASK, BMI2_AXIS_SIGN,
    BMI2_X_AXIS_MASK, BMI2_X_AXIS_SIGN_MASK, BMI2_X_AXIS_SIGN_POS,
    BMI2_Y_AXIS_MASK, BMI2_Y_AXIS_POS, BMI2_Y_AXIS_SIGN_MASK, BMI2_Y_AXIS_SIGN_POS,
    BMI2_Z_AXIS_MASK, BMI2_Z_AXIS_POS, BMI2_Z_AXIS_SIGN_MASK,
    BMI2_E_REMAP_ERROR,
    # Interruption pin
    BMI2_INT_NONE, BMI2_INT1, BMI2_INT2, BMI2_INT_BOTH, BMI2_INT_PIN_MAX,
    BMI2_INT_LATCH_MASK,
    BMI2_INT_LEVEL_MASK, BMI2_INT_LEVEL_POS,
    BMI2_INT_OPEN_DRAIN_MASK, BMI2_INT_OPEN_DRAIN_POS,
    BMI2_INT_OUTPUT_EN_MASK, BMI2_INT_OUTPUT_EN_POS,
    BMI2_INT_INPUT_EN_MASK, BMI2_INT_INPUT_EN_POS,
    # Utilitaires macros -> fonctions
    bmi2_set_bits, bmi2_get_bits,
    bmi2_set_bit_pos0, bmi2_get_bit_pos0, bmi2_set_bit_val0,
    # Classes
    Bmi2AxesRemap, Bmi2SensData, Bmi2SensAxesData,
    Bmi2AccelConfig, Bmi2GyroConfig, Bmi2FeatureConfig,
)


# ---------------------------------------------------------------------------
# Classe device (equivalente a struct bmi2_dev de bmi2_defs.h)
# ---------------------------------------------------------------------------

class Bmi2Dev:
    """Structure de configuration/etat du capteur BMI2.

    Les attributs `read`, `write`, `delay_us` sont des callables initialises
    a None. Ils doivent etre assignes par le driver haut niveau avant appel
    des fonctions d'initialisation :

        dev.read     = callable(reg_addr, length) -> bytes/bytearray (len)
        dev.write    = callable(reg_addr, data: bytes/bytearray) -> int (0 = OK)
        dev.delay_us = callable(period_us)
    """

    def __init__(self):
        # Chip id attendu / lu
        self.chip_id = 0
        # Interface utilisateur (non utilise en I2C pur)
        self.intf_ptr = None
        # Warnings
        self.info = 0
        # Interface (I2C/SPI)
        self.intf = BMI2_I2C_INTF
        # Resultat retour bas niveau
        self.intf_rslt = BMI2_INTF_RET_SUCCESS
        # Byte muet (SPI = 1, I2C = 0)
        self.dummy_byte = 0
        # Resolution (16 bits sur BMI270)
        self.resolution = 16
        # Longueur max burst read/write pour le firmware
        self.read_write_len = 32
        # Statut de chargement config
        self.load_status = 0
        # Pointeur vers le blob firmware
        self.config_file_ptr = None
        # Nombre max de pages features
        self.page_max = 0
        # Nombre de features en entree / sortie
        self.input_sens = 0
        self.out_sens = 0
        # Mode auto / manuel pour le bus AUX
        self.aux_man_en = 1
        self.aux_man_rd_burst_len = 0
        # Tableau des features configurables / de sortie (pas utilise en MVP)
        self.feat_config = None
        self.feat_output = None
        # Remap des axes
        self.remap = Bmi2AxesRemap()
        # Etat d'enablement des capteurs (bitfield)
        self.sens_en_stat = 0
        # Callbacks
        self.read = None       # callable(reg_addr, length) -> bytes
        self.write = None      # callable(reg_addr, bytes) -> int
        self.delay_us = None   # callable(period_us)
        # Cross-sensitivite gyro
        self.gyr_cross_sens_zx = 0
        # Statut enable gyro (flag)
        self.gyro_en = 0
        # Statut advance power save
        self.aps_status = BMI2_ENABLE
        # Features specifiques variante
        self.variant_feature = 0
        # Taille du blob config
        self.config_size = 0
        # Callbacks wake-up / tap (non utilises)
        self.get_wakeup_config = None
        self.set_wakeup_config = None
        self.get_tap_config = None
        self.set_tap_config = None
        # Table de mapping des interruptions
        self.map_int = None
        self.sens_int_map = 0


# ---------------------------------------------------------------------------
# Utilitaires
# ---------------------------------------------------------------------------

def _null_ptr_check(dev):
    """Equivalent de static int8_t null_ptr_check(const struct bmi2_dev *dev)."""
    if dev is None or dev.read is None or dev.write is None or dev.delay_us is None:
        return BMI2_E_NULL_PTR
    return BMI2_OK


# ---------------------------------------------------------------------------
# Lecture / ecriture de registres
# ---------------------------------------------------------------------------

def bmi2_get_regs(reg_addr, length, dev):
    """Lit `length` octets a partir du registre `reg_addr`.

    Retourne (rslt, data) ou data est un bytes/bytearray de longueur `length`.
    """
    rslt = _null_ptr_check(dev)
    if rslt != BMI2_OK:
        return rslt, None

    if dev.intf == BMI2_SPI_INTF:
        reg_addr = reg_addr | BMI2_SPI_RD_MASK

    total = length + dev.dummy_byte
    try:
        raw = dev.read(reg_addr, total)
    except Exception:
        dev.intf_rslt = BMI2_E_COM_FAIL
        return BMI2_E_COM_FAIL, None

    dev.intf_rslt = BMI2_INTF_RET_SUCCESS

    # Delais inter-transaction comme en C
    if dev.aps_status == BMI2_ENABLE:
        dev.delay_us(450)
    else:
        dev.delay_us(2)

    # Retire l'octet muet (SPI) si present
    if dev.dummy_byte and raw is not None:
        data = bytes(raw[dev.dummy_byte:dev.dummy_byte + length])
    else:
        data = bytes(raw) if raw is not None else b""

    if len(data) < length:
        return BMI2_E_COM_FAIL, data

    return BMI2_OK, data


def bmi2_set_regs(reg_addr, data, length, dev):
    """Ecrit `length` octets depuis `data` vers `reg_addr`."""
    rslt = _null_ptr_check(dev)
    if rslt != BMI2_OK:
        return rslt
    if data is None:
        return BMI2_E_NULL_PTR

    if dev.intf == BMI2_SPI_INTF:
        reg_addr = reg_addr & BMI2_SPI_WR_MASK

    try:
        if dev.aps_status == BMI2_ENABLE:
            # Ecriture byte-par-byte avec delai long en mode power save
            for i in range(length):
                dev.write((reg_addr + i) & 0xFF, bytes([data[i]]))
                dev.delay_us(BMI2_POWER_SAVE_MODE_DELAY_IN_US)
        else:
            # Ecriture burst en mode normal
            dev.write(reg_addr, bytes(data[:length]))
            dev.delay_us(BMI2_NORMAL_MODE_DELAY_IN_US)
    except Exception:
        dev.intf_rslt = BMI2_E_COM_FAIL
        return BMI2_E_COM_FAIL

    dev.intf_rslt = BMI2_INTF_RET_SUCCESS

    # Met a jour le flag APS si on ecrit PWR_CONF
    if reg_addr == BMI2_PWR_CONF_ADDR:
        if data[0] & BMI2_ADV_POW_EN_MASK:
            dev.aps_status = BMI2_ENABLE
        else:
            dev.aps_status = BMI2_DISABLE

    return BMI2_OK


# ---------------------------------------------------------------------------
# Command register / soft reset / power save
# ---------------------------------------------------------------------------

def bmi2_set_command_register(command, dev):
    """Envoie une commande (ex: BMI2_SOFT_RESET_CMD)."""
    return bmi2_set_regs(BMI2_CMD_REG_ADDR, bytes([command]), 1, dev)


def bmi2_set_adv_power_save(enable, dev):
    rslt = _null_ptr_check(dev)
    if rslt != BMI2_OK:
        return rslt
    rslt, data = bmi2_get_regs(BMI2_PWR_CONF_ADDR, 1, dev)
    if rslt != BMI2_OK:
        return rslt
    reg = bmi2_set_bit_pos0(data[0], BMI2_ADV_POW_EN_MASK, enable)
    rslt = bmi2_set_regs(BMI2_PWR_CONF_ADDR, bytes([reg]), 1, dev)
    if rslt != BMI2_OK:
        return BMI2_E_SET_APS_FAIL
    dev.aps_status = bmi2_get_bit_pos0(reg, BMI2_ADV_POW_EN_MASK)
    return BMI2_OK


def bmi2_get_adv_power_save(dev):
    """Retourne (rslt, aps_status)."""
    rslt = _null_ptr_check(dev)
    if rslt != BMI2_OK:
        return rslt, 0
    rslt, data = bmi2_get_regs(BMI2_PWR_CONF_ADDR, 1, dev)
    if rslt != BMI2_OK:
        return rslt, 0
    aps = bmi2_get_bit_pos0(data[0], BMI2_ADV_POW_EN_MASK)
    dev.aps_status = aps
    return BMI2_OK, aps


def bmi2_soft_reset(dev):
    rslt = _null_ptr_check(dev)
    if rslt != BMI2_OK:
        return rslt
    rslt = bmi2_set_regs(BMI2_CMD_REG_ADDR, bytes([BMI2_SOFT_RESET_CMD]), 1, dev)
    dev.delay_us(2000)

    # Apres reset, le capteur est en mode advance power save
    dev.aps_status = BMI2_ENABLE

    if rslt == BMI2_OK and dev.intf == BMI2_SPI_INTF:
        # Dummy read pour re-passer en SPI
        rslt, _ = bmi2_get_regs(BMI2_CHIP_ID_ADDR, 1, dev)

    if rslt == BMI2_OK:
        rslt = bmi2_write_config_file(dev)

    if rslt == BMI2_OK:
        dev.sens_en_stat = 0

    return rslt


# ---------------------------------------------------------------------------
# Chargement du blob firmware
# ---------------------------------------------------------------------------

def _set_config_load(enable, dev):
    rslt, data = bmi2_get_regs(BMI2_INIT_CTRL_ADDR, 1, dev)
    if rslt != BMI2_OK:
        return rslt
    reg = bmi2_set_bit_pos0(data[0], BMI2_CONF_LOAD_EN_MASK, enable)
    return bmi2_set_regs(BMI2_INIT_CTRL_ADDR, bytes([reg]), 1, dev)


def _upload_file(chunk, index, dev):
    """Upload un fragment de firmware déjà lu en mémoire.

    chunk : bytes ou bytearray contenant les octets à envoyer.
    index : position en octets dans le firmware (calcul de l'adresse registre).
    """
    if chunk is None:
        return BMI2_E_NULL_PTR
    addr_array = bytes([(index // 2) & 0x0F, ((index // 2) >> 4) & 0xFF])
    rslt = bmi2_set_regs(BMI2_INIT_ADDR_0, addr_array, 2, dev)
    if rslt != BMI2_OK:
        return rslt
    return bmi2_set_regs(BMI2_INIT_DATA_ADDR, chunk, len(chunk), dev)


def bmi2_get_internal_status(dev):
    """Retourne (rslt, int_stat)."""
    rslt = _null_ptr_check(dev)
    if rslt != BMI2_OK:
        return rslt, 0
    # Attend que l'ASIC soit initialise (~20 ms)
    dev.delay_us(BMI2_INTERNAL_STATUS_READ_DELAY_MS)
    rslt, data = bmi2_get_regs(BMI2_INTERNAL_STATUS_ADDR, 1, dev)
    if rslt != BMI2_OK:
        return rslt, 0
    return BMI2_OK, data[0]


def bmi2_write_config_file(dev):
    """Charge le firmware BMI2 dans le capteur.

    dev.config_file_ptr peut être :
      - un chemin de fichier (str) : lecture par chunks sans charger en RAM,
      - un blob bytes/bytearray/list : comportement original.
    """
    rslt = _null_ptr_check(dev)
    if rslt != BMI2_OK:
        return rslt
    if dev.config_size == 0 or dev.config_file_ptr is None:
        return BMI2_E_NULL_PTR

    # Les écritures doivent être de taille paire (alignement 2 octets)
    if dev.read_write_len % 2 != 0:
        dev.read_write_len -= 1
    if dev.read_write_len < 2:
        dev.read_write_len = 2

    config_size = dev.config_size
    cfg = dev.config_file_ptr
    rw_len = dev.read_write_len

    # Disable advance power save (nécessaire pour l'upload)
    rslt = bmi2_set_adv_power_save(BMI2_DISABLE, dev)
    if rslt != BMI2_OK:
        return rslt
    # Disable config load
    rslt = _set_config_load(BMI2_DISABLE, dev)
    if rslt != BMI2_OK:
        return rslt

    if isinstance(cfg, str):
        # ----------------------------------------------------------------
        # Chargement depuis un fichier binaire : un seul chunk en RAM
        # ----------------------------------------------------------------
        f = open(cfg, 'rb')
        try:
            buf = bytearray(rw_len)
            index = 0
            while index < config_size and rslt == BMI2_OK:
                remain = config_size - index
                rlen = rw_len if remain >= rw_len else remain
                if rlen % 2 != 0:
                    rlen -= 1
                if rlen == 0:
                    break
                n = f.readinto(memoryview(buf)[:rlen])
                if not n:
                    break
                rslt = _upload_file(memoryview(buf)[:n], index, dev)
                index += n
        finally:
            f.close()
    else:
        # ----------------------------------------------------------------
        # Blob en mémoire (bytes / bytearray / list) — comportement original
        # ----------------------------------------------------------------
        remain = config_size % rw_len
        if remain == 0:
            index = 0
            while index < config_size and rslt == BMI2_OK:
                rslt = _upload_file(cfg[index:index + rw_len], index, dev)
                index += rw_len
        else:
            bal_byte = config_size - remain
            index = 0
            while index < bal_byte and rslt == BMI2_OK:
                rslt = _upload_file(cfg[index:index + rw_len], index, dev)
                index += rw_len
            if rslt == BMI2_OK:
                saved_len = rw_len
                dev.read_write_len = 2
                index = bal_byte
                while index < config_size and rslt == BMI2_OK:
                    rslt = _upload_file(cfg[index:index + 2], index, dev)
                    index += 2
                dev.read_write_len = saved_len

    if rslt == BMI2_OK:
        rslt = _set_config_load(BMI2_ENABLE, dev)
    if rslt == BMI2_OK:
        rslt = bmi2_set_adv_power_save(BMI2_ENABLE, dev)
    if rslt != BMI2_OK:
        return rslt

    # Vérification du statut de chargement
    rslt, load_status = bmi2_get_internal_status(dev)
    if rslt == BMI2_OK:
        load_status &= BMI2_CONFIG_LOAD_STATUS_MASK
        dev.load_status = load_status
        if load_status != BMI2_CONFIG_LOAD_SUCCESS:
            rslt = BMI2_E_CONFIG_LOAD

    return rslt


# ---------------------------------------------------------------------------
# Init secondaire (verification chip id + soft reset)
# ---------------------------------------------------------------------------

def bmi2_sec_init(dev):
    rslt = _null_ptr_check(dev)
    if rslt != BMI2_OK:
        return rslt

    # Apres reset le capteur est en advance power save
    dev.aps_status = BMI2_ENABLE

    # Dummy read pour passer en SPI le cas echeant
    if dev.intf == BMI2_SPI_INTF:
        bmi2_get_regs(BMI2_CHIP_ID_ADDR, 1, dev)

    rslt, data = bmi2_get_regs(BMI2_CHIP_ID_ADDR, 1, dev)
    if rslt != BMI2_OK:
        return rslt

    chip_id = data[0]
    if chip_id != dev.chip_id:
        dev.chip_id = chip_id
        return BMI2_E_DEV_NOT_FOUND

    # Valeurs par defaut
    dev.resolution = 16
    dev.aux_man_en = 1
    dev.remap.x_axis = BMI2_MAP_X_AXIS
    dev.remap.x_axis_sign = BMI2_POS_SIGN
    dev.remap.y_axis = BMI2_MAP_Y_AXIS
    dev.remap.y_axis_sign = BMI2_POS_SIGN
    dev.remap.z_axis = BMI2_MAP_Z_AXIS
    dev.remap.z_axis_sign = BMI2_POS_SIGN

    return bmi2_soft_reset(dev)


# ---------------------------------------------------------------------------
# Enable / disable capteurs (PWR_CTRL)
# ---------------------------------------------------------------------------

def _select_sensor(sens_list):
    """Construit un bitfield sensor_sel a partir d'une liste d'ids (capteurs principaux)."""
    sensor_sel = 0
    for s in sens_list:
        if s == BMI2_ACCEL:
            sensor_sel |= BMI2_ACCEL_SENS_SEL
        elif s == BMI2_GYRO:
            sensor_sel |= BMI2_GYRO_SENS_SEL
        elif s == BMI2_AUX:
            sensor_sel |= BMI2_AUX_SENS_SEL
        elif s == BMI2_TEMP:
            sensor_sel |= BMI2_TEMP_SENS_SEL
        else:
            return BMI2_E_INVALID_SENSOR, 0
    return BMI2_OK, sensor_sel


def _sensor_enable(sensor_sel, dev):
    rslt, data = bmi2_get_regs(BMI2_PWR_CTRL_ADDR, 1, dev)
    if rslt != BMI2_OK:
        return rslt
    reg = data[0]
    if sensor_sel & BMI2_ACCEL_SENS_SEL:
        reg = bmi2_set_bits(reg, BMI2_ACC_EN_MASK, BMI2_ACC_EN_POS, BMI2_ENABLE)
    if sensor_sel & BMI2_GYRO_SENS_SEL:
        reg = bmi2_set_bits(reg, BMI2_GYR_EN_MASK, BMI2_GYR_EN_POS, BMI2_ENABLE)
    if sensor_sel & BMI2_AUX_SENS_SEL:
        reg = bmi2_set_bit_pos0(reg, BMI2_AUX_EN_MASK, BMI2_ENABLE)
    if sensor_sel & BMI2_TEMP_SENS_SEL:
        reg = bmi2_set_bits(reg, BMI2_TEMP_EN_MASK, BMI2_TEMP_EN_POS, BMI2_ENABLE)
    if sensor_sel & BMI2_MAIN_SENSORS:
        return bmi2_set_regs(BMI2_PWR_CTRL_ADDR, bytes([reg]), 1, dev)
    return BMI2_OK


def _sensor_disable(sensor_sel, dev):
    rslt, data = bmi2_get_regs(BMI2_PWR_CTRL_ADDR, 1, dev)
    if rslt != BMI2_OK:
        return rslt
    reg = data[0]
    if sensor_sel & BMI2_ACCEL_SENS_SEL:
        reg = bmi2_set_bit_val0(reg, BMI2_ACC_EN_MASK)
    if sensor_sel & BMI2_GYRO_SENS_SEL:
        reg = bmi2_set_bit_val0(reg, BMI2_GYR_EN_MASK)
    if sensor_sel & BMI2_AUX_SENS_SEL:
        reg = bmi2_set_bit_val0(reg, BMI2_AUX_EN_MASK)
    if sensor_sel & BMI2_TEMP_SENS_SEL:
        reg = bmi2_set_bit_val0(reg, BMI2_TEMP_EN_MASK)
    if sensor_sel & BMI2_MAIN_SENSORS:
        return bmi2_set_regs(BMI2_PWR_CTRL_ADDR, bytes([reg]), 1, dev)
    return BMI2_OK


# ---------------------------------------------------------------------------
# Set / get config ACCEL et GYRO
# ---------------------------------------------------------------------------

def _set_accel_config(cfg, dev):
    """cfg : Bmi2AccelConfig (odr, bwp, filter_perf, range)."""
    data = bytearray(2)
    # Byte 0 : ACC_CONF (odr + bwp + filter_perf)
    reg0 = 0
    reg0 = bmi2_set_bits(reg0, BMI2_ACC_FILTER_PERF_MODE_MASK,
                         BMI2_ACC_FILTER_PERF_MODE_POS, cfg.filter_perf)
    reg0 = bmi2_set_bits(reg0, BMI2_ACC_BW_PARAM_MASK,
                         BMI2_ACC_BW_PARAM_POS, cfg.bwp)
    reg0 = bmi2_set_bit_pos0(reg0, BMI2_ACC_ODR_MASK, cfg.odr)
    data[0] = reg0
    # Byte 1 : ACC_RANGE (range)
    data[1] = bmi2_set_bit_pos0(0, BMI2_ACC_RANGE_MASK, cfg.range)
    return bmi2_set_regs(BMI2_ACC_CONF_ADDR, data, 2, dev)


def _get_accel_config(cfg, dev):
    rslt, data = bmi2_get_regs(BMI2_ACC_CONF_ADDR, 2, dev)
    if rslt != BMI2_OK:
        return rslt
    cfg.filter_perf = bmi2_get_bits(data[0], BMI2_ACC_FILTER_PERF_MODE_MASK,
                                    BMI2_ACC_FILTER_PERF_MODE_POS)
    cfg.bwp = bmi2_get_bits(data[0], BMI2_ACC_BW_PARAM_MASK, BMI2_ACC_BW_PARAM_POS)
    cfg.odr = bmi2_get_bit_pos0(data[0], BMI2_ACC_ODR_MASK)
    cfg.range = bmi2_get_bit_pos0(data[1], BMI2_ACC_RANGE_MASK)
    return BMI2_OK


def _set_gyro_config(cfg, dev):
    """cfg : Bmi2GyroConfig (odr, bwp, noise_perf, filter_perf, range, ois_range)."""
    data = bytearray(2)
    reg0 = 0
    reg0 = bmi2_set_bits(reg0, BMI2_GYR_FILTER_PERF_MODE_MASK,
                         BMI2_GYR_FILTER_PERF_MODE_POS, cfg.filter_perf)
    reg0 = bmi2_set_bits(reg0, BMI2_GYR_NOISE_PERF_MODE_MASK,
                         BMI2_GYR_NOISE_PERF_MODE_POS, cfg.noise_perf)
    reg0 = bmi2_set_bits(reg0, BMI2_GYR_BW_PARAM_MASK,
                         BMI2_GYR_BW_PARAM_POS, cfg.bwp)
    reg0 = bmi2_set_bit_pos0(reg0, BMI2_GYR_ODR_MASK, cfg.odr)
    data[0] = reg0

    reg1 = 0
    reg1 = bmi2_set_bits(reg1, BMI2_GYR_OIS_RANGE_MASK,
                         BMI2_GYR_OIS_RANGE_POS, cfg.ois_range)
    reg1 = bmi2_set_bit_pos0(reg1, BMI2_GYR_RANGE_MASK, cfg.range)
    data[1] = reg1

    return bmi2_set_regs(BMI2_GYR_CONF_ADDR, data, 2, dev)


def _get_gyro_config(cfg, dev):
    rslt, data = bmi2_get_regs(BMI2_GYR_CONF_ADDR, 2, dev)
    if rslt != BMI2_OK:
        return rslt
    cfg.filter_perf = bmi2_get_bits(data[0], BMI2_GYR_FILTER_PERF_MODE_MASK,
                                    BMI2_GYR_FILTER_PERF_MODE_POS)
    cfg.noise_perf = bmi2_get_bits(data[0], BMI2_GYR_NOISE_PERF_MODE_MASK,
                                   BMI2_GYR_NOISE_PERF_MODE_POS)
    cfg.bwp = bmi2_get_bits(data[0], BMI2_GYR_BW_PARAM_MASK, BMI2_GYR_BW_PARAM_POS)
    cfg.odr = bmi2_get_bit_pos0(data[0], BMI2_GYR_ODR_MASK)
    cfg.ois_range = bmi2_get_bits(data[1], BMI2_GYR_OIS_RANGE_MASK, BMI2_GYR_OIS_RANGE_POS)
    cfg.range = bmi2_get_bit_pos0(data[1], BMI2_GYR_RANGE_MASK)
    return BMI2_OK


def bmi2_set_sensor_config(sens_cfg_list, dev):
    """Set configuration pour une liste de Bmi2SensConfig (seulement ACCEL/GYRO)."""
    rslt = _null_ptr_check(dev)
    if rslt != BMI2_OK:
        return rslt
    aps_stat = dev.aps_status
    for cfg in sens_cfg_list:
        if aps_stat == BMI2_ENABLE:
            rslt = bmi2_set_adv_power_save(BMI2_DISABLE, dev)
            if rslt != BMI2_OK:
                return rslt
        if cfg.type == BMI2_ACCEL:
            rslt = _set_accel_config(cfg.cfg.acc, dev)
        elif cfg.type == BMI2_GYRO:
            rslt = _set_gyro_config(cfg.cfg.gyr, dev)
        else:
            rslt = BMI2_E_INVALID_SENSOR
        if rslt != BMI2_OK:
            break
    if aps_stat == BMI2_ENABLE and rslt == BMI2_OK:
        rslt = bmi2_set_adv_power_save(BMI2_ENABLE, dev)
    return rslt


def bmi2_get_sensor_config(sens_cfg_list, dev):
    """Lit la configuration de chaque entree de la liste."""
    rslt = _null_ptr_check(dev)
    if rslt != BMI2_OK:
        return rslt
    for cfg in sens_cfg_list:
        if cfg.type == BMI2_ACCEL:
            rslt = _get_accel_config(cfg.cfg.acc, dev)
        elif cfg.type == BMI2_GYRO:
            rslt = _get_gyro_config(cfg.cfg.gyr, dev)
        else:
            rslt = BMI2_E_INVALID_SENSOR
        if rslt != BMI2_OK:
            break
    return rslt


# ---------------------------------------------------------------------------
# Lecture des donnees capteur (accel + gyro + aux + sensortime)
# ---------------------------------------------------------------------------

def _to_signed_16(val):
    """Convertit un uint16 en int16 signe."""
    if val >= 0x8000:
        val -= 0x10000
    return val


def _get_acc_gyr_data(data_axes, reg_data, offset):
    """Remplit data_axes (Bmi2SensAxesData) a partir de 6 octets LSB/MSB x3."""
    x_lsb = reg_data[offset + 0]
    x_msb = reg_data[offset + 1]
    y_lsb = reg_data[offset + 2]
    y_msb = reg_data[offset + 3]
    z_lsb = reg_data[offset + 4]
    z_msb = reg_data[offset + 5]
    data_axes.x = _to_signed_16((x_msb << 8) | x_lsb)
    data_axes.y = _to_signed_16((y_msb << 8) | y_lsb)
    data_axes.z = _to_signed_16((z_msb << 8) | z_lsb)


def _get_remapped_data(data_axes, dev):
    raw = [data_axes.x, data_axes.y, data_axes.z]
    rm = dev.remap

    if rm.x_axis_sign == BMI2_POS_SIGN:
        data_axes.x = raw[rm.x_axis]
    else:
        data_axes.x = -raw[rm.x_axis]
    if rm.y_axis_sign == BMI2_POS_SIGN:
        data_axes.y = raw[rm.y_axis]
    else:
        data_axes.y = -raw[rm.y_axis]
    if rm.z_axis_sign == BMI2_POS_SIGN:
        data_axes.z = raw[rm.z_axis]
    else:
        data_axes.z = -raw[rm.z_axis]


def bmi2_get_sensor_data(data, dev):
    """Remplit `data` (Bmi2SensData) : acc, gyr, aux, sens_time.

    Lit d'un seul burst BMI2_STATUS_ADDR .. +24 octets.
    """
    rslt, raw = bmi2_get_regs(BMI2_STATUS_ADDR, BMI2_ACC_GYR_AUX_SENSORTIME_NUM_BYTES, dev)
    if rslt != BMI2_OK:
        return rslt
    # Copie des donnees auxiliaires
    for i in range(BMI2_AUX_NUM_BYTES):
        data.aux_data[i] = raw[BMI2_AUX_START_INDEX + i]
    # Accelerometre
    _get_acc_gyr_data(data.acc, raw, BMI2_ACC_START_INDEX)
    _get_remapped_data(data.acc, dev)
    # Gyroscope
    _get_acc_gyr_data(data.gyr, raw, BMI2_GYR_START_INDEX)
    _get_remapped_data(data.gyr, dev)
    # Sensortime (3 octets : byte1..byte3)
    data.sens_time = (raw[21] | (raw[22] << 8) | (raw[23] << 16))
    return BMI2_OK


# ---------------------------------------------------------------------------
# Acces aux pages de configuration des features
# ---------------------------------------------------------------------------

def bmi2_get_feat_config(sw_page, dev):
    """Lit les BMI2_FEAT_SIZE_IN_BYTES octets de la page feature sw_page.

    Retourne (rslt, bytearray[16]).
    Equivalent de bmi2_get_feat_config() dans bmi2.c.
    """
    rslt = _null_ptr_check(dev)
    if rslt != BMI2_OK:
        return rslt, None
    if sw_page >= dev.page_max:
        return BMI2_E_INVALID_PAGE, None

    # Commutation de page
    rslt = bmi2_set_regs(BMI2_FEAT_PAGE_ADDR, bytes([sw_page]), 1, dev)
    if rslt != BMI2_OK:
        return rslt, None

    feat_config = bytearray(BMI2_FEAT_SIZE_IN_BYTES)

    if dev.read_write_len < BMI2_FEAT_SIZE_IN_BYTES:
        rw_len = dev.read_write_len
        if rw_len % 2 != 0:
            rw_len -= 1
        addr = BMI2_FEATURES_REG_ADDR
        idx = 0
        remain = BMI2_FEAT_SIZE_IN_BYTES
        while remain > 0:
            chunk_len = rw_len if remain >= rw_len else remain
            rslt, chunk = bmi2_get_regs(addr, chunk_len, dev)
            if rslt != BMI2_OK:
                return rslt, None
            feat_config[idx:idx + chunk_len] = chunk
            idx += chunk_len
            addr += chunk_len
            remain -= chunk_len
    else:
        rslt, data = bmi2_get_regs(BMI2_FEATURES_REG_ADDR, BMI2_FEAT_SIZE_IN_BYTES, dev)
        if rslt != BMI2_OK:
            return rslt, None
        feat_config[:] = data

    return BMI2_OK, feat_config


def bmi2_extract_input_feat_config(feat_cfg_obj, type_val, dev):
    """Cherche dans dev.feat_config la feature de type type_val.

    Si trouvee, copie type/page/start_addr dans feat_cfg_obj et retourne True.
    Equivalent de bmi2_extract_input_feat_config() dans bmi2.c.

    dev.feat_config doit etre une liste d'objets avec attributs type/page/start_addr.
    """
    if dev.feat_config is None:
        return False
    for fc in dev.feat_config:
        if fc.type == type_val:
            feat_cfg_obj.type = fc.type
            feat_cfg_obj.page = fc.page
            feat_cfg_obj.start_addr = fc.start_addr
            return True
    return False


# ---------------------------------------------------------------------------
# Statut et configuration des interruptions
# ---------------------------------------------------------------------------

def bmi2_get_int_status(dev):
    """Lit les registres INT_STATUS_0 et INT_STATUS_1.

    Retourne (rslt, status_uint16).
    Le byte bas correspond aux feature interrupts, le byte haut aux data interrupts.
    """
    rslt = _null_ptr_check(dev)
    if rslt != BMI2_OK:
        return rslt, 0
    rslt, data = bmi2_get_regs(BMI2_INT_STATUS_0_ADDR, 2, dev)
    if rslt != BMI2_OK:
        return rslt, 0
    status = data[0] | (data[1] << 8)
    return BMI2_OK, status


def bmi2_set_int_pin_config(int_cfg, dev):
    """Configure les broches d'interruption INT1 et/ou INT2.

    int_cfg : instance de Bmi2IntPinConfig (pin_type, int_latch, pin_cfg[]).
    Equivalent de bmi2_set_int_pin_config() dans bmi2.c.
    """
    rslt = _null_ptr_check(dev)
    if rslt != BMI2_OK:
        return rslt
    if int_cfg is None:
        return BMI2_E_NULL_PTR

    int_pin = int_cfg.pin_type
    if int_pin <= BMI2_INT_NONE or int_pin >= BMI2_INT_PIN_MAX:
        return BMI2_E_INVALID_INT_PIN

    # Lit les 3 registres : INT1_IO_CTRL, INT2_IO_CTRL, INT_LATCH
    rslt, raw = bmi2_get_regs(BMI2_INT1_IO_CTRL_ADDR, 3, dev)
    if rslt != BMI2_OK:
        return rslt

    data_array = bytearray(raw)

    if int_pin == BMI2_INT1 or int_pin == BMI2_INT_BOTH:
        reg = data_array[0]
        reg = bmi2_set_bits(reg, BMI2_INT_LEVEL_MASK, BMI2_INT_LEVEL_POS, int_cfg.pin_cfg[0].lvl)
        reg = bmi2_set_bits(reg, BMI2_INT_OPEN_DRAIN_MASK, BMI2_INT_OPEN_DRAIN_POS, int_cfg.pin_cfg[0].od)
        reg = bmi2_set_bits(reg, BMI2_INT_OUTPUT_EN_MASK, BMI2_INT_OUTPUT_EN_POS, int_cfg.pin_cfg[0].output_en)
        reg = bmi2_set_bits(reg, BMI2_INT_INPUT_EN_MASK, BMI2_INT_INPUT_EN_POS, int_cfg.pin_cfg[0].input_en)
        data_array[0] = reg

    if int_pin == BMI2_INT2 or int_pin == BMI2_INT_BOTH:
        reg = data_array[1]
        reg = bmi2_set_bits(reg, BMI2_INT_LEVEL_MASK, BMI2_INT_LEVEL_POS, int_cfg.pin_cfg[1].lvl)
        reg = bmi2_set_bits(reg, BMI2_INT_OPEN_DRAIN_MASK, BMI2_INT_OPEN_DRAIN_POS, int_cfg.pin_cfg[1].od)
        reg = bmi2_set_bits(reg, BMI2_INT_OUTPUT_EN_MASK, BMI2_INT_OUTPUT_EN_POS, int_cfg.pin_cfg[1].output_en)
        reg = bmi2_set_bits(reg, BMI2_INT_INPUT_EN_MASK, BMI2_INT_INPUT_EN_POS, int_cfg.pin_cfg[1].input_en)
        data_array[1] = reg

    # Mode latch / non-latch
    data_array[2] = bmi2_set_bit_pos0(data_array[2], BMI2_INT_LATCH_MASK, int_cfg.int_latch)

    return bmi2_set_regs(BMI2_INT1_IO_CTRL_ADDR, data_array, 3, dev)


def bmi2_map_feat_int(type_val, hw_int_pin, dev):
    """Mappe une interruption feature sur une broche INT1/INT2.

    type_val   : type de feature (BMI2_ANY_MOTION, BMI2_NO_MOTION, etc.)
    hw_int_pin : BMI2_INT1, BMI2_INT2, BMI2_INT_BOTH ou BMI2_INT_NONE
    Equivalent de bmi2_map_feat_int() dans bmi2.c.
    """
    rslt = _null_ptr_check(dev)
    if rslt != BMI2_OK:
        return rslt

    rslt, raw = bmi2_get_regs(BMI2_INT1_MAP_FEAT_ADDR, 2, dev)
    if rslt != BMI2_OK:
        return rslt

    data_array = bytearray(raw)

    # Cherche le masque dans la table map_int du device
    feat_int = 0
    if dev.map_int is not None:
        for mi in dev.map_int:
            if mi.type == type_val:
                feat_int = mi.sens_map_int
                break

    if hw_int_pin >= BMI2_INT_PIN_MAX:
        return BMI2_E_INVALID_INT_PIN

    if hw_int_pin == BMI2_INT_NONE:
        data_array[0] &= ~feat_int
        data_array[1] &= ~feat_int
    elif hw_int_pin == BMI2_INT1:
        data_array[0] |= feat_int
        data_array[1] &= ~feat_int
    elif hw_int_pin == BMI2_INT2:
        data_array[1] |= feat_int
        data_array[0] &= ~feat_int
    elif hw_int_pin == BMI2_INT_BOTH:
        data_array[0] |= feat_int
        data_array[1] |= feat_int

    rslt = bmi2_set_regs(BMI2_INT1_MAP_FEAT_ADDR, bytes([data_array[0]]), 1, dev)
    if rslt == BMI2_OK:
        rslt = bmi2_set_regs(BMI2_INT2_MAP_FEAT_ADDR, bytes([data_array[1]]), 1, dev)
    return rslt


def bmi2_map_data_int(data_int, int_pin, dev):
    """Mappe une interruption donnee (DRDY, FFULL, etc.) sur une broche.

    Equivalent de bmi2_map_data_int() dans bmi2.c.
    """
    rslt, raw = bmi2_get_regs(BMI2_INT_MAP_DATA_ADDR, 1, dev)
    if rslt != BMI2_OK:
        return rslt

    reg_data = raw[0]
    int1_mask = data_int & 0xFF
    int2_mask = (data_int << 4) & 0xFF

    if int_pin >= BMI2_INT_PIN_MAX:
        return BMI2_E_INVALID_INT_PIN

    if int_pin == BMI2_INT_NONE:
        reg_data &= ~(int1_mask | int2_mask)
    elif int_pin == BMI2_INT1:
        reg_data |= int1_mask
    elif int_pin == BMI2_INT2:
        reg_data |= int2_mask
    elif int_pin == BMI2_INT_BOTH:
        reg_data |= (int1_mask | int2_mask)

    return bmi2_set_regs(BMI2_INT_MAP_DATA_ADDR, bytes([reg_data]), 1, dev)


# ---------------------------------------------------------------------------
# Statut du capteur
# ---------------------------------------------------------------------------

def bmi2_get_status(dev):
    """Lit le registre STATUS (data-ready, commande ready, AUX busy).

    Retourne (rslt, status_byte).
    Bits utiles :
        BMI2_DRDY_ACC  (0x80) — accéléromètre prêt
        BMI2_DRDY_GYR  (0x40) — gyroscope prêt
        BMI2_DRDY_AUX  (0x20) — auxiliaire prêt
        BMI2_CMD_RDY   (0x10) — commande traitée
    """
    rslt = _null_ptr_check(dev)
    if rslt != BMI2_OK:
        return rslt, 0
    rslt, raw = bmi2_get_regs(BMI2_STATUS_ADDR, 1, dev)
    if rslt != BMI2_OK:
        return rslt, 0
    return BMI2_OK, raw[0]


# ---------------------------------------------------------------------------
# Remapping des axes
# ---------------------------------------------------------------------------

def _set_remap_axes(remap, dev):
    """Écrit le remapping d'axes dans la page feature (set_remap_axes interne)."""
    aps_stat = dev.aps_status
    rslt = BMI2_OK
    if aps_stat == BMI2_ENABLE:
        rslt = bmi2_set_adv_power_save(BMI2_DISABLE, dev)
    if rslt != BMI2_OK:
        return rslt

    fc = Bmi2FeatureConfig()
    if not bmi2_extract_input_feat_config(fc, BMI2_AXIS_MAP, dev):
        if aps_stat == BMI2_ENABLE:
            bmi2_set_adv_power_save(BMI2_ENABLE, dev)
        return BMI2_E_INVALID_SENSOR

    rslt, feat_config = bmi2_get_feat_config(fc.page, dev)
    if rslt != BMI2_OK:
        if aps_stat == BMI2_ENABLE:
            bmi2_set_adv_power_save(BMI2_ENABLE, dev)
        return rslt

    idx = fc.start_addr

    # Byte 0 : x_axis | x_axis_sign | y_axis | y_axis_sign | z_axis
    x_axis      = remap.x_axis & BMI2_X_AXIS_MASK
    x_axis_sign = (remap.x_axis_sign << BMI2_X_AXIS_SIGN_POS) & BMI2_X_AXIS_SIGN_MASK
    y_axis      = (remap.y_axis << BMI2_Y_AXIS_POS) & BMI2_Y_AXIS_MASK
    y_axis_sign = (remap.y_axis_sign << BMI2_Y_AXIS_SIGN_POS) & BMI2_Y_AXIS_SIGN_MASK
    z_axis      = (remap.z_axis << BMI2_Z_AXIS_POS) & BMI2_Z_AXIS_MASK
    feat_config[idx] = x_axis | x_axis_sign | y_axis | y_axis_sign | z_axis

    # Byte 1 : z_axis_sign au bit 0 (ne pas écraser les autres bits)
    z_axis_sign = remap.z_axis_sign & BMI2_Z_AXIS_SIGN_MASK
    feat_config[idx + 1] = bmi2_set_bit_pos0(feat_config[idx + 1], BMI2_Z_AXIS_SIGN_MASK, z_axis_sign)

    # Écrit seulement les 2 octets concernés (reg + start_addr offset)
    reg_addr = BMI2_FEATURES_REG_ADDR + fc.start_addr
    rslt = bmi2_set_regs(reg_addr, feat_config[idx:idx + 2], 2, dev)

    if aps_stat == BMI2_ENABLE and rslt == BMI2_OK:
        rslt = bmi2_set_adv_power_save(BMI2_ENABLE, dev)
    return rslt


def bmi2_set_remap_axes(remapped_axis, dev):
    """Reconfigure le mapping X/Y/Z du capteur.

    remapped_axis : instance de Bmi2Remap avec attributs .x, .y, .z
                    contenant des constantes BMI2_AXIS_POS_*/BMI2_AXIS_NEG_*.

    Exemple (X→Y, Y→-Z, Z→X) :
        axes = Bmi2Remap()
        axes.x = BMI2_AXIS_POS_Y
        axes.y = BMI2_AXIS_NEG_Z
        axes.z = BMI2_AXIS_POS_X
        bmi2_set_remap_axes(axes, dev)

    Retourne int8_t (BMI2_OK = 0 en cas de succès).
    """
    rslt = _null_ptr_check(dev)
    if rslt != BMI2_OK:
        return rslt
    if remapped_axis is None:
        return BMI2_E_NULL_PTR

    # Vérifie que chaque axe physique (X/Y/Z) est représenté exactement une fois
    remap_axes = remapped_axis.x | remapped_axis.y | remapped_axis.z
    if (remap_axes & BMI2_AXIS_MASK) != BMI2_AXIS_MASK:
        return BMI2_E_REMAP_ERROR

    # Convertit valeurs utilisateur → représentation interne (Bmi2AxesRemap)
    # Valeurs user : BMI2_X=1, BMI2_Y=2, BMI2_Z=4 (+ BMI2_AXIS_SIGN=8 si négatif)
    # Valeurs internes : MAP_X_AXIS=0, MAP_Y_AXIS=1, MAP_Z_AXIS=2
    _map = {1: BMI2_MAP_X_AXIS, 2: BMI2_MAP_Y_AXIS, 4: BMI2_MAP_Z_AXIS}

    remap = Bmi2AxesRemap()
    remap.x_axis      = _map.get(remapped_axis.x & BMI2_AXIS_MASK, 0)
    remap.x_axis_sign = BMI2_MAP_NEGATIVE if (remapped_axis.x & BMI2_AXIS_SIGN) else BMI2_MAP_POSITIVE
    remap.y_axis      = _map.get(remapped_axis.y & BMI2_AXIS_MASK, 0)
    remap.y_axis_sign = BMI2_MAP_NEGATIVE if (remapped_axis.y & BMI2_AXIS_SIGN) else BMI2_MAP_POSITIVE
    remap.z_axis      = _map.get(remapped_axis.z & BMI2_AXIS_MASK, 0)
    remap.z_axis_sign = BMI2_MAP_NEGATIVE if (remapped_axis.z & BMI2_AXIS_SIGN) else BMI2_MAP_POSITIVE

    # Met à jour la structure dev (signe interne : BMI2_POS_SIGN/BMI2_NEG_SIGN)
    dev.remap.x_axis      = remap.x_axis
    dev.remap.x_axis_sign = BMI2_NEG_SIGN if remap.x_axis_sign == BMI2_MAP_NEGATIVE else BMI2_POS_SIGN
    dev.remap.y_axis      = remap.y_axis
    dev.remap.y_axis_sign = BMI2_NEG_SIGN if remap.y_axis_sign == BMI2_MAP_NEGATIVE else BMI2_POS_SIGN
    dev.remap.z_axis      = remap.z_axis
    dev.remap.z_axis_sign = BMI2_NEG_SIGN if remap.z_axis_sign == BMI2_MAP_NEGATIVE else BMI2_POS_SIGN

    return _set_remap_axes(remap, dev)
