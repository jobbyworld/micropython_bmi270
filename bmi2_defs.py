"""
bmi2_defs.py

MicroPython conversion of Bosch Sensortec's `bmi2_defs.h` (v2.86.1).

This module contains:
  - All numeric constants (`#define`) from the original header as
    `micropython.const()` module-level attributes.
  - Utility bit-manipulation helpers (formerly C macros).
  - Python classes corresponding to C structs / unions.
  - Enum namespaces (classes) for `bmi2_intf`, `bmi2_sensor_config_error`,
    `bmi2_hw_int_pin`, `bmi2_wear_arm_pos`, `bmi2_act_recog_type` and
    `bmi2_act_recog_stat`. Individual enum values are also exposed as
    module-level constants (the way they are referenced in the C code).

Function pointer typedefs (`bmi2_read_fptr_t`, `bmi2_write_fptr_t`,
`bmi2_delay_fptr_t`, `bmi2_wake_up_fptr_t`, `bmi2_tap_fptr_t`) are not
translated here: they are represented in `bmi2.py` as `None` attributes
on `Bmi2Dev` that the user assigns with a Python callable.

Target platform: Arduino Nano ESP32-C6 running MicroPython.
"""

from micropython import const


# =============================================================================
# Utility helpers (formerly macros in bmi2_defs.h)
# =============================================================================

def bmi2_set_bits(reg_data, mask, pos, data):
    """Equivalent of BMI2_SET_BITS(reg_data, bitname, data).

    In C, `bitname` is a token that is concatenated with `_MASK` / `_POS`.
    In Python, the mask and the position are passed explicitly.
    """
    return (reg_data & ~mask) | ((data << pos) & mask)


def bmi2_get_bits(reg_data, mask, pos):
    """Equivalent of BMI2_GET_BITS(reg_data, bitname)."""
    return (reg_data & mask) >> pos


def bmi2_set_bit_pos0(reg_data, mask, data):
    """Equivalent of BMI2_SET_BIT_POS0(reg_data, bitname, data)."""
    return (reg_data & ~mask) | (data & mask)


def bmi2_get_bit_pos0(reg_data, mask):
    """Equivalent of BMI2_GET_BIT_POS0(reg_data, bitname)."""
    return reg_data & mask


def bmi2_set_bit_val0(reg_data, mask):
    """Equivalent of BMI2_SET_BIT_VAL0(reg_data, bitname)."""
    return reg_data & ~mask


def bmi2_get_lsb(var):
    """Low byte of a 16-bit value."""
    return var & 0x00FF


def bmi2_get_msb(var):
    """High byte of a 16-bit value."""
    return (var & 0xFF00) >> 8


def bmi2_abs(a):
    """Absolute value helper (BMI2_ABS)."""
    return a if a > 0 else -a


# =============================================================================
# General Macro Definitions
# =============================================================================

BMI2_MAX_BUFFER_SIZE        = const(128)

# LSB and MSB mask definitions
BMI2_SET_LOW_BYTE           = const(0x00FF)
BMI2_SET_HIGH_BYTE          = const(0xFF00)
BMI2_SET_LOW_NIBBLE         = const(0x0F)

# For enable and disable
BMI2_ENABLE                 = const(1)
BMI2_DISABLE                = const(0)

# TRUE/FALSE
BMI2_TRUE                   = const(1)
BMI2_FALSE                  = const(0)

# Maximum length of read
BMI2_MAX_LEN                = const(128)

# Sensor interface success code
BMI2_INTF_RET_SUCCESS       = const(0)

# Success code
BMI2_OK                     = const(0)

# Delay values
BMI2_POWER_SAVE_MODE_DELAY_IN_US = const(450)
BMI2_NORMAL_MODE_DELAY_IN_US     = const(2)

# Error codes
BMI2_E_NULL_PTR                    = const(-1)
BMI2_E_COM_FAIL                    = const(-2)
BMI2_E_DEV_NOT_FOUND               = const(-3)
BMI2_E_OUT_OF_RANGE                = const(-4)
BMI2_E_ACC_INVALID_CFG             = const(-5)
BMI2_E_GYRO_INVALID_CFG            = const(-6)
BMI2_E_ACC_GYR_INVALID_CFG         = const(-7)
BMI2_E_INVALID_SENSOR              = const(-8)
BMI2_E_CONFIG_LOAD                 = const(-9)
BMI2_E_INVALID_PAGE                = const(-10)
BMI2_E_INVALID_FEAT_BIT            = const(-11)
BMI2_E_INVALID_INT_PIN             = const(-12)
BMI2_E_SET_APS_FAIL                = const(-13)
BMI2_E_AUX_INVALID_CFG             = const(-14)
BMI2_E_AUX_BUSY                    = const(-15)
BMI2_E_SELF_TEST_FAIL              = const(-16)
BMI2_E_REMAP_ERROR                 = const(-17)
BMI2_E_GYR_USER_GAIN_UPD_FAIL      = const(-18)
BMI2_E_SELF_TEST_NOT_DONE          = const(-19)
BMI2_E_INVALID_INPUT               = const(-20)
BMI2_E_INVALID_STATUS              = const(-21)
BMI2_E_CRT_ERROR                   = const(-22)
BMI2_E_ST_ALREADY_RUNNING          = const(-23)
BMI2_E_CRT_READY_FOR_DL_FAIL_ABORT = const(-24)
BMI2_E_DL_ERROR                    = const(-25)
BMI2_E_PRECON_ERROR                = const(-26)
BMI2_E_ABORT_ERROR                 = const(-27)
BMI2_E_GYRO_SELF_TEST_ERROR        = const(-28)
BMI2_E_GYRO_SELF_TEST_TIMEOUT      = const(-29)
BMI2_E_WRITE_CYCLE_ONGOING         = const(-30)
BMI2_E_WRITE_CYCLE_TIMEOUT         = const(-31)
BMI2_E_ST_NOT_RUNING                = const(-32)
BMI2_E_DATA_RDY_INT_FAILED         = const(-33)
BMI2_E_INVALID_FOC_POSITION        = const(-34)

# FIFO warning codes
BMI2_W_FIFO_EMPTY     = const(1)
BMI2_W_PARTIAL_READ   = const(2)
BMI2_W_DUMMY_BYTE     = const(3)

# Dummy frame header FIFO headerless mode
BMI2_FIFO_HEADERLESS_DUMMY_ACC   = const(0x01)
BMI2_FIFO_HEADERLESS_DUMMY_GYR   = const(0x02)
BMI2_FIFO_HEADERLESS_DUMMY_AUX   = const(0x03)
BMI2_FIFO_HEADERLESS_DUMMY_BYTE_1 = const(0x7F)
BMI2_FIFO_HEADERLESS_DUMMY_BYTE_2 = const(0x00)
BMI2_FIFO_HEADERLESS_DUMMY_BYTE_3 = const(0x80)

# Bit wise to define information
BMI2_I_MIN_VALUE = const(1)
BMI2_I_MAX_VALUE = const(2)

# =============================================================================
# BMI2 register addresses
# =============================================================================
BMI2_CHIP_ID_ADDR            = const(0x00)
BMI2_STATUS_ADDR             = const(0x03)
BMI2_AUX_X_LSB_ADDR          = const(0x04)
BMI2_ACC_X_LSB_ADDR          = const(0x0C)
BMI2_GYR_X_LSB_ADDR          = const(0x12)
BMI2_SENSORTIME_ADDR         = const(0x18)
BMI2_EVENT_ADDR              = const(0x1B)
BMI2_INT_STATUS_0_ADDR       = const(0x1C)
BMI2_INT_STATUS_1_ADDR       = const(0x1D)
BMI2_SC_OUT_0_ADDR           = const(0x1E)
BMI2_SYNC_COMMAND_ADDR       = const(0x1E)
BMI2_GYR_CAS_GPIO0_ADDR      = const(0x1E)
BMI2_INTERNAL_STATUS_ADDR    = const(0x21)
BMI2_TEMPERATURE_0_ADDR      = const(0x22)
BMI2_TEMPERATURE_1_ADDR      = const(0x23)
BMI2_FIFO_LENGTH_0_ADDR      = const(0x24)
BMI2_FIFO_DATA_ADDR          = const(0x26)
BMI2_FEAT_PAGE_ADDR          = const(0x2F)
BMI2_FEATURES_REG_ADDR       = const(0x30)
BMI2_ACC_CONF_ADDR           = const(0x40)
BMI2_GYR_CONF_ADDR           = const(0x42)
BMI2_AUX_CONF_ADDR           = const(0x44)
BMI2_FIFO_DOWNS_ADDR         = const(0x45)
BMI2_FIFO_WTM_0_ADDR         = const(0x46)
BMI2_FIFO_WTM_1_ADDR         = const(0x47)
BMI2_FIFO_CONFIG_0_ADDR      = const(0x48)
BMI2_FIFO_CONFIG_1_ADDR      = const(0x49)
BMI2_SATURATION_ADDR         = const(0x4A)
BMI2_AUX_DEV_ID_ADDR         = const(0x4B)
BMI2_AUX_IF_CONF_ADDR        = const(0x4C)
BMI2_AUX_RD_ADDR             = const(0x4D)
BMI2_AUX_WR_ADDR             = const(0x4E)
BMI2_AUX_WR_DATA_ADDR        = const(0x4F)
BMI2_ERR_REG_MSK_ADDR        = const(0x52)
BMI2_INT1_IO_CTRL_ADDR       = const(0x53)
BMI2_INT2_IO_CTRL_ADDR       = const(0x54)
BMI2_INT_LATCH_ADDR          = const(0x55)
BMI2_INT1_MAP_FEAT_ADDR      = const(0x56)
BMI2_INT2_MAP_FEAT_ADDR      = const(0x57)
BMI2_INT_MAP_DATA_ADDR       = const(0x58)
BMI2_INIT_CTRL_ADDR          = const(0x59)
BMI2_INIT_ADDR_0             = const(0x5B)
BMI2_INIT_ADDR_1             = const(0x5C)
BMI2_INIT_DATA_ADDR          = const(0x5E)
BMI2_INTERNAL_ERR_ADDR       = const(0x5F)
BMI2_AUX_IF_TRIM             = const(0x68)
BMI2_GYR_CRT_CONF_ADDR       = const(0x69)
BMI2_NVM_CONF_ADDR           = const(0x6A)
BMI2_IF_CONF_ADDR            = const(0x6B)
BMI2_DRV_STR_ADDR            = const(0x6C)
BMI2_ACC_SELF_TEST_ADDR      = const(0x6D)
BMI2_GYR_SELF_TEST_AXES_ADDR = const(0x6E)
BMI2_SELF_TEST_MEMS_ADDR     = const(0x6F)
BMI2_NV_CONF_ADDR            = const(0x70)
BMI2_ACC_OFF_COMP_0_ADDR     = const(0x71)
BMI2_GYR_OFF_COMP_3_ADDR     = const(0x74)
BMI2_GYR_OFF_COMP_6_ADDR     = const(0x77)
BMI2_GYR_USR_GAIN_0_ADDR     = const(0x78)
BMI2_PWR_CONF_ADDR           = const(0x7C)
BMI2_PWR_CTRL_ADDR           = const(0x7D)
BMI2_CMD_REG_ADDR            = const(0x7E)

# BMI2 I2C addresses
BMI2_I2C_PRIM_ADDR = const(0x68)
BMI2_I2C_SEC_ADDR  = const(0x69)

# BMI2 Commands
BMI2_G_TRIGGER_CMD   = const(0x02)
BMI2_USR_GAIN_CMD    = const(0x03)
BMI2_NVM_PROG_CMD    = const(0xA0)
BMI2_SOFT_RESET_CMD  = const(0xB6)
BMI2_FIFO_FLUSH_CMD  = const(0xB0)

# Sensor data bytes
BMI2_AUX_NUM_BYTES                    = const(8)
BMI2_ACC_NUM_BYTES                    = const(6)
BMI2_GYR_NUM_BYTES                    = const(6)
BMI2_STATUS_INDEX                     = const(0)
BMI2_AUX_START_INDEX                  = const(1)
BMI2_ACC_START_INDEX                  = const(9)
BMI2_GYR_START_INDEX                  = const(15)
BMI2_ACC_GYR_AUX_SENSORTIME_NUM_BYTES = const(24)
BMI2_CRT_CONFIG_FILE_SIZE             = const(2048)
BMI2_FEAT_SIZE_IN_BYTES               = const(16)
BMI2_ACC_CONFIG_LENGTH                = const(2)

# Configuration load status
BMI2_CONFIG_LOAD_SUCCESS     = const(1)
BMI2_CONFIG_LOAD_STATUS_MASK = const(0x0F)

# BMI2 pages
BMI2_PAGE_0 = const(0)
BMI2_PAGE_1 = const(1)
BMI2_PAGE_2 = const(2)
BMI2_PAGE_3 = const(3)
BMI2_PAGE_4 = const(4)
BMI2_PAGE_5 = const(5)
BMI2_PAGE_6 = const(6)
BMI2_PAGE_7 = const(7)

# Array parameter definitions
BMI2_PARSE_SENSOR_TIME_LSB_BYTE  = const(21)
BMI2_PARSE_SENSOR_TIME_XLSB_BYTE = const(22)
BMI2_PARSE_SENSOR_TIME_MSB_BYTE  = const(23)

BMI2_SENSOR_TIME_XLSB_BYTE = const(1)
BMI2_SENSOR_TIME_MSB_BYTE  = const(2)

# Gyro CRT
BMI2_GYR_RDY_FOR_DL_MASK  = const(0x08)
BMI2_GYR_CRT_RUNNING_MASK = const(0x04)

# Status register
BMI2_AUX_BUSY_MASK = const(0x04)
BMI2_CMD_RDY_MASK  = const(0x10)
BMI2_DRDY_AUX_MASK = const(0x20)
BMI2_DRDY_GYR_MASK = const(0x40)
BMI2_DRDY_ACC_MASK = const(0x80)

# SPI read/write address
BMI2_SPI_RD_MASK = const(0x80)
BMI2_SPI_WR_MASK = const(0x7F)

# Power configuration register
BMI2_ADV_POW_EN_MASK = const(0x01)
BMI2_FUP_EN_POS      = const(0x02)
BMI2_FUP_EN_MASK     = const(0x04)

# Init control
BMI2_CONF_LOAD_EN_MASK = const(0x01)

# Power control register
BMI2_AUX_EN_MASK  = const(0x01)
BMI2_GYR_EN_MASK  = const(0x02)
BMI2_ACC_EN_MASK  = const(0x04)
BMI2_TEMP_EN_MASK = const(0x08)

# Sensor event flags
BMI2_EVENT_FLAG_MASK = const(0x1C)

# Switch page
BMI2_SWITCH_PAGE_EN_MASK = const(0x07)

# NVM register
BMI2_NV_SPI_EN_MASK     = const(0x01)
BMI2_NV_I2C_WD_SEL_MASK = const(0x02)
BMI2_NV_I2C_WD_EN_MASK  = const(0x04)
BMI2_NV_ACC_OFFSET_MASK = const(0x08)

# DRV register
BMI2_DRV_STR_MASK = const(0xFF)

# Config version
BMI2_CONFIG_MAJOR_MASK = const(0x3C0)
BMI2_CONFIG_MINOR_MASK = const(0x3F)

# Activity recognition settings
BMI2_ACT_RECG_POST_PROS_EN_DIS_MASK = const(0x01)
BMI2_ACT_RECG_BUFF_SIZE_MASK        = const(0x0F)
BMI2_ACT_RECG_MIN_SEG_CONF_MASK     = const(0x0F)

# Activity recognition hc settings
BMI2_HC_ACT_RECG_SEGMENT_SIZE_MASK  = const(0x03)
BMI2_HC_ACT_RECG_PP_EN_MASK         = const(0x01)
BMI2_HC_ACT_RECG_MIN_GDI_THRES_MASK = const(0xFFFF)
BMI2_HC_ACT_RECG_MAX_GDI_THRES_MASK = const(0xFFFF)
BMI2_HC_ACT_RECG_BUF_SIZE_MASK      = const(0xFFFF)
BMI2_HC_ACT_RECG_MIN_SEG_CONF_MASK  = const(0xFFFF)

BMI2_GYRO_CROSS_AXES_SENSE_MASK          = const(0x7F)
BMI2_GYRO_CROSS_AXES_SENSE_SIGN_BIT_MASK = const(0x40)

# Bit positions for Gyro CRT
BMI2_GYR_RDY_FOR_DL_POS  = const(0x03)
BMI2_GYR_CRT_RUNNING_POS = const(0x02)

# Bit positions for status register
BMI2_AUX_BUSY_POS = const(0x02)
BMI2_CMD_RDY_POS  = const(0x04)
BMI2_DRDY_AUX_POS = const(0x05)
BMI2_DRDY_GYR_POS = const(0x06)
BMI2_DRDY_ACC_POS = const(0x07)

# Internal error register
BMI2_INTERNAL_ERROR_REG_POS = const(0x00)

# Power control register bit positions
BMI2_GYR_EN_POS  = const(0x01)
BMI2_ACC_EN_POS  = const(0x02)
BMI2_TEMP_EN_POS = const(0x03)

# Sensor event flags
BMI2_EVENT_FLAG_POS = const(0x02)

# NVM register bit positions
BMI2_NV_SPI_EN_POS     = const(0x00)
BMI2_NV_I2C_WD_SEL_POS = const(0x01)
BMI2_NV_I2C_WD_EN_POS  = const(0x02)
BMI2_NV_ACC_OFFSET_POS = const(0x03)

# DRV register
BMI2_DRV_STR_POS = const(0x00)

# Config major version
BMI2_CONFIG_MAJOR_POS = const(0x06)

# Filter/Noise performance modes
BMI2_POWER_OPT_MODE = const(0)
BMI2_PERF_OPT_MODE  = const(1)

# Index for config major/minor information
BMI2_CONFIG_INFO_LOWER  = const(52)
BMI2_CONFIG_INFO_HIGHER = const(53)

# Sensor status
BMI2_DRDY_ACC = const(0x80)
BMI2_DRDY_GYR = const(0x40)
BMI2_DRDY_AUX = const(0x20)
BMI2_CMD_RDY  = const(0x10)
BMI2_AUX_BUSY = const(0x04)

# Accelerometer / gyroscope configuration values for FOC
BMI2_FOC_ACC_CONF_VAL = const(0xB7)
BMI2_FOC_GYR_CONF_VAL = const(0xB6)

# X Y Z axis
BMI2_X_AXIS = const(0)
BMI2_Y_AXIS = const(1)
BMI2_Z_AXIS = const(2)

BMI2_FOC_INVERT_VALUE = const(-1)

# Delay for internal status read
BMI2_INTERNAL_STATUS_READ_DELAY_MS = const(20000)

# =============================================================================
# Sensor Macro Definitions
# =============================================================================

BMI2_ACCEL                        = const(0)
BMI2_GYRO                         = const(1)
BMI2_AUX                          = const(2)
BMI2_SIG_MOTION                   = const(3)
BMI2_ANY_MOTION                   = const(4)
BMI2_NO_MOTION                    = const(5)
BMI2_STEP_DETECTOR                = const(6)
BMI2_STEP_COUNTER                 = const(7)
BMI2_STEP_ACTIVITY                = const(8)
BMI2_GYRO_GAIN_UPDATE             = const(9)
BMI2_TILT                         = const(10)
BMI2_UP_HOLD_TO_WAKE              = const(11)
BMI2_GLANCE_DETECTOR              = const(12)
BMI2_WAKE_UP                      = const(13)
BMI2_ORIENTATION                  = const(14)
BMI2_HIGH_G                       = const(15)
BMI2_LOW_G                        = const(16)
BMI2_FLAT                         = const(17)
BMI2_EXT_SENS_SYNC                = const(18)
BMI2_WRIST_GESTURE                = const(19)
BMI2_WRIST_WEAR_WAKE_UP           = const(20)
BMI2_WRIST_WEAR_WAKE_UP_WH        = const(21)
BMI2_WRIST_GESTURE_WH             = const(22)
BMI2_PRIMARY_OIS                  = const(23)
BMI2_FREE_FALL_DET                = const(24)
BMI2_SINGLE_TAP                   = const(25)
BMI2_DOUBLE_TAP                   = const(26)
BMI2_TRIPLE_TAP                   = const(27)
BMI2_TAP                          = const(28)

# Non virtual sensor features
BMI2_STEP_COUNTER_PARAMS           = const(29)
BMI2_TAP_DETECTOR_1                = const(30)
BMI2_TAP_DETECTOR_2                = const(31)
BMI2_TEMP                          = const(32)
BMI2_ACCEL_SELF_TEST               = const(33)
BMI2_GYRO_SELF_OFF                 = const(34)
BMI2_ACTIVITY_RECOGNITION          = const(35)
BMI2_MAX_BURST_LEN                 = const(36)
BMI2_SENS_MAX_NUM                  = const(37)
BMI2_AXIS_MAP                      = const(38)
BMI2_NVM_STATUS                    = const(39)
BMI2_VFRM_STATUS                   = const(40)
BMI2_GYRO_CROSS_SENSE              = const(41)
BMI2_CRT_GYRO_SELF_TEST            = const(42)
BMI2_ABORT_CRT_GYRO_SELF_TEST      = const(43)
BMI2_NVM_PROG_PREP                 = const(44)
BMI2_ACTIVITY_RECOGNITION_SETTINGS = const(45)
BMI2_OIS_OUTPUT                    = const(46)
BMI2_CONFIG_ID                     = const(47)
BMI2_EXT_TCO                       = const(48)
BMI2_LPD                           = const(49)
BMI2_LAPTOP_POSITION_DETECTOR_1    = const(50)
BMI2_LAPTOP_POSITION_DETECTOR_2    = const(51)
BMI2_LAPTOP_POSITION_DETECTOR_3    = const(52)
BMI2_LAPTOP_POSITION_DETECTOR_4    = const(53)
BMI2_WRIST_GESTURE_WH_1            = const(54)
BMI2_WRIST_GESTURE_WH_2            = const(55)
BMI2_WRIST_WEAR_DROP_WH            = const(56)
BMI2_DOOR_STATE_DETECTOR           = const(57)

# Bit-wise selection of BMI2 sensors/features.
# These are composite expressions; they cannot be const() because const()
# only accepts literal integer expressions in MicroPython.
BMI2_ACCEL_SENS_SEL             = 1
BMI2_GYRO_SENS_SEL              = (1 << BMI2_GYRO)
BMI2_AUX_SENS_SEL               = (1 << BMI2_AUX)
BMI2_TEMP_SENS_SEL              = (1 << BMI2_TEMP)
BMI2_ANY_MOT_SEL                = (1 << BMI2_ANY_MOTION)
BMI2_NO_MOT_SEL                 = (1 << BMI2_NO_MOTION)
BMI2_TILT_SEL                   = (1 << BMI2_TILT)
BMI2_ORIENT_SEL                 = (1 << BMI2_ORIENTATION)
BMI2_SIG_MOTION_SEL             = (1 << BMI2_SIG_MOTION)
BMI2_STEP_DETECT_SEL            = (1 << BMI2_STEP_DETECTOR)
BMI2_STEP_COUNT_SEL             = (1 << BMI2_STEP_COUNTER)
BMI2_STEP_ACT_SEL               = (1 << BMI2_STEP_ACTIVITY)
BMI2_GYRO_GAIN_UPDATE_SEL       = (1 << BMI2_GYRO_GAIN_UPDATE)
BMI2_UP_HOLD_TO_WAKE_SEL        = (1 << BMI2_UP_HOLD_TO_WAKE)
BMI2_GLANCE_DET_SEL             = (1 << BMI2_GLANCE_DETECTOR)
BMI2_WAKE_UP_SEL                = (1 << BMI2_WAKE_UP)
BMI2_TAP_SEL                    = (1 << BMI2_TAP)
BMI2_HIGH_G_SEL                 = (1 << BMI2_HIGH_G)
BMI2_LOW_G_SEL                  = (1 << BMI2_LOW_G)
BMI2_FLAT_SEL                   = (1 << BMI2_FLAT)
BMI2_EXT_SENS_SEL               = (1 << BMI2_EXT_SENS_SYNC)
BMI2_SINGLE_TAP_SEL             = (1 << BMI2_SINGLE_TAP)
BMI2_DOUBLE_TAP_SEL             = (1 << BMI2_DOUBLE_TAP)
BMI2_TRIPLE_TAP_SEL             = (1 << BMI2_TRIPLE_TAP)
BMI2_GYRO_SELF_OFF_SEL          = (1 << BMI2_GYRO_SELF_OFF)
BMI2_WRIST_GEST_SEL             = (1 << BMI2_WRIST_GESTURE)
BMI2_WRIST_WEAR_WAKE_UP_SEL     = (1 << BMI2_WRIST_WEAR_WAKE_UP)
BMI2_ACTIVITY_RECOGNITION_SEL   = (1 << BMI2_ACTIVITY_RECOGNITION)
BMI2_ACCEL_SELF_TEST_SEL        = (1 << BMI2_ACCEL_SELF_TEST)
BMI2_WRIST_GEST_W_SEL           = (1 << BMI2_WRIST_GESTURE_WH)
BMI2_WRIST_WEAR_WAKE_UP_WH_SEL  = (1 << BMI2_WRIST_WEAR_WAKE_UP_WH)
BMI2_PRIMARY_OIS_SEL            = (1 << BMI2_PRIMARY_OIS)
BMI2_FREE_FALL_DET_SEL          = (1 << BMI2_FREE_FALL_DET)
BMI2_EXT_TCO_SEL                = (1 << BMI2_EXT_TCO)
BMI2_LPD_SEL                    = (1 << BMI2_LPD)
BMI2_WRIST_WEAR_DROP_WH_SEL     = (1 << BMI2_WRIST_WEAR_DROP_WH)
BMI2_DOOR_STATE_DETECTOR_SEL    = (1 << BMI2_DOOR_STATE_DETECTOR)

# Internal error
BMI2_INTERNAL_ERROR_REG_MASK = const(0xFF)
BMI2_INTERNAL_ERROR_1_MASK   = const(0x02)
BMI2_INTERNAL_ERROR_2_MASK   = const(0x04)
BMI2_FEAT_ENG_DIS_MASK       = const(0x10)

# Saturation register
BMI2_SATURATION_REG_MASK   = const(0x3F)
BMI2_SATURATION_ACC_X_MASK = const(0x01)
BMI2_SATURATION_ACC_Y_MASK = const(0x02)
BMI2_SATURATION_ACC_Z_MASK = const(0x04)
BMI2_SATURATION_GYR_X_MASK = const(0x08)
BMI2_SATURATION_GYR_Y_MASK = const(0x10)
BMI2_SATURATION_GYR_Z_MASK = const(0x20)

# BMI2 wake-up feature (bmi260)
BMI2_WAKEUP_SENSITIVITY_MASK   = const(0x0E)
BMI2_WAKEUP_SINGLE_TAP_EN_MASK = const(0x01)
BMI2_WAKEUP_DOUBLE_TAP_EN_MASK = const(0x02)
BMI2_WAKEUP_TRIPLE_TAP_EN_MASK = const(0x04)
BMI2_WAKEUP_DATA_REG_EN_MASK   = const(0x08)
BMI2_WAKEUP_AXIS_SEL_MASK      = const(0x03)

BMI2_WAKEUP_SENSITIVITY_POS   = const(0x01)
BMI2_WAKEUP_DOUBLE_TAP_EN_POS = const(0x01)
BMI2_WAKEUP_TRIPLE_TAP_EN_POS = const(0x02)
BMI2_WAKEUP_DATA_REG_EN_POS   = const(0x03)

# BMI2 tap feature (bmi260t)
BMI2_TAP_SENSITIVITY_MASK   = const(0x0E)
BMI2_TAP_SINGLE_TAP_EN_MASK = const(0x01)
BMI2_TAP_DOUBLE_TAP_EN_MASK = const(0x02)
BMI2_TAP_TRIPLE_TAP_EN_MASK = const(0x04)
BMI2_TAP_DATA_REG_EN_MASK   = const(0x08)
BMI2_TAP_AXIS_SEL_MASK      = const(0x03)

BMI2_TAP_SENSITIVITY_POS   = const(0x01)
BMI2_TAP_DOUBLE_TAP_EN_POS = const(0x01)
BMI2_TAP_TRIPLE_TAP_EN_POS = const(0x02)
BMI2_TAP_DATA_REG_EN_POS   = const(0x03)

# BMI2 wake-up feature (other than bmi261)
BMI2_WAKE_UP_SENSITIVITY_MASK   = const(0x000E)
BMI2_WAKE_UP_SINGLE_TAP_EN_MASK = const(0x0010)

BMI2_WAKE_UP_SENSITIVITY_POS   = const(0x01)
BMI2_WAKE_UP_SINGLE_TAP_EN_POS = const(0x04)

# Offsets from feature start address for feature enable/disable
BMI2_ANY_MOT_FEAT_EN_OFFSET       = const(0x03)
BMI2_NO_MOT_FEAT_EN_OFFSET        = const(0x03)
BMI2_SIG_MOT_FEAT_EN_OFFSET       = const(0x0A)
BMI2_STEP_COUNT_FEAT_EN_OFFSET    = const(0x01)
BMI2_GYR_USER_GAIN_FEAT_EN_OFFSET = const(0x05)
BMI2_HIGH_G_FEAT_EN_OFFSET        = const(0x03)
BMI2_LOW_G_FEAT_EN_OFFSET         = const(0x03)
BMI2_TILT_FEAT_EN_OFFSET          = const(0x00)

# Feature enable/disable masks
BMI2_ANY_NO_MOT_EN_MASK              = const(0x80)
BMI2_TILT_FEAT_EN_MASK               = const(0x01)
BMI2_ORIENT_FEAT_EN_MASK             = const(0x01)
BMI2_SIG_MOT_FEAT_EN_MASK            = const(0x01)
BMI2_STEP_DET_FEAT_EN_MASK           = const(0x08)
BMI2_STEP_COUNT_FEAT_EN_MASK         = const(0x10)
BMI2_STEP_ACT_FEAT_EN_MASK           = const(0x20)
BMI2_GYR_USER_GAIN_FEAT_EN_MASK      = const(0x08)
BMI2_UP_HOLD_TO_WAKE_FEAT_EN_MASK    = const(0x01)
BMI2_GLANCE_FEAT_EN_MASK             = const(0x01)
BMI2_WAKE_UP_FEAT_EN_MASK            = const(0x01)
BMI2_TAP_FEAT_EN_MASK                = const(0x01)
BMI2_HIGH_G_FEAT_EN_MASK             = const(0x80)
BMI2_LOW_G_FEAT_EN_MASK              = const(0x10)
BMI2_FLAT_FEAT_EN_MASK               = const(0x01)
BMI2_EXT_SENS_SYNC_FEAT_EN_MASK      = const(0x01)
BMI2_GYR_SELF_OFF_CORR_FEAT_EN_MASK  = const(0x02)
BMI2_WRIST_GEST_FEAT_EN_MASK         = const(0x20)
BMI2_WRIST_WEAR_WAKE_UP_FEAT_EN_MASK = const(0x10)
BMI2_WRIST_WEAR_DROP_FEAT_EN_MASK    = const(0x20)
BMI2_ACTIVITY_RECOG_EN_MASK          = const(0x01)
BMI2_ACC_SELF_TEST_FEAT_EN_MASK      = const(0x02)
BMI2_GYRO_SELF_TEST_CRT_EN_MASK      = const(0x01)
BMI2_ABORT_FEATURE_EN_MASK           = const(0x02)
BMI2_NVM_PREP_FEATURE_EN_MASK        = const(0x04)
BMI2_FREE_FALL_DET_FEAT_EN_MASK      = const(0x01)
BMI2_WRIST_GEST_WH_FEAT_EN_MASK      = const(0x02)
BMI2_DOOR_STATE_DETECTOR_FEAT_EN_MASK = const(0x01)

# Feature enable/disable bit positions
BMI2_ANY_NO_MOT_EN_POS              = const(0x07)
BMI2_STEP_DET_FEAT_EN_POS           = const(0x03)
BMI2_STEP_COUNT_FEAT_EN_POS         = const(0x04)
BMI2_STEP_ACT_FEAT_EN_POS           = const(0x05)
BMI2_GYR_USER_GAIN_FEAT_EN_POS      = const(0x03)
BMI2_HIGH_G_FEAT_EN_POS             = const(0x07)
BMI2_LOW_G_FEAT_EN_POS              = const(0x04)
BMI2_GYR_SELF_OFF_CORR_FEAT_EN_POS  = const(0x01)
BMI2_WRIST_GEST_FEAT_EN_POS         = const(0x05)
BMI2_WRIST_WEAR_WAKE_UP_FEAT_EN_POS = const(0x04)
BMI2_WRIST_WEAR_DROP_FEAT_EN_POS    = const(0x05)
BMI2_ACC_SELF_TEST_FEAT_EN_POS      = const(0x01)
BMI2_ABORT_FEATURE_EN_POS           = const(0x1)
BMI2_NVM_PREP_FEATURE_EN_POS        = const(0x02)
BMI2_WRIST_GEST_WH_FEAT_EN_POS      = const(0x01)

# BMI2 Error register
BMI2_ERR_REG_READ_MASK = const(0xFF)
BMI2_ERR_REG_READ_POS  = const(0x00)

BMI2_FATAL_ERR_MASK = const(0x01)
BMI2_FATAL_ERR_POS  = const(0x00)

BMI2_INTERNAL_ERR_MASK = const(0x1E)
BMI2_INTERNAL_ERR_POS  = const(0x02)

BMI2_FIFO_ERR_MASK = const(0x40)
BMI2_FIFO_ERR_POS  = const(0x06)

BMI2_AUX_ERR_MASK = const(0x80)
BMI2_AUX_ERR_POS  = const(0x07)

# Primary OIS low pass filter
BMI2_LP_FILTER_EN_MASK       = const(0x01)
BMI2_LP_FILTER_CONFIG_POS    = const(0x01)
BMI2_LP_FILTER_CONFIG_MASK   = const(0x06)
BMI2_PRIMARY_OIS_GYR_EN_POS  = const(0x06)
BMI2_PRIMARY_OIS_GYR_EN_MASK = const(0x40)
BMI2_PRIMARY_OIS_ACC_EN_POS  = const(0x07)
BMI2_PRIMARY_OIS_ACC_EN_MASK = const(0x80)

# Any- and no-motion feature
BMI2_ANY_NO_MOT_DUR_MASK   = const(0x1FFF)
BMI2_ANY_NO_MOT_X_SEL_MASK = const(0x2000)
BMI2_ANY_NO_MOT_Y_SEL_MASK = const(0x4000)
BMI2_ANY_NO_MOT_Z_SEL_MASK = const(0x8000)
BMI2_ANY_NO_MOT_THRES_MASK = const(0x07FF)
BMI2_ANY_MOT_INT_MASK      = const(0x40)

# No-motion interrupt mapping
BMI2_NO_MOT_INT_MASK = const(0x20)

# Any/no-motion feature bit positions
BMI2_ANY_NO_MOT_X_SEL_POS = const(0x0D)
BMI2_ANY_NO_MOT_Y_SEL_POS = const(0x0E)
BMI2_ANY_NO_MOT_Z_SEL_POS = const(0x0F)

# Orientation feature configuration
BMI2_ORIENT_UP_DOWN_MASK    = const(0x0002)
BMI2_ORIENT_SYMM_MODE_MASK  = const(0x000C)
BMI2_ORIENT_BLOCK_MODE_MASK = const(0x0030)
BMI2_ORIENT_THETA_MASK      = const(0x0FC0)
BMI2_ORIENT_HYST_MASK       = const(0x07FF)

BMI2_ORIENT_UP_DOWN_POS    = const(0x01)
BMI2_ORIENT_SYMM_MODE_POS  = const(0x02)
BMI2_ORIENT_BLOCK_MODE_POS = const(0x04)
BMI2_ORIENT_THETA_POS      = const(0x06)

# Sig-motion feature
BMI2_SIG_MOT_PARAM_1_MASK = const(0xFFFF)
BMI2_SIG_MOT_PARAM_2_MASK = const(0xFFFF)
BMI2_SIG_MOT_PARAM_3_MASK = const(0xFFFF)
BMI2_SIG_MOT_PARAM_4_MASK = const(0xFFFF)
BMI2_SIG_MOT_PARAM_5_MASK = const(0xFFFF)

# Parameter configurations
BMI2_STEP_COUNT_PARAMS_MASK = const(0xFFFF)

# Step-counter/detector feature
BMI2_STEP_COUNT_WM_LEVEL_MASK = const(0x03FF)
BMI2_STEP_COUNT_RST_CNT_MASK  = const(0x0400)
BMI2_STEP_BUFFER_SIZE_MASK    = const(0xFF00)
BMI2_STEP_COUNT_INT_MASK      = const(0x02)
BMI2_STEP_ACT_INT_MASK        = const(0x04)

BMI2_STEP_COUNT_RST_CNT_POS = const(0x0A)
BMI2_STEP_BUFFER_SIZE_POS   = const(0x08)

# Gyroscope user gain feature
BMI2_GYR_USER_GAIN_RATIO_X_MASK = const(0x07FF)
BMI2_GYR_USER_GAIN_RATIO_Y_MASK = const(0x07FF)
BMI2_GYR_USER_GAIN_RATIO_Z_MASK = const(0x07FF)

# Gyroscope user gain saturation status
BMI2_GYR_USER_GAIN_SAT_STAT_X_MASK = const(0x01)
BMI2_GYR_USER_GAIN_SAT_STAT_Y_MASK = const(0x02)
BMI2_GYR_USER_GAIN_SAT_STAT_Z_MASK = const(0x04)
BMI2_G_TRIGGER_STAT_MASK           = const(0x38)

BMI2_GYR_USER_GAIN_SAT_STAT_Y_POS = const(0x01)
BMI2_GYR_USER_GAIN_SAT_STAT_Z_POS = const(0x02)
BMI2_G_TRIGGER_STAT_POS           = const(0x03)

# MSB values of gyroscope compensation
BMI2_GYR_OFF_COMP_MSB_X_MASK = const(0x03)
BMI2_GYR_OFF_COMP_MSB_Y_MASK = const(0x0C)
BMI2_GYR_OFF_COMP_MSB_Z_MASK = const(0x30)

BMI2_GYR_OFF_COMP_MSB_Y_POS = const(0x02)
BMI2_GYR_OFF_COMP_MSB_Z_POS = const(0x04)

# Gyroscope compensation from user input
BMI2_GYR_OFF_COMP_MSB_MASK = const(0x0300)
BMI2_GYR_OFF_COMP_LSB_MASK = const(0x00FF)

# Orientation status
BMI2_ORIENT_DETECT_MASK      = const(0x03)
BMI2_ORIENT_FACE_UP_DWN_MASK = const(0x04)

BMI2_ORIENT_FACE_UP_DWN_POS = const(0x02)

# NVM-VFRM error status
BMI2_NVM_LOAD_ERR_STATUS_MASK   = const(0x01)
BMI2_NVM_PROG_ERR_STATUS_MASK   = const(0x02)
BMI2_NVM_ERASE_ERR_STATUS_MASK  = const(0x04)
BMI2_NVM_END_EXCEED_STATUS_MASK = const(0x08)
BMI2_NVM_PRIV_ERR_STATUS_MASK   = const(0x10)
BMI2_VFRM_LOCK_ERR_STATUS_MASK  = const(0x20)
BMI2_VFRM_WRITE_ERR_STATUS_MASK = const(0x40)
BMI2_VFRM_FATAL_ERR_STATUS_MASK = const(0x80)

BMI2_NVM_PROG_ERR_STATUS_POS   = const(0x01)
BMI2_NVM_ERASE_ERR_STATUS_POS  = const(0x02)
BMI2_NVM_END_EXCEED_STATUS_POS = const(0x03)
BMI2_NVM_PRIV_ERR_STATUS_POS   = const(0x04)
BMI2_VFRM_LOCK_ERR_STATUS_POS  = const(0x05)
BMI2_VFRM_WRITE_ERR_STATUS_POS = const(0x06)
BMI2_VFRM_FATAL_ERR_STATUS_POS = const(0x07)

# Accelerometer self-test status
BMI2_ACC_SELF_TEST_DONE_MASK = const(0x01)
BMI2_ACC_X_OK_MASK           = const(0x02)
BMI2_ACC_Y_OK_MASK           = const(0x04)
BMI2_ACC_Z_OK_MASK           = const(0x08)

BMI2_ACC_X_OK_POS = const(0x01)
BMI2_ACC_Y_OK_POS = const(0x02)
BMI2_ACC_Z_OK_POS = const(0x03)

# High-g feature
BMI2_HIGH_G_THRES_MASK = const(0x7FFF)
BMI2_HIGH_G_HYST_MASK  = const(0x0FFF)
BMI2_HIGH_G_X_SEL_MASK = const(0x1000)
BMI2_HIGH_G_Y_SEL_MASK = const(0x2000)
BMI2_HIGH_G_Z_SEL_MASK = const(0x4000)
BMI2_HIGH_G_DUR_MASK   = const(0x0FFF)

BMI2_HIGH_G_X_SEL_POS = const(0x0C)
BMI2_HIGH_G_Y_SEL_POS = const(0x0D)
BMI2_HIGH_G_Z_SEL_POS = const(0x0E)

# Low-g feature
BMI2_LOW_G_THRES_MASK = const(0x7FFF)
BMI2_LOW_G_HYST_MASK  = const(0x0FFF)
BMI2_LOW_G_DUR_MASK   = const(0x0FFF)

# Free-fall detection feature
BMI2_FREE_FALL_ACCEL_SETT_MASK = const(0xFFFF)

# Flat feature
BMI2_FLAT_THETA_MASK     = const(0x007E)
BMI2_FLAT_BLOCK_MASK     = const(0x0180)
BMI2_FLAT_HYST_MASK      = const(0x003F)
BMI2_FLAT_HOLD_TIME_MASK = const(0x3FC0)

BMI2_FLAT_THETA_POS     = const(0x01)
BMI2_FLAT_BLOCK_POS     = const(0x07)
BMI2_FLAT_HOLD_TIME_POS = const(0x06)

# Wrist gesture configuration
BMI2_WRIST_GEST_WEAR_ARM_MASK = const(0x0010)
BMI2_WRIST_GEST_WEAR_ARM_POS  = const(0x04)

# Wrist gesture wh configuration
BMI2_WRIST_GEST_WH_DEVICE_POS_MASK = const(0x0001)
BMI2_WRIST_GEST_WH_INT             = const(0x10)
BMI2_WRIST_GEST_WH_START_ADD       = const(0x08)

# Wrist wear wake-up
BMI2_WRIST_WAKE_UP_WH_INT_MASK = const(0x08)

# Wrist wear wake-up (wearable variant)
BMI2_WRIST_WAKE_UP_ANGLE_LR_MASK             = const(0x00FF)
BMI2_WRIST_WAKE_UP_ANGLE_LL_MASK             = const(0xFF00)
BMI2_WRIST_WAKE_UP_ANGLE_PD_MASK             = const(0x00FF)
BMI2_WRIST_WAKE_UP_ANGLE_PU_MASK             = const(0xFF00)
BMI2_WRIST_WAKE_UP_MIN_DUR_MOVED_MASK        = const(0x00FF)
BMI2_WRIST_WAKE_UP_MIN_DUR_QUITE_MASK        = const(0xFF00)
BMI2_WRIST_WAKE_UP_MIN_DROP_POS_DUR_MASK     = const(0x00FF)
BMI2_WRIST_WAKE_UP_MIN_DROP_POS_DUR_LOC_MASK = const(0xFF00)

BMI2_WRIST_WAKE_UP_ANGLE_LL_POS             = const(0x0008)
BMI2_WRIST_WAKE_UP_ANGLE_PU_POS             = const(0x0008)
BMI2_WRIST_WAKE_UP_MIN_DUR_QUITE_POS        = const(0x0008)
BMI2_WRIST_WAKE_UP_MIN_DROP_POS_DUR_LOC_POS = const(0x0008)

# EXT TCO
BMI2_EXT_TCO_MASK = const(0x01)

# Axis re-map
BMI2_MAP_X_AXIS   = const(0x00)
BMI2_MAP_Y_AXIS   = const(0x01)
BMI2_MAP_Z_AXIS   = const(0x02)
BMI2_MAP_POSITIVE = const(0x00)
BMI2_MAP_NEGATIVE = const(0x01)

BMI2_X_AXIS_MASK      = const(0x03)
BMI2_X_AXIS_SIGN_MASK = const(0x04)
BMI2_Y_AXIS_MASK      = const(0x18)
BMI2_Y_AXIS_SIGN_MASK = const(0x20)
BMI2_Z_AXIS_MASK      = const(0xC0)
BMI2_Z_AXIS_SIGN_MASK = const(0x01)

BMI2_X_AXIS_SIGN_POS = const(0x02)
BMI2_Y_AXIS_POS      = const(0x03)
BMI2_Y_AXIS_SIGN_POS = const(0x05)
BMI2_Z_AXIS_POS      = const(0x06)

# Virtual frame
BMI2_EIS_VFRM_DATA_MASK = const(0x08)
BMI2_EIS_VFRM_DATA_POS  = const(3)

BMI2_VIRTUAL_FRAME_LEN = const(19)

# Polarity
BMI2_NEG_SIGN = const(1)
BMI2_POS_SIGN = const(0)

# CRT
BMI2_CRT_READY_FOR_DOWNLOAD_US         = const(2000)
BMI2_CRT_READY_FOR_DOWNLOAD_RETRY      = const(100)
BMI2_CRT_WAIT_RUNNING_US               = const(10000)
BMI2_CRT_WAIT_RUNNING_RETRY_EXECUTION  = const(200)
BMI2_CRT_MIN_BURST_WORD_LENGTH         = const(2)
BMI2_CRT_MAX_BURST_WORD_LENGTH         = const(255)

# Gyro FOC references (dps)
BMI2_GYRO_FOC_2000_DPS_REF = const(16)
BMI2_GYRO_FOC_1000_DPS_REF = const(33)
BMI2_GYRO_FOC_500_DPS_REF  = const(66)
BMI2_GYRO_FOC_250_DPS_REF  = const(131)
BMI2_GYRO_FOC_125_DPS_REF  = const(262)

# Accelerometer FOC references and offsets
BMI2_ACC_FOC_2G_REF  = const(16384)
BMI2_ACC_FOC_4G_REF  = const(8192)
BMI2_ACC_FOC_8G_REF  = const(4096)
BMI2_ACC_FOC_16G_REF = const(2048)

BMI2_ACC_FOC_2G_OFFSET  = const(328)
BMI2_ACC_FOC_4G_OFFSET  = const(164)
BMI2_ACC_FOC_8G_OFFSET  = const(82)
BMI2_ACC_FOC_16G_OFFSET = const(41)

BMI2_FOC_SAMPLE_LIMIT = const(128)

# Composite expressions (no const() - only literals allowed there)
BMI2_ACC_2G_MAX_NOISE_LIMIT  = (BMI2_ACC_FOC_2G_REF + BMI2_ACC_FOC_2G_OFFSET)
BMI2_ACC_2G_MIN_NOISE_LIMIT  = (BMI2_ACC_FOC_2G_REF - BMI2_ACC_FOC_2G_OFFSET)
BMI2_ACC_4G_MAX_NOISE_LIMIT  = (BMI2_ACC_FOC_4G_REF + BMI2_ACC_FOC_4G_OFFSET)
BMI2_ACC_4G_MIN_NOISE_LIMIT  = (BMI2_ACC_FOC_4G_REF - BMI2_ACC_FOC_4G_OFFSET)
BMI2_ACC_8G_MAX_NOISE_LIMIT  = (BMI2_ACC_FOC_8G_REF + BMI2_ACC_FOC_8G_OFFSET)
BMI2_ACC_8G_MIN_NOISE_LIMIT  = (BMI2_ACC_FOC_8G_REF - BMI2_ACC_FOC_8G_OFFSET)
BMI2_ACC_16G_MAX_NOISE_LIMIT = (BMI2_ACC_FOC_16G_REF + BMI2_ACC_FOC_16G_OFFSET)
BMI2_ACC_16G_MIN_NOISE_LIMIT = (BMI2_ACC_FOC_16G_REF - BMI2_ACC_FOC_16G_OFFSET)

# Bit wise selection of BMI2 main sensors
BMI2_MAIN_SENSORS = (BMI2_ACCEL_SENS_SEL | BMI2_GYRO_SENS_SEL
                     | BMI2_AUX_SENS_SEL | BMI2_TEMP_SENS_SEL)

BMI2_MAIN_SENS_MAX_NUM = const(4)

BMI2_STEP_CNT_N_PARAMS          = const(25)
BMI2_FREE_FALL_ACCEL_SET_PARAMS = const(7)

BMI2_SELECT_GYRO_SELF_TEST = const(0)
BMI2_SELECT_CRT            = const(1)

# NVM enable
BMI2_NVM_UNLOCK_ENABLE  = const(0x02)
BMI2_NVM_UNLOCK_DISABLE = const(0x00)

# Gyro self test / CRT selection
BMI2_GYRO_SELF_TEST_SEL = const(0)
BMI2_CRT_SEL            = const(1)

# =============================================================================
# Accelerometer Macro Definitions
# =============================================================================

# Accelerometer Bandwidth parameters
BMI2_ACC_OSR4_AVG1   = const(0x00)
BMI2_ACC_OSR2_AVG2   = const(0x01)
BMI2_ACC_NORMAL_AVG4 = const(0x02)
BMI2_ACC_CIC_AVG8    = const(0x03)
BMI2_ACC_RES_AVG16   = const(0x04)
BMI2_ACC_RES_AVG32   = const(0x05)
BMI2_ACC_RES_AVG64   = const(0x06)
BMI2_ACC_RES_AVG128  = const(0x07)

# Accelerometer Output Data Rate
BMI2_ACC_ODR_0_78HZ = const(0x01)
BMI2_ACC_ODR_1_56HZ = const(0x02)
BMI2_ACC_ODR_3_12HZ = const(0x03)
BMI2_ACC_ODR_6_25HZ = const(0x04)
BMI2_ACC_ODR_12_5HZ = const(0x05)
BMI2_ACC_ODR_25HZ   = const(0x06)
BMI2_ACC_ODR_50HZ   = const(0x07)
BMI2_ACC_ODR_100HZ  = const(0x08)
BMI2_ACC_ODR_200HZ  = const(0x09)
BMI2_ACC_ODR_400HZ  = const(0x0A)
BMI2_ACC_ODR_800HZ  = const(0x0B)
BMI2_ACC_ODR_1600HZ = const(0x0C)

# Accelerometer G Range
BMI2_ACC_RANGE_2G  = const(0x00)
BMI2_ACC_RANGE_4G  = const(0x01)
BMI2_ACC_RANGE_8G  = const(0x02)
BMI2_ACC_RANGE_16G = const(0x03)

# Accelerometer config register masks
BMI2_ACC_RANGE_MASK            = const(0x03)
BMI2_ACC_ODR_MASK              = const(0x0F)
BMI2_ACC_BW_PARAM_MASK         = const(0x70)
BMI2_ACC_FILTER_PERF_MODE_MASK = const(0x80)

BMI2_ACC_BW_PARAM_POS         = const(0x04)
BMI2_ACC_FILTER_PERF_MODE_POS = const(0x07)

# Self test range
BMI2_ACC_SELF_TEST_RANGE = const(16)

# Self test min diff in mg
BMI2_ST_ACC_X_SIG_MIN_DIFF = const(16000)
BMI2_ST_ACC_Y_SIG_MIN_DIFF = const(-15000)
BMI2_ST_ACC_Z_SIG_MIN_DIFF = const(10000)

# Self-test masks
BMI2_ACC_SELF_TEST_EN_MASK   = const(0x01)
BMI2_ACC_SELF_TEST_SIGN_MASK = const(0x04)
BMI2_ACC_SELF_TEST_AMP_MASK  = const(0x08)

BMI2_ACC_SELF_TEST_SIGN_POS = const(0x02)
BMI2_ACC_SELF_TEST_AMP_POS  = const(0x03)

# Gyro self test status
BMI2_GYR_ST_AXES_DONE_MASK = const(0x01)
BMI2_GYR_AXIS_X_OK_MASK    = const(0x02)
BMI2_GYR_AXIS_Y_OK_MASK    = const(0x04)
BMI2_GYR_AXIS_Z_OK_MASK    = const(0x08)

BMI2_GYR_AXIS_X_OK_POS = const(0x01)
BMI2_GYR_AXIS_Y_OK_POS = const(0x02)
BMI2_GYR_AXIS_Z_OK_POS = const(0x03)

# =============================================================================
# Gyroscope Macro Definitions
# =============================================================================

BMI2_GYR_OSR4_MODE   = const(0x00)
BMI2_GYR_OSR2_MODE   = const(0x01)
BMI2_GYR_NORMAL_MODE = const(0x02)
BMI2_GYR_CIC_MODE    = const(0x03)

BMI2_GYR_ODR_25HZ   = const(0x06)
BMI2_GYR_ODR_50HZ   = const(0x07)
BMI2_GYR_ODR_100HZ  = const(0x08)
BMI2_GYR_ODR_200HZ  = const(0x09)
BMI2_GYR_ODR_400HZ  = const(0x0A)
BMI2_GYR_ODR_800HZ  = const(0x0B)
BMI2_GYR_ODR_1600HZ = const(0x0C)
BMI2_GYR_ODR_3200HZ = const(0x0D)

BMI2_GYR_OIS_250  = const(0x00)
BMI2_GYR_OIS_2000 = const(0x01)

BMI2_GYR_RANGE_2000 = const(0x00)
BMI2_GYR_RANGE_1000 = const(0x01)
BMI2_GYR_RANGE_500  = const(0x02)
BMI2_GYR_RANGE_250  = const(0x03)
BMI2_GYR_RANGE_125  = const(0x04)

BMI2_GYR_RANGE_MASK            = const(0x07)
BMI2_GYR_OIS_RANGE_MASK        = const(0x08)
BMI2_GYR_ODR_MASK              = const(0x0F)
BMI2_GYR_BW_PARAM_MASK         = const(0x30)
BMI2_GYR_NOISE_PERF_MODE_MASK  = const(0x40)
BMI2_GYR_FILTER_PERF_MODE_MASK = const(0x80)

BMI2_GYR_OIS_RANGE_POS        = const(0x03)
BMI2_GYR_BW_PARAM_POS         = const(0x04)
BMI2_GYR_NOISE_PERF_MODE_POS  = const(0x06)
BMI2_GYR_FILTER_PERF_MODE_POS = const(0x07)

# =============================================================================
# Auxiliary Macro Definitions
# =============================================================================

BMI2_AUX_ODR_RESERVED = const(0x00)
BMI2_AUX_ODR_0_78HZ   = const(0x01)
BMI2_AUX_ODR_1_56HZ   = const(0x02)
BMI2_AUX_ODR_3_12HZ   = const(0x03)
BMI2_AUX_ODR_6_25HZ   = const(0x04)
BMI2_AUX_ODR_12_5HZ   = const(0x05)
BMI2_AUX_ODR_25HZ     = const(0x06)
BMI2_AUX_ODR_50HZ     = const(0x07)
BMI2_AUX_ODR_100HZ    = const(0x08)
BMI2_AUX_ODR_200HZ    = const(0x09)
BMI2_AUX_ODR_400HZ    = const(0x0A)
BMI2_AUX_ODR_800HZ    = const(0x0B)

# Burst read lengths for manual and auto modes
BMI2_AUX_READ_LEN_0 = const(0x00)
BMI2_AUX_READ_LEN_1 = const(0x01)
BMI2_AUX_READ_LEN_2 = const(0x02)
BMI2_AUX_READ_LEN_3 = const(0x03)

BMI2_AUX_RD_BURST_FRM_LEN_1 = const(1)
BMI2_AUX_RD_BURST_FRM_LEN_2 = const(2)
BMI2_AUX_RD_BURST_FRM_LEN_6 = const(6)
BMI2_AUX_RD_BURST_FRM_LEN_8 = const(8)

# Auxiliary interface configuration register
BMI2_AUX_SET_I2C_ADDR_MASK    = const(0xFE)
BMI2_AUX_MAN_MODE_EN_MASK     = const(0x80)
BMI2_AUX_FCU_WR_EN_MASK       = const(0x40)
BMI2_AUX_MAN_READ_BURST_MASK  = const(0x0C)
BMI2_AUX_READ_BURST_MASK      = const(0x03)
BMI2_AUX_ODR_EN_MASK          = const(0x0F)
BMI2_AUX_OFFSET_READ_OUT_MASK = const(0xF0)

BMI2_AUX_SET_I2C_ADDR_POS    = const(0x01)
BMI2_AUX_MAN_MODE_EN_POS     = const(0x07)
BMI2_AUX_FCU_WR_EN_POS       = const(0x06)
BMI2_AUX_MAN_READ_BURST_POS  = const(0x02)
BMI2_AUX_OFFSET_READ_OUT_POS = const(0x04)

# =============================================================================
# FIFO Macro Definitions
# =============================================================================

BMI2_FIFO_VIRT_FRM_MODE = const(0x03)

# FIFO Header masks
BMI2_FIFO_HEADER_ACC_FRM       = const(0x84)
BMI2_FIFO_HEADER_AUX_FRM       = const(0x90)
BMI2_FIFO_HEADER_GYR_FRM       = const(0x88)
BMI2_FIFO_HEADER_GYR_ACC_FRM   = const(0x8C)
BMI2_FIFO_HEADER_AUX_ACC_FRM   = const(0x94)
BMI2_FIFO_HEADER_AUX_GYR_FRM   = const(0x98)
BMI2_FIFO_HEADER_ALL_FRM       = const(0x9C)
BMI2_FIFO_HEADER_SENS_TIME_FRM = const(0x44)
BMI2_FIFO_HEADER_SKIP_FRM      = const(0x40)
BMI2_FIFO_HEADER_INPUT_CFG_FRM = const(0x48)
BMI2_FIFO_HEAD_OVER_READ_MSB   = const(0x80)
BMI2_FIFO_VIRT_ACT_RECOG_FRM   = const(0xC8)

# Sensor selection for header-less frames
BMI2_FIFO_HEAD_LESS_ACC_FRM     = const(0x40)
BMI2_FIFO_HEAD_LESS_AUX_FRM     = const(0x20)
BMI2_FIFO_HEAD_LESS_GYR_FRM     = const(0x80)
BMI2_FIFO_HEAD_LESS_GYR_AUX_FRM = const(0xA0)
BMI2_FIFO_HEAD_LESS_GYR_ACC_FRM = const(0xC0)
BMI2_FIFO_HEAD_LESS_AUX_ACC_FRM = const(0x60)
BMI2_FIFO_HEAD_LESS_ALL_FRM     = const(0xE0)

# FIFO frame content config masks
BMI2_FIFO_STOP_ON_FULL = const(0x0001)
BMI2_FIFO_TIME_EN      = const(0x0002)
BMI2_FIFO_TAG_INT1     = const(0x0300)
BMI2_FIFO_TAG_INT2     = const(0x0C00)
BMI2_FIFO_HEADER_EN    = const(0x1000)
BMI2_FIFO_AUX_EN       = const(0x2000)
BMI2_FIFO_ACC_EN       = const(0x4000)
BMI2_FIFO_GYR_EN       = const(0x8000)
BMI2_FIFO_ALL_EN       = const(0xE000)

# Sensortime resolution in seconds (float literal, not const)
BMI2_SENSORTIME_RESOLUTION = 0.0000390625

# FIFO sensor data lengths
BMI2_FIFO_ACC_LENGTH       = const(6)
BMI2_FIFO_GYR_LENGTH       = const(6)
BMI2_FIFO_ACC_GYR_LENGTH   = const(12)
BMI2_SENSOR_TIME_LENGTH    = const(3)
BMI2_FIFO_CONFIG_LENGTH    = const(2)
BMI2_FIFO_WM_LENGTH        = const(2)
BMI2_MAX_VALUE_FIFO_FILTER = const(1)
BMI2_FIFO_DATA_LENGTH      = const(2)
BMI2_FIFO_LENGTH_MSB_BYTE  = const(1)
BMI2_FIFO_INPUT_CFG_LENGTH = const(4)
BMI2_FIFO_SKIP_FRM_LENGTH  = const(1)

# FIFO virtual data lengths (sensor data + sensor time)
BMI2_FIFO_VIRT_ACC_LENGTH     = const(9)
BMI2_FIFO_VIRT_GYR_LENGTH     = const(9)
BMI2_FIFO_VIRT_AUX_LENGTH     = const(11)
BMI2_FIFO_VIRT_ACC_AUX_LENGTH = const(17)
BMI2_FIFO_VIRT_GYR_AUX_LENGTH = const(17)
BMI2_FIFO_VIRT_ACC_GYR_LENGTH = const(15)
BMI2_FIFO_VIRT_ALL_LENGTH     = const(23)

# FIFO virtual data lengths: activity recognition
BMI2_FIFO_VIRT_ACT_DATA_LENGTH = const(6)
BMI2_FIFO_VIRT_ACT_TIME_LENGTH = const(4)
BMI2_FIFO_VIRT_ACT_TYPE_LENGTH = const(1)
BMI2_FIFO_VIRT_ACT_STAT_LENGTH = const(1)

# FIFO data filter modes
BMI2_FIFO_UNFILTERED_DATA = const(0)
BMI2_FIFO_FILTERED_DATA   = const(1)

# FIFO frame masks
BMI2_FIFO_LSB_CONFIG_CHECK = const(0x00)
BMI2_FIFO_MSB_CONFIG_CHECK = const(0x80)
BMI2_FIFO_TAG_INTR_MASK    = const(0xFF)

# FIFO configuration registers
BMI2_FIFO_CONFIG_0_MASK = const(0x0003)
BMI2_FIFO_CONFIG_1_MASK = const(0xFF00)

# FIFO self wake-up
BMI2_FIFO_SELF_WAKE_UP_MASK = const(0x02)

# FIFO down sampling
BMI2_ACC_FIFO_DOWNS_MASK = const(0x70)
BMI2_GYR_FIFO_DOWNS_MASK = const(0x07)
BMI2_ACC_FIFO_DOWNS_POS  = const(0x04)

# FIFO filter
BMI2_ACC_FIFO_FILT_DATA_MASK = const(0x80)
BMI2_GYR_FIFO_FILT_DATA_MASK = const(0x08)
BMI2_ACC_FIFO_FILT_DATA_POS  = const(0x07)
BMI2_GYR_FIFO_FILT_DATA_POS  = const(0x03)

# FIFO byte counter
BMI2_FIFO_BYTE_COUNTER_MSB_MASK = const(0x3F)

# FIFO self wake-up
BMI2_FIFO_SELF_WAKE_UP_POS = const(0x01)

# Virtual FIFO frames
BMI2_FIFO_VIRT_FRM_MODE_MASK = const(0xC0)
BMI2_FIFO_VIRT_PAYLOAD_MASK  = const(0x3C)

BMI2_FIFO_VIRT_FRM_MODE_POS = const(0x06)
BMI2_FIFO_VIRT_PAYLOAD_POS  = const(0x02)

# =============================================================================
# Interrupt Macro Definitions
# =============================================================================

# Interrupt modes
BMI2_INT_NON_LATCH = const(0)
BMI2_INT_LATCH     = const(1)

# Interrupt Pin Behavior
BMI2_INT_PUSH_PULL  = const(0)
BMI2_INT_OPEN_DRAIN = const(1)

# Interrupt Pin Level
BMI2_INT_ACTIVE_LOW  = const(0)
BMI2_INT_ACTIVE_HIGH = const(1)

# Interrupt Output Enable
BMI2_INT_OUTPUT_DISABLE = const(0)
BMI2_INT_OUTPUT_ENABLE  = const(1)

# Interrupt Input Enable
BMI2_INT_INPUT_DISABLE = const(0)
BMI2_INT_INPUT_ENABLE  = const(1)

# Interrupt pin configuration masks
BMI2_INT_LATCH_MASK      = const(0x01)
BMI2_INT_LEVEL_MASK      = const(0x02)
BMI2_INT_OPEN_DRAIN_MASK = const(0x04)
BMI2_INT_OUTPUT_EN_MASK  = const(0x08)
BMI2_INT_INPUT_EN_MASK   = const(0x10)

BMI2_INT_LEVEL_POS      = const(0x01)
BMI2_INT_OPEN_DRAIN_POS = const(0x02)
BMI2_INT_OUTPUT_EN_POS  = const(0x03)
BMI2_INT_INPUT_EN_POS   = const(0x04)

# Data interrupt mapping
BMI2_FFULL_INT = const(0x01)
BMI2_FWM_INT   = const(0x02)
BMI2_DRDY_INT  = const(0x04)
BMI2_ERR_INT   = const(0x08)

# Data interrupt status bits
BMI2_FFULL_INT_STATUS_MASK = const(0x0100)
BMI2_FWM_INT_STATUS_MASK   = const(0x0200)
BMI2_ERR_INT_STATUS_MASK   = const(0x0400)
BMI2_AUX_DRDY_INT_MASK     = const(0x2000)
BMI2_GYR_DRDY_INT_MASK     = const(0x4000)
BMI2_ACC_DRDY_INT_MASK     = const(0x8000)

# Maximum number of interrupt pins
BMI2_INT_PIN_MAX_NUM = const(2)

# Feature interrupt mapping
BMI2_FEAT_BIT_DISABLE = const(0)
BMI2_FEAT_BIT0        = const(1)
BMI2_FEAT_BIT1        = const(2)
BMI2_FEAT_BIT2        = const(3)
BMI2_FEAT_BIT3        = const(4)
BMI2_FEAT_BIT4        = const(5)
BMI2_FEAT_BIT5        = const(6)
BMI2_FEAT_BIT6        = const(7)
BMI2_FEAT_BIT7        = const(8)
BMI2_FEAT_BIT_MAX     = const(9)

# =============================================================================
# OIS Interface Macro Definitions
# =============================================================================

BMI2_SPI3_MODE_MASK  = const(0x01)
BMI2_SPI3_OIS_MASK   = const(0x02)
BMI2_OIS_IF_EN_MASK  = const(0x10)
BMI2_AUX_IF_EN_MASK  = const(0x20)

BMI2_SPI3_MODE_POS  = const(0x00)
BMI2_SPI3_OIS_POS   = const(0x01)
BMI2_OIS_IF_EN_POS  = const(0x04)
BMI2_AUX_IF_EN_POS  = const(0x05)

# =============================================================================
# Axes re-mapping Macro Definitions
# =============================================================================

BMI2_X         = const(0x01)
BMI2_NEG_X     = const(0x09)
BMI2_Y         = const(0x02)
BMI2_NEG_Y     = const(0x0A)
BMI2_Z         = const(0x04)
BMI2_NEG_Z     = const(0x0C)
BMI2_AXIS_MASK = const(0x07)
BMI2_AXIS_SIGN = const(0x08)

# =============================================================================
# Offset and gain compensation
# =============================================================================

BMI2_GYR_GAIN_EN_MASK     = const(0x80)
BMI2_GYR_OFF_COMP_EN_MASK = const(0x40)
BMI2_GYR_OFF_COMP_EN_POS  = const(0x06)

BMI2_GYR_USR_GAIN_X_MASK = const(0x7F)
BMI2_GYR_USR_GAIN_Y_MASK = const(0x7F)
BMI2_GYR_USR_GAIN_Z_MASK = const(0x7F)

BMI2_GYR_GAIN_EN_POS = const(0x07)

# =============================================================================
# Internal status
# =============================================================================

BMI2_NOT_INIT         = const(0x00)
BMI2_INIT_OK          = const(0x01)
BMI2_INIT_ERR         = const(0x02)
BMI2_DRV_ERR          = const(0x03)
BMI2_SNS_STOP         = const(0x04)
BMI2_NVM_ERROR        = const(0x05)
BMI2_START_UP_ERROR   = const(0x06)
BMI2_COMPAT_ERROR     = const(0x07)
BMI2_VFM_SKIPPED      = const(0x10)
BMI2_AXES_MAP_ERROR   = const(0x20)
BMI2_ODR_50_HZ_ERROR  = const(0x40)
BMI2_ODR_HIGH_ERROR   = const(0x80)

# =============================================================================
# Error status from gyro gain update status
# =============================================================================

BMI2_G_TRIGGER_NO_ERROR     = const(0x00)
BMI2_G_TRIGGER_PRECON_ERROR = const(0x01)
BMI2_G_TRIGGER_DL_ERROR     = const(0x02)
BMI2_G_TRIGGER_ABORT_ERROR  = const(0x03)

# =============================================================================
# Variant specific features selection macros
# =============================================================================

BMI2_CRT_RTOSK_ENABLE       = const(0x01)
BMI2_GYRO_CROSS_SENS_ENABLE = const(0x02)
BMI2_GYRO_USER_GAIN_ENABLE  = const(0x08)
BMI2_NO_FEATURE_ENABLE      = const(0x00)
BMI2_CRT_IN_FIFO_NOT_REQ    = const(0x10)
BMI2_MAXIMUM_FIFO_VARIANT   = const(0x20)

# Pull-up configuration for ASDA
BMI2_ASDA_PUPSEL_OFF = const(0x00)
BMI2_ASDA_PUPSEL_40K = const(0x01)
BMI2_ASDA_PUPSEL_10K = const(0x02)
BMI2_ASDA_PUPSEL_2K  = const(0x03)


# =============================================================================
# Enum declarations (exposed as module-level constants and namespace classes)
# =============================================================================

# Enum bmi2_intf
BMI2_SPI_INTF = const(0)
BMI2_I2C_INTF = const(1)
BMI2_I3C_INTF = const(2)


class Bmi2Intf:
    """Namespace for BMI2 sensor interfaces (enum bmi2_intf)."""
    SPI = BMI2_SPI_INTF
    I2C = BMI2_I2C_INTF
    I3C = BMI2_I3C_INTF


# Enum bmi2_sensor_config_error
BMI2_NO_ERROR      = const(0)
BMI2_ACC_ERROR     = const(1)
BMI2_GYR_ERROR     = const(2)
BMI2_ACC_GYR_ERROR = const(3)


class Bmi2SensorConfigError:
    """Namespace for BMI2 sensor configuration errors."""
    NO_ERROR      = BMI2_NO_ERROR
    ACC_ERROR     = BMI2_ACC_ERROR
    GYR_ERROR     = BMI2_GYR_ERROR
    ACC_GYR_ERROR = BMI2_ACC_GYR_ERROR


# Enum bmi2_hw_int_pin
BMI2_INT_NONE    = const(0)
BMI2_INT1        = const(1)
BMI2_INT2        = const(2)
BMI2_INT_BOTH    = const(3)
BMI2_INT_PIN_MAX = const(4)


class Bmi2HwIntPin:
    """Namespace for BMI2 interrupt pins."""
    NONE    = BMI2_INT_NONE
    INT1    = BMI2_INT1
    INT2    = BMI2_INT2
    BOTH    = BMI2_INT_BOTH
    PIN_MAX = BMI2_INT_PIN_MAX


# Enum bmi2_wear_arm_pos
BMI2_ARM_LEFT  = const(0)
BMI2_ARM_RIGHT = const(1)


class Bmi2WearArmPos:
    """Namespace for the position of the wearable device."""
    LEFT  = BMI2_ARM_LEFT
    RIGHT = BMI2_ARM_RIGHT


# Enum bmi2_act_recog_type
BMI2_ACT_UNKNOWN = const(0)
BMI2_ACT_STILL   = const(1)
BMI2_ACT_WALK    = const(2)
BMI2_ACT_RUN     = const(3)
BMI2_ACT_BIKE    = const(4)
BMI2_ACT_VEHICLE = const(5)
BMI2_ACT_TILTED  = const(6)


class Bmi2ActRecogType:
    """Namespace for the type of activity recognition."""
    UNKNOWN = BMI2_ACT_UNKNOWN
    STILL   = BMI2_ACT_STILL
    WALK    = BMI2_ACT_WALK
    RUN     = BMI2_ACT_RUN
    BIKE    = BMI2_ACT_BIKE
    VEHICLE = BMI2_ACT_VEHICLE
    TILTED  = BMI2_ACT_TILTED


# Enum bmi2_act_recog_stat (values start at 1)
BMI2_ACT_START = const(1)
BMI2_ACT_END   = const(2)


class Bmi2ActRecogStat:
    """Namespace for activity recognition status."""
    START = BMI2_ACT_START
    END   = BMI2_ACT_END




# =============================================================================
# FORMAT_ERROR enum (exposed as module-level constants and namespace class)
# =============================================================================
Bmi2FormatError=(
'BMI2_E_NULL_PTR',#= const(-1)
'BMI2_E_COM_FAIL',#= const(-2)
'BMI2_E_DEV_NOT_FOUND',#= const(-3)
'BMI2_E_OUT_OF_RANGE',#= const(-4)
'BMI2_E_ACC_INVALID_CFG',#= const(-5)
'BMI2_E_GYRO_INVALID_CFG',#= const(-6)
'BMI2_E_ACC_GYR_INVALID_CFG',#= const(-7)
'BMI2_E_INVALID_SENSOR',#= const(-8)
'BMI2_E_CONFIG_LOAD',#= const(-9)
'BMI2_E_INVALID_PAGE',#= const(-10)
'BMI2_E_INVALID_FEAT_BIT',#= const(-11)
'BMI2_E_INVALID_INT_PIN',#= const(-12)
'BMI2_E_SET_APS_FAIL',#= const(-13)
'BMI2_E_AUX_INVALID_CFG',#= const(-14)
'BMI2_E_AUX_BUSY',#= const(-15)
'BMI2_E_SELF_TEST_FAIL',#= const(-16)
'BMI2_E_REMAP_ERROR',#= const(-17)
'BMI2_E_GYR_USER_GAIN_UPD_FAIL',#= const(-18)
'BMI2_E_SELF_TEST_NOT_DONE',#= const(-19)
'BMI2_E_INVALID_INPUT',#= const(-20)
'BMI2_E_INVALID_STATUS',#= const(-21)
'BMI2_E_CRT_ERROR',#= const(-22)
'BMI2_E_ST_ALREADY_RUNNING',#= const(-23)
'BMI2_E_CRT_READY_FOR_DL_FAIL_ABORT',#= const(-24)
'BMI2_E_DL_ERROR',#= const(-25)
'BMI2_E_PRECON_ERROR',#= const(-26)
'BMI2_E_ABORT_ERROR',#= const(-27)
'BMI2_E_GYRO_SELF_TEST_ERROR',#= const(-28)
'BMI2_E_GYRO_SELF_TEST_TIMEOUT',#= const(-29)
'BMI2_E_WRITE_CYCLE_ONGOING',#= const(-30)
'BMI2_E_WRITE_CYCLE_TIMEOUT',#= const(-31)
'BMI2_E_ST_NOT_RUNING',# = const(-32)
'BMI2_E_DATA_RDY_INT_FAILED',#= const(-33)
'BMI2_E_INVALID_FOC_POSITION',#= const(-34)
)
def format_error_code(code):
    """Format an error code as a string."""
    if code >= 0:
        return f"OK ({code})"
    index = -code - 1
    if 0 <= index < len(Bmi2FormatError):
        return f"{Bmi2FormatError[index]} ({code})"
    return f"Unknown error code ({code})"


# =============================================================================
# Structure / Union Python classes
# =============================================================================

class Bmi2GyroUserGainData:
    """Compensated user-gain data of gyroscope (struct bmi2_gyro_user_gain_data)."""

    def __init__(self):
        self.x = 0
        self.y = 0
        self.z = 0


class Bmi2Remap:
    """Re-mapped axis (struct bmi2_remap)."""

    def __init__(self):
        self.x = 0
        self.y = 0
        self.z = 0


class Bmi2AxesRemap:
    """Re-mapped axis value and sign (struct bmi2_axes_remap)."""

    def __init__(self):
        self.x_axis = 0
        self.y_axis = 0
        self.z_axis = 0
        self.x_axis_sign = 0
        self.y_axis_sign = 0
        self.z_axis_sign = 0


class Bmi2SensIntConfig:
    """Type of sensor and its interrupt pin (struct bmi2_sens_int_config)."""

    def __init__(self):
        self.type = 0
        self.hw_int_pin = 0  # enum bmi2_hw_int_pin


class Bmi2ActRecogOutput:
    """Output for activity recognition (struct bmi2_act_recog_output)."""

    def __init__(self):
        self.time_stamp = 0
        self.curr_act = 0
        self.prev_act = 0


class Bmi2FifoFrame:
    """FIFO frame configuration (struct bmi2_fifo_frame)."""

    def __init__(self):
        # In C: uint8_t *data — in Python use a bytearray / memoryview / None
        self.data = None
        self.length = 0
        self.header_enable = 0
        self.data_enable = 0
        self.acc_byte_start_idx = 0
        self.act_recog_byte_start_idx = 0
        self.aux_byte_start_idx = 0
        self.gyr_byte_start_idx = 0
        self.sensor_time = 0
        self.skipped_frame_count = 0
        self.data_int_map = 0
        self.wm_lvl = 0
        self.acc_frm_len = 0
        self.gyr_frm_len = 0
        self.aux_frm_len = 0
        self.acc_gyr_frm_len = 0
        self.acc_aux_frm_len = 0
        self.aux_gyr_frm_len = 0
        self.all_frm_len = 0


class Bmi2IntPinCfg:
    """Interrupt pin configuration (struct bmi2_int_pin_cfg)."""

    def __init__(self):
        self.lvl = 0
        self.od = 0
        self.output_en = 0
        self.input_en = 0


class Bmi2IntPinConfig:
    """Interrupt pin type, mode and configurations (struct bmi2_int_pin_config)."""

    def __init__(self):
        self.pin_type = 0
        self.int_latch = 0
        # Array of Bmi2IntPinCfg of length BMI2_INT_PIN_MAX_NUM
        self.pin_cfg = [Bmi2IntPinCfg() for _ in range(BMI2_INT_PIN_MAX_NUM)]


class Bmi2AuxFifoData:
    """Array of 8 auxiliary data bytes (struct bmi2_aux_fifo_data)."""

    def __init__(self):
        self.data = bytearray(8)
        self.virt_sens_time = 0


class Bmi2SensAxesData:
    """Sensor axes data (struct bmi2_sens_axes_data).

    Values x, y, z are signed 16-bit integers in the C source; Python stores
    them as normal ints.
    """

    def __init__(self):
        self.x = 0
        self.y = 0
        self.z = 0
        self.virt_sens_time = 0


class Bmi2GyrUserGainStatus:
    """Gyroscope saturation status of user gain (struct bmi2_gyr_user_gain_status)."""

    def __init__(self):
        self.sat_x = 0
        self.sat_y = 0
        self.sat_z = 0
        self.g_trigger_status = 0


class Bmi2GyroSelfTestStatus:
    """Status of gyro self test result (struct bmi2_gyro_self_test_status)."""

    def __init__(self):
        self.gyr_st_axes_done = 0
        self.gyr_axis_x_ok = 0
        self.gyr_axis_y_ok = 0
        self.gyr_axis_z_ok = 0


class Bmi2NvmErrStatus:
    """NVM error status (struct bmi2_nvm_err_status)."""

    def __init__(self):
        self.load_error = 0
        self.prog_error = 0
        self.erase_error = 0
        self.exceed_error = 0
        self.privil_error = 0


class Bmi2VfrmErrStatus:
    """VFRM error status (struct bmi2_vfrm_err_status)."""

    def __init__(self):
        self.lock_error = 0
        self.write_error = 0
        self.fatal_error = 0


class Bmi2AccSelfTestStatus:
    """Accelerometer self test feature status (struct bmi2_acc_self_test_status)."""

    def __init__(self):
        self.acc_self_test_done = 0
        self.acc_x_ok = 0
        self.acc_y_ok = 0
        self.acc_z_ok = 0


class Bmi2OrientationOutput:
    """Orientation output (struct bmi2_orientation_output)."""

    def __init__(self):
        self.portrait_landscape = 0
        self.faceup_down = 0


class Bmi2OisOutput:
    """OIS output (struct bmi2_ois_output)."""

    def __init__(self):
        self.ois_acc_x = 0
        self.ois_acc_y = 0
        self.ois_acc_z = 0
        self.ois_gyro_x = 0
        self.ois_gyro_y = 0
        self.ois_gyro_z = 0


class Bmi2SensData:
    """BMI2 sensor data (struct bmi2_sens_data)."""

    def __init__(self):
        self.acc = Bmi2SensAxesData()
        self.gyr = Bmi2SensAxesData()
        self.aux_data = bytearray(BMI2_AUX_NUM_BYTES)
        self.sens_time = 0
        self.status = 0


class Bmi2DoorStateDetectorOutput:
    """DSD output (struct bmi2_door_state_detector_output)."""

    def __init__(self):
        self.door_event_output = 0
        self.heading_output = 0


class Bmi2FeatureData:
    """Feature data union (union bmi2_feature_data).

    All union fields are exposed as attributes. Only one should be used at a
    time (exclusivity by convention).
    """

    def __init__(self):
        self.step_counter_output = 0
        self.activity_output = 0
        self.orient_output = Bmi2OrientationOutput()
        self.high_g_output = 0
        self.gyro_user_gain_status = Bmi2GyrUserGainStatus()
        self.nvm_status = Bmi2NvmErrStatus()
        self.vfrm_status = Bmi2VfrmErrStatus()
        self.wrist_gesture_output = 0
        self.wrist_wear_wakeup_output = 0
        self.correction_factor_zx = 0
        self.accel_self_test_output = Bmi2AccSelfTestStatus()
        self.ois_output = Bmi2OisOutput()
        self.door_state_detector_output = Bmi2DoorStateDetectorOutput()


class Bmi2FeatSensorData:
    """Type of sensor and their respective feature data (struct bmi2_feat_sensor_data)."""

    def __init__(self):
        self.type = 0
        self.sens_data = Bmi2FeatureData()


class Bmi2SensorData:
    """Type of sensor and respective data (struct bmi2_sensor_data)."""

    def __init__(self):
        self.type = 0
        self.sens_data = Bmi2SensData()


class Bmi2AccelConfig:
    """Accelerometer configuration (struct bmi2_accel_config)."""

    def __init__(self):
        self.odr = 0
        self.bwp = 0
        self.filter_perf = 0
        self.range = 0


class Bmi2GyroConfig:
    """Gyroscope configuration (struct bmi2_gyro_config)."""

    def __init__(self):
        self.odr = 0
        self.bwp = 0
        self.filter_perf = 0
        self.ois_range = 0
        self.range = 0
        self.noise_perf = 0


class Bmi2AuxConfig:
    """Auxiliary sensor configuration (struct bmi2_aux_config)."""

    def __init__(self):
        self.aux_en = 0
        self.manual_en = 0
        self.fcu_write_en = 0
        self.man_rd_burst = 0
        self.aux_rd_burst = 0
        self.odr = 0
        self.offset = 0
        self.i2c_device_addr = 0
        self.read_addr = 0


class Bmi2AnyMotionConfig:
    """Any-motion configuration (struct bmi2_any_motion_config)."""

    def __init__(self):
        self.duration = 0
        self.threshold = 0
        self.select_x = 0
        self.select_y = 0
        self.select_z = 0


class Bmi2NoMotionConfig:
    """No-motion configuration (struct bmi2_no_motion_config)."""

    def __init__(self):
        self.duration = 0
        self.threshold = 0
        self.select_x = 0
        self.select_y = 0
        self.select_z = 0


class Bmi2SigMotionConfig:
    """Sig-motion configuration (struct bmi2_sig_motion_config)."""

    def __init__(self):
        self.block_size = 0


class Bmi2ExtTco:
    """EXT TCO configuration (struct bmi2_ext_tco)."""

    def __init__(self):
        self.hw_comp_enable = 0


class Bmi2StepConfig:
    """Step counter/detector/activity configuration (struct bmi2_step_config)."""

    def __init__(self):
        self.watermark_level = 0
        self.reset_counter = 0
        self.step_buffer_size = 0


class Bmi2GyroUserGainConfig:
    """Gyroscope user gain configuration (struct bmi2_gyro_user_gain_config)."""

    def __init__(self):
        self.ratio_x = 0
        self.ratio_y = 0
        self.ratio_z = 0


class Bmi2WakeUpConfig:
    """Wake-up configuration (struct bmi2_wake_up_config)."""

    def __init__(self):
        self.sensitivity = 0
        self.single_tap_en = 0


class Bmi2TapConfig:
    """Tap configuration (struct bmi2_tap_config)."""

    def __init__(self):
        self.data_reg_en = 0
        self.tap_sens_thres = 0
        self.max_gest_dur = 0
        self.quite_time_after_gest = 0
        self.wait_for_timeout = 0
        self.axis_sel = 0


class Bmi2OrientConfig:
    """Orientation configuration (struct bmi2_orient_config)."""

    def __init__(self):
        self.ud_en = 0
        self.mode = 0
        self.blocking = 0
        self.theta = 0
        self.hysteresis = 0


class Bmi2HighGConfig:
    """High-g configuration (struct bmi2_high_g_config)."""

    def __init__(self):
        self.threshold = 0
        self.hysteresis = 0
        self.select_x = 0
        self.select_y = 0
        self.select_z = 0
        self.duration = 0


class Bmi2LowGConfig:
    """Low-g configuration (struct bmi2_low_g_config)."""

    def __init__(self):
        self.threshold = 0
        self.hysteresis = 0
        self.duration = 0


class Bmi2FlatConfig:
    """Flat configuration (struct bmi2_flat_config)."""

    def __init__(self):
        self.theta = 0
        self.blocking = 0
        self.hysteresis = 0
        self.hold_time = 0


class Bmi2WristGestConfig:
    """Wrist gesture configuration (struct bmi2_wrist_gest_config)."""

    def __init__(self):
        self.wearable_arm = 0
        self.min_flick_peak = 0
        self.min_flick_samples = 0
        self.max_duration = 0


class Bmi2WristWearWakeUpConfig:
    """Wrist wear wake-up configuration (struct bmi2_wrist_wear_wake_up_config)."""

    def __init__(self):
        self.min_angle_focus = 0
        self.min_angle_nonfocus = 0
        self.max_tilt_lr = 0
        self.max_tilt_ll = 0
        self.max_tilt_pd = 0
        self.max_tilt_pu = 0


class Bmi2WristWearWakeUpWhConfig:
    """Wrist wear wake-up (wearable) configuration (struct bmi2_wrist_wear_wake_up_wh_config)."""

    def __init__(self):
        self.min_angle_focus = 0
        self.min_angle_nonfocus = 0
        self.angle_landscape_right = 0
        self.angle_landscape_left = 0
        self.angle_portrait_down = 0
        self.angle_portrait_up = 0
        self.min_dur_moved = 0
        self.min_dur_quite = 0


class Bmi2PrimaryOisConfig:
    """Primary OIS configuration (struct bmi2_primary_ois_config)."""

    def __init__(self):
        self.lp_filter_enabled = 0
        self.lp_filter_config = 0
        self.gyr_en = 0
        self.acc_en = 0


class Bmi2FreeFallDetConfig:
    """Free-fall detection configuration (struct bmi2_free_fall_det_config)."""

    def __init__(self):
        self.freefall_accel_settings = [0] * BMI2_FREE_FALL_ACCEL_SET_PARAMS


class Bmi2WristGestWConfig:
    """Wrist gesture configuration for wearable variant (struct bmi2_wrist_gest_w_config)."""

    def __init__(self):
        self.device_position = 0
        self.min_flick_peak_y_threshold = 0
        self.min_flick_peak_z_threshold = 0
        self.gravity_bounds_x_pos = 0
        self.gravity_bounds_x_neg = 0
        self.gravity_bounds_y_neg = 0
        self.gravity_bounds_z_neg = 0
        self.flick_peak_decay_coeff = 0
        self.lp_mean_filter_coeff = 0
        self.max_duration_jiggle_peaks = 0


class Bmi2LpdConfig:
    """Laptop position recognition configuration (struct bmi2_lpd_config)."""

    def __init__(self):
        self.flit_data_en = 0
        self.lpd_enable = 0
        self.portrait_theta = 0
        self.portrait_hysteresis = 0
        self.landscape_theta = 0
        self.landscape_hysteresis = 0
        self.flat_posture_theta = 0
        self.flat_posture_hysteresis = 0
        self.blocking_mode = 0
        self.hold_time = 0
        self.blockingslope_thres = 0
        self.segment_size = 0
        self.post_processing_enable = 0
        self.mingdithreshold = 0
        self.maxgdithreshold = 0
        self.output_buffersize = 0
        self.minseg_moderateconf = 0


class Bmi2WristGestureConfig:
    """Wrist gesture configuration (struct bmi2_wrist_gesture_config)."""

    def __init__(self):
        self.min_flick_peak_y_threshold = 0
        self.min_flick_peak_z_threshold = 0
        self.gravity_bounds_x_pos = 0
        self.gravity_bounds_x_neg = 0
        self.gravity_bounds_y_neg = 0
        self.gravity_bounds_z_neg = 0
        self.flick_peak_decay_coeff = 0
        self.lp_mean_filter_coeff = 0
        self.max_duration_jiggle_peaks = 0
        self.device_position = 0


class Bmi2DoorStateDetectorConfig:
    """Door state detector configuration (struct bmi2_door_state_detector_config)."""

    def __init__(self):
        self.dsd_enable = 0
        self.remap_flag = 0
        self.z_sign = 0
        self.z_axis = 0
        self.init_calib_thr = 0
        self.reset_enable_flag = 0
        self.bias_x_low_word = 0
        self.bias_x_high_word = 0
        self.bias_y_low_word = 0
        self.bias_y_high_word = 0
        self.bias_z_low_word = 0
        self.bias_z_high_word = 0


class Bmi2SensConfigTypes:
    """Union of all sensor configuration types (union bmi2_sens_config_types).

    All fields are initialized; use the one appropriate for the selected sensor
    type.
    """

    def __init__(self):
        self.acc = Bmi2AccelConfig()
        self.gyr = Bmi2GyroConfig()
        self.aux = Bmi2AuxConfig()
        self.any_motion = Bmi2AnyMotionConfig()
        self.no_motion = Bmi2NoMotionConfig()
        self.sig_motion = Bmi2SigMotionConfig()
        self.ext_tco = Bmi2ExtTco()
        self.step_counter_params = [0] * BMI2_STEP_CNT_N_PARAMS
        self.step_counter = Bmi2StepConfig()
        self.gyro_gain_update = Bmi2GyroUserGainConfig()
        self.wake_up_conf = Bmi2WakeUpConfig()
        self.tap_conf = Bmi2TapConfig()
        self.orientation = Bmi2OrientConfig()
        self.high_g = Bmi2HighGConfig()
        self.low_g = Bmi2LowGConfig()
        self.flat = Bmi2FlatConfig()
        self.wrist_gest = Bmi2WristGestConfig()
        self.wrist_wear_wake_up = Bmi2WristWearWakeUpConfig()
        self.wrist_gest_w = Bmi2WristGestWConfig()
        self.wrist_wear_wake_up_wh = Bmi2WristWearWakeUpWhConfig()
        self.primary_ois = Bmi2PrimaryOisConfig()
        self.free_fall_det = Bmi2FreeFallDetConfig()
        self.lap_pos_det = Bmi2LpdConfig()
        self.wrist_g_config = Bmi2WristGestureConfig()
        self.door_state_detector = Bmi2DoorStateDetectorConfig()


class Bmi2SensConfig:
    """Type of the sensor and its configurations (struct bmi2_sens_config)."""

    def __init__(self):
        self.type = 0
        self.cfg = Bmi2SensConfigTypes()


class Bmi2FeatureConfig:
    """Feature configuration (struct bmi2_feature_config)."""

    def __init__(self):
        self.type = 0
        self.page = 0
        self.start_addr = 0


class Bmi2MapInt:
    """Feature interrupt configuration (struct bmi2_map_int)."""

    def __init__(self):
        self.type = 0
        self.sens_map_int = 0


class Bmi2AccelFocGValue:
    """Accel axis enable for FOC (struct bmi2_accel_foc_g_value)."""

    def __init__(self):
        self.x = 0
        self.y = 0
        self.z = 0
        self.sign = 0


class Bmi2ActRecgSett:
    """Activity recognition settings (struct bmi2_act_recg_sett)."""

    def __init__(self):
        self.pp_en = 0
        self.min_gdi_thres = 0
        self.max_gdi_thres = 0
        self.buf_size = 0
        self.min_seg_conf = 0


class Bmi2HcActRecgSett:
    """Activity recognition settings for bmi270hc (struct bmi2_hc_act_recg_sett)."""

    def __init__(self):
        self.segment_size = 0
        self.pp_en = 0
        self.min_gdi_thres = 0
        self.max_gdi_thres = 0
        self.buf_size = 0
        self.min_seg_conf = 0


# =============================================================================
# Bmi2Dev - main device structure (struct bmi2_dev)
# =============================================================================

class Bmi2Dev:
    """BMI2 sensor configurations (struct bmi2_dev).

    The `read`, `write`, `delay_us`, `get_wakeup_config`, `set_wakeup_config`,
    `get_tap_config` and `set_tap_config` attributes must be populated with
    Python callables before calling any function that accesses the device.

    Expected signatures:
        read(reg_addr, length) -> bytes/bytearray
        write(reg_addr, data:  bytes/bytearray) -> None or int
        delay_us(period_us) -> None
    """

    def __init__(self):
        # Chip id of BMI2
        self.chip_id = 0

        # Interface pointer (implementation-defined handle)
        self.intf_ptr = None

        # Warnings
        self.info = 0

        # Type of Interface (enum bmi2_intf)
        self.intf = 0

        # Interface pointer error (BMI2_INTF_RETURN_TYPE)
        self.intf_rslt = 0

        # For switching from I2C to SPI
        self.dummy_byte = 0

        # Resolution for FOC
        self.resolution = 0

        # User set read/write length
        self.read_write_len = 0

        # Load status
        self.load_status = 0

        # Pointer to configuration data buffer (bytes in Python)
        self.config_file_ptr = None

        # Maximum page number
        self.page_max = 0

        # Maximum number of input sensors/features
        self.input_sens = 0

        # Maximum number of output sensors/features
        self.out_sens = 0

        # Manual enable for auxiliary communication
        self.aux_man_en = 0

        # Manual read burst length for auxiliary communication
        self.aux_man_rd_burst_len = 0

        # Array of feature input configuration structures (list of Bmi2FeatureConfig)
        self.feat_config = None

        # Array of feature output configuration structures
        self.feat_output = None

        # Re-mapped axis
        self.remap = Bmi2AxesRemap()

        # Enable status of sensors (64-bit flags in C)
        self.sens_en_stat = 0

        # Function pointers — user-assigned callables
        self.read = None
        self.write = None
        self.delay_us = None

        # Gyroscope cross sensitivity value
        self.gyr_cross_sens_zx = 0

        # Gyro enable status (CRT flag)
        self.gyro_en = 0

        # Advance power saving mode status (CRT flag)
        self.aps_status = 0

        # Variant specific features flag
        self.variant_feature = 0

        # Size of config file
        self.config_size = 0

        # Function pointers for wakeup / tap configurations
        self.get_wakeup_config = None
        self.set_wakeup_config = None
        self.get_tap_config = None
        self.set_tap_config = None

        # Array of feature interrupts configuration structures
        self.map_int = None

        # Maximum number of interrupts
        self.sens_int_map = 0
