"""bmi270_maximum_fifo.py -- Port MicroPython du driver Bosch BMI270 en mode FIFO maximal.

Source : bmi270_maximum_fifo.h + bmi270_maximum_fifo.c

Ce module expose uniquement bmi270_maximum_fifo_init(dev) qui configure
le BMI270 pour un FIFO de taille maximale. Aucune feature algorithmique
n'est activée dans cette variante — seul le streaming FIFO brut est supporté.

Firmware : même que bmi270.py (bmi270_config.bin). Copier sur la carte :
    mpremote mkdir /firmware
    mpremote cp micropython_bmi270/firmware/bmi270_config.bin :/firmware/bmi270_config.bin
"""

import os

from bmi2_defs import (
    BMI2_OK, BMI2_E_NULL_PTR, BMI2_SPI_INTF,
    BMI2_GYRO_CROSS_SENS_ENABLE, BMI2_CRT_RTOSK_ENABLE,
    BMI2_MAXIMUM_FIFO_VARIANT,
)
from bmi2 import bmi2_sec_init

# ---------------------------------------------------------------------------
# Constantes spécifiques BMI270_MAXIMUM_FIFO
# ---------------------------------------------------------------------------
BMI270_MAXIMUM_FIFO_CHIP_ID      = 0x24
BMI270_MAXIMUM_FIFO_MAX_PAGE_NUM = 0
BMI270_MAXIMUM_FIFO_MAX_FEAT_IN  = 0
BMI270_MAXIMUM_FIFO_MAX_FEAT_OUT = 0

# ---------------------------------------------------------------------------
# Firmware : même configuration que bmi270.py (variante standard)
# ---------------------------------------------------------------------------
# Mode fichier (recommandé) : chemin vers le binaire sur le système de fichiers.
# Mode inline : remplacer par bytes([...]) avec le contenu de bmi270_config_file[]
# depuis src/bmi270_api/bmi270.c (identique à la variante standard).
BMI270_MAXIMUM_FIFO_CONFIG_FILE = "/firmware/bmi270_config.bin"


def bmi270_maximum_fifo_init(dev):
    """Initialise le BMI270 en mode FIFO maximal.

    Équivalent de bmi270_maximum_fifo_init() dans bmi270_maximum_fifo.c.
    Active le FIFO en mode pleine taille, sans features algorithmiques.
    Utilise le même firmware que la variante BMI270 standard.

    Args:
        dev: instance de Bmi2Dev (voir bmi2.py).

    Returns:
        int8_t -- BMI2_OK (0) en cas de succès, code d'erreur négatif sinon.
    """
    if dev is None:
        return BMI2_E_NULL_PTR

    dev.chip_id = BMI270_MAXIMUM_FIFO_CHIP_ID

    if isinstance(BMI270_MAXIMUM_FIFO_CONFIG_FILE, str):
        dev.config_size = os.stat(BMI270_MAXIMUM_FIFO_CONFIG_FILE)[6]
    else:
        dev.config_size = len(BMI270_MAXIMUM_FIFO_CONFIG_FILE)

    dev.variant_feature = BMI2_GYRO_CROSS_SENS_ENABLE | BMI2_MAXIMUM_FIFO_VARIANT
    dev.dummy_byte = 1 if dev.intf == BMI2_SPI_INTF else 0
    dev.config_file_ptr = BMI270_MAXIMUM_FIFO_CONFIG_FILE

    rslt = bmi2_sec_init(dev)
    if rslt != BMI2_OK:
        return rslt

    dev.feat_config  = []
    dev.feat_output  = []
    dev.map_int      = []
    dev.page_max     = BMI270_MAXIMUM_FIFO_MAX_PAGE_NUM
    dev.input_sens   = BMI270_MAXIMUM_FIFO_MAX_FEAT_IN
    dev.out_sens     = BMI270_MAXIMUM_FIFO_MAX_FEAT_OUT
    dev.sens_int_map = 0
    dev.variant_feature = 0

    return BMI2_OK
