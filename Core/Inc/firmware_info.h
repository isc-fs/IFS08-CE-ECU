#ifndef FIRMWARE_INFO_H
#define FIRMWARE_INFO_H

#include <stdint.h>

/* Firmware identity from the bootloader fwinfo record (Core/Src/firmware_info.c),
 * surfaced for pit-diag. */
uint8_t        ecu_fw_version_major(void);
uint8_t        ecu_fw_version_minor(void);
uint8_t        ecu_fw_version_patch(void);
const uint8_t *ecu_git_hash(void);  /* 8 bytes; first 4 are the git short hash */

#endif /* FIRMWARE_INFO_H */
