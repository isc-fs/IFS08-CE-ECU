/* Emits the bl_fwinfo_t record that isc-fs/stm32-can-bootloader requires at a
 * fixed flash offset (app base + 0x400 = 0x08020400) to consider this app
 * valid. Without it the bootloader rejects JUMP with NO_VALID_APP even after a
 * successful FLASH_WRITE + FLASH_VERIFY.
 *
 * The struct layout + magic are taken verbatim from
 *   isc-fs/stm32-can-bootloader/Core/Inc/bl_fwinfo.h
 * We can't #include that header (the BL repo isn't a submodule of this tree),
 * so if the BL bumps the magic or record schema we must follow in lockstep.
 *
 * fw_version / git_hash / build_timestamp are populated from CMake at
 * configure time (ECU_FW_VERSION_*, ECU_GIT_HASH_*, ECU_BUILD_TIMESTAMP). When
 * built without those (e.g. a host TU that pulls this in), the fallbacks below
 * emit a zero identification block — still a valid, bootable record.
 */

#include <stdint.h>

#ifndef ECU_FW_VERSION_MAJOR
#  define ECU_FW_VERSION_MAJOR 0u
#endif
#ifndef ECU_FW_VERSION_MINOR
#  define ECU_FW_VERSION_MINOR 0u
#endif
#ifndef ECU_FW_VERSION_PATCH
#  define ECU_FW_VERSION_PATCH 0u
#endif
#ifndef ECU_GIT_HASH_0
#  define ECU_GIT_HASH_0 0u
#endif
#ifndef ECU_GIT_HASH_1
#  define ECU_GIT_HASH_1 0u
#endif
#ifndef ECU_GIT_HASH_2
#  define ECU_GIT_HASH_2 0u
#endif
#ifndef ECU_GIT_HASH_3
#  define ECU_GIT_HASH_3 0u
#endif
#ifndef ECU_BUILD_TIMESTAMP
#  define ECU_BUILD_TIMESTAMP 0u
#endif

typedef struct
{
  uint32_t magic;            /* offset 0  — BL_FWINFO_MAGIC */
  uint32_t record_version;   /* offset 4  — 0x00MM.mmmm (major/minor) */
  uint32_t fw_version_major; /* offset 8  */
  uint32_t fw_version_minor; /* offset 12 */
  uint32_t fw_version_patch; /* offset 16 */
  uint32_t mcu_id;           /* offset 20 — DBGMCU IDCODE family */
  uint8_t  git_hash[8];      /* offset 24 — first 8 bytes of SHA-1 */
  uint64_t build_timestamp;  /* offset 32 — unix seconds, little-endian */
  char     product_name[16]; /* offset 40 — ASCII, NUL-padded */
  uint32_t reserved[2];      /* offset 56 — zero-pad to 64 bytes */
} bl_fwinfo_t;

_Static_assert(sizeof(bl_fwinfo_t) == 64, "bl_fwinfo_t must be 64 bytes");

const bl_fwinfo_t __firmware_info
    __attribute__((section(".firmware_info"), used)) = {
    .magic            = 0xF14F1B00u,   /* BL_FWINFO_MAGIC — keep in sync with bl_fwinfo.h */
    .record_version   = 0x00010000u,   /* record schema 1.0 (BL accepts major >= 1) */
    .fw_version_major = ECU_FW_VERSION_MAJOR,
    .fw_version_minor = ECU_FW_VERSION_MINOR,
    .fw_version_patch = ECU_FW_VERSION_PATCH,
    .mcu_id           = 0x00000450u,   /* STM32H7x3 */
    /* First 4 bytes of the 32-bit git short hash; trailing 4 stay zero. */
    .git_hash         = { ECU_GIT_HASH_0, ECU_GIT_HASH_1,
                          ECU_GIT_HASH_2, ECU_GIT_HASH_3,
                          0u, 0u, 0u, 0u },
    .build_timestamp  = (uint64_t)ECU_BUILD_TIMESTAMP,
    .product_name     = "IFS08-CE-ECU",
    /* reserved[0]: per-board node ID on the bootloader's multi-node CAN bus
       (flasher cross-check). Left 0 until the ECU's BL node ID is assigned —
       must not collide with the AMS. reserved[1] free. */
    .reserved         = { 0u, 0u },
};
