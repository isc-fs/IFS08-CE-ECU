// SPDX-License-Identifier: proprietary
//
// pedal_cal_nvm.hpp -- reading the stored pedal calibration out of the
// bootloader's NVM sector. READ ONLY; the write path is step 5 of #169.
//
// WHERE IT LIVES. The bootloader owns flash sector 7 (0x080E0000, 128 KB) as a
// log-structured key/value store. That sector is OUTSIDE the application region
// (sectors 1..6, 0x08020000..0x080DFFFF), so a firmware reflash does not erase
// it and a calibration survives a firmware update.
//
// WHY WE PARSE IT OURSELVES. bl_nvm lives in the bootloader's own sector and
// exposes no entry point callable from the running application, so the app has
// to read the format directly. For READS that is safe -- flash is memory
// mapped, we only load from it, and a misparse can at worst make us fall back
// to defaults. It would NOT be safe for writes: a second implementation of the
// append/compaction logic could corrupt the bootloader's own keys. Step 5 must
// settle that separately and must not simply extend this file.
//
// >>> THE CONSTANTS BELOW ARE DUPLICATED FROM THE BOOTLOADER. <<<
// Source of truth: isc-fs/stm32-can-bootloader
//   Core/Inc/bl_memmap.h : BL_NVM_BASE, BL_NVM_SIZE, BL_APP_METADATA_ADDR
//   Core/Inc/bl_nvm.h    : BL_NVM_ENTRY_SIZE, BL_NVM_ENTRY_MAGIC,
//                          BL_NVM_MAX_VALUE_LEN, the 32-byte entry layout
// If the bootloader changes any of them this file silently reads rubbish and
// falls back to defaults. Diff the two on any bootloader bump.

#ifndef PEDAL_CAL_NVM_HPP_
#define PEDAL_CAL_NVM_HPP_

#include <cstddef>
#include <cstdint>

#include "app/pedal_cal.hpp"

namespace ecu {

// ---- mirrored bootloader geometry (see the warning above) ----
inline constexpr std::uint32_t CalNvmBase      = 0x080E0000u;
inline constexpr std::uint32_t CalNvmSize      = (128u * 1024u) - 32u;  // sector minus the app-metadata word
inline constexpr std::uint16_t CalNvmEntrySize = 32u;
inline constexpr std::uint16_t CalNvmMagic     = 0xABCDu;

// Our key. The bootloader reserves everything below 0x1000 for itself
// (0x0001 node id, 0x0002 CAN bitrate, 0x0003 flash write count) and documents
// that app keys start at 0x1000.
//
// ONE key holds the WHOLE calibration, not one per sensor. A single 32-byte
// entry is one flash word, so the entire record commits atomically -- there is
// no window in which the APPS half is from a new session and the brake half is
// from an old one. A torn write fails the magic check and the previous entry
// still wins.
inline constexpr std::uint16_t CalNvmKey = 0x1000u;

// Record body: version, reserved, then the eight values big-endian, in the same
// order as the 0x7E4 / 0x7E5 wire frames. 18 bytes, inside the 20-byte limit.
inline constexpr std::uint8_t CalRecordVersion = 1u;
inline constexpr std::size_t  CalRecordLen     = 18u;

enum class CalLoad : std::uint8_t {
    Defaults   = 0,  // nothing stored -- compile-time defaults, the normal first-boot case
    Loaded     = 1,  // a stored record was found and passed validation
    Invalid    = 2,  // found, but validate_cal() rejected it -> defaults
    BadVersion = 3,  // found, but a record version this build does not understand -> defaults
};

struct CalLoadResult {
    PedalCal       cal{};                     // always usable: defaults unless status == Loaded
    CalLoad        status = CalLoad::Defaults;
    std::uint8_t   flags  = 0;                // cal_flag::* when status == Invalid
};

// Scan a memory-mapped NVM region for the newest valid calibration record.
//
// Pure apart from reading through the pointer, so the SIL suite drives it
// against a synthetic sector in RAM. Never returns an unusable calibration: on
// absent, torn, unknown-version or invalid records it returns the compile-time
// defaults and says which happened, because refusing to boot over a bad
// calibration would be worse than running the values the car shipped with.
[[nodiscard]] CalLoadResult load_cal_from_nvm(const void* nvm_base,
                                              std::uint32_t nvm_size) noexcept;

// Serialise a calibration into the record body. Shared with the future write
// path so the two can never disagree about the layout.
void encode_cal_record(const PedalCal& c, std::uint8_t out[CalRecordLen]) noexcept;

// CRC-32 over the canonical record serialisation above.
//
// Carried in the guard field of a COMMIT so the ECU can prove the client is
// committing the set it believes it staged -- a client that has drifted out of
// sync cannot commit values it does not hold, and the guard and the
// consistency check end up being the same field.
//
// CRC-32/ISO-HDLC: reflected, polynomial 0xEDB88320, init 0xFFFFFFFF, final
// XOR 0xFFFFFFFF. Stated explicitly because the pit tool has to reproduce it
// byte for byte; it is the same variant can-flasher already uses for firmware
// images, so its existing helper works unmodified.
[[nodiscard]] std::uint32_t cal_crc32(const PedalCal& c) noexcept;

}  // namespace ecu

#endif  // PEDAL_CAL_NVM_HPP_
