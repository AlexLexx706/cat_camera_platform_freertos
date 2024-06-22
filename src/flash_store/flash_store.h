#ifndef _FLASH_STORE_H_
#define _FLASH_STORE_H_
#include <hardware/flash.h>
#include <stdint.h>

// Set the target offest to the last sector of flash
// #define FLASH_TARGET_OFFSET (PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE)

namespace FlashStore {
    constexpr auto target_offset = (PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE);
    constexpr auto store_ptr = (XIP_BASE + target_offset);

    // Number of bytes to be erased 4096 bytes (one sector).
    void erase();

    // Number of bytes to program. Must be a multiple of 256 bytes (one page)!!!.
    void save(const uint8_t *data, int size);

    // size <=  FLASH_SECTOR_SIZE
    void read(uint8_t *data, int size);
}; // namespace FlashStore

#endif //_FLASH_STORE_H_