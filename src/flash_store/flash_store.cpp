#include "flash_store.h"
#include <hardware/flash.h>
#include <string.h>
#include <stdio.h>
#include <hardware/sync.h>
// Set the target offest to the last sector of flash
#define FLASH_TARGET_OFFSET (PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE)
// #define FLASH_TARGET_OFFSET (256 * 1024)


namespace FlashStore {
    void clear() {
        printf("clear\n");
        uint32_t ints = save_and_disable_interrupts();
        flash_range_erase(FLASH_TARGET_OFFSET, FLASH_SECTOR_SIZE);
        restore_interrupts (ints);
    }

    void save(const uint8_t * data, int size) {
        size = size - (size % FLASH_PAGE_SIZE);
        printf("save data:%s size:%d\n", data, size);
        uint32_t ints = save_and_disable_interrupts();
        flash_range_program(FLASH_TARGET_OFFSET, data, size);
        restore_interrupts (ints);
    }

    void read(uint8_t* data, int size) {
        if (size > FLASH_SECTOR_SIZE) {
            size = FLASH_SECTOR_SIZE;
        }
        memcpy(data, (void *)(XIP_BASE + FLASH_TARGET_OFFSET), size);
        printf("read data:%s size:%d\n", data, size);
    }
};
