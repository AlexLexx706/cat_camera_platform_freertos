#include "flash_store.h"
#include <string.h>
#include <stdio.h>
#include <hardware/sync.h>
#include <assert.h>

namespace FlashStore {
    void erase() {
        uint32_t ints = save_and_disable_interrupts();
        flash_range_erase(target_offset, FLASH_SECTOR_SIZE);
        restore_interrupts (ints);
    }

    void save(const uint8_t * data, int size) {
        assert(data);
        assert(size);
        assert((size % FLASH_PAGE_SIZE) != 0);
        uint32_t ints = save_and_disable_interrupts();
        flash_range_program(target_offset, data, size);
        restore_interrupts (ints);
    }

    void read(uint8_t* data, int size) {
        assert(data);
        assert(size);
        assert(size <= FLASH_SECTOR_SIZE);
        memcpy(data, (void *)(XIP_BASE + target_offset), size);
    }
};
