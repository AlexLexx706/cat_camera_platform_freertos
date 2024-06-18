#ifndef _FLASH_STORE_H_
#define  _FLASH_STORE_H_
#include <stdint.h>

//max stored bytes: 4096 bytes
namespace FlashStore {
    //Number of bytes to be erased 4096 bytes (one sector).
    void clear();

    //Number of bytes to program. Must be a multiple of 256 bytes (one page).
    void save(const uint8_t * data, int size);

    //size <  FLASH_SECTOR_SIZE
    void read(uint8_t * data, int size);
};

#endif //_FLASH_STORE_H_