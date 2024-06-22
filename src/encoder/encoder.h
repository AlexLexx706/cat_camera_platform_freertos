#ifndef _ENCODER_H_
#define _ENCODER_H_
#include "hardware/gpio.h"
#include <stdint.h>

class Encoder {
    struct EncoderState {
        int8_t last_encoded;
        int32_t encoder_value;
    };
    EncoderState enc_1 = {};
    EncoderState enc_2 = {};

    int a1_pin;
    int b1_pin;
    int a2_pin;
    int b2_pin;
    uint32_t last_print_time = 0;
    uint32_t print_period = 100000;
    int debug_level = 1;
 public:
    void init(int a1_pin, int b1_pin, int a2_pin, int b2_pin,
              gpio_irq_callback_t gpio_irq_callback);
    void process(uint32_t encoder_time, bool is_enc1);

    int32_t get_val_1() const { return enc_1.encoder_value; }
    int32_t get_val_2() const { return enc_2.encoder_value; }
};

extern Encoder encoder;

#endif //_ENCODER_H_