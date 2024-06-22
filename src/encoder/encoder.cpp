#include "encoder.h"
#include <stdio.h>

void Encoder::init(int _a1_pin, int _b1_pin, int _a2_pin, int _b2_pin,
                   gpio_irq_callback_t gpio_irq_callback) {
    a1_pin = _a1_pin;
    b1_pin = _b1_pin;
    a2_pin = _a2_pin;
    b2_pin = _b2_pin;
    // 1. init encoder
    gpio_init(a1_pin);
    gpio_init(a2_pin);
    gpio_init(b1_pin);
    gpio_init(b2_pin);

    gpio_set_dir(a1_pin, GPIO_IN);
    gpio_set_dir(a2_pin, GPIO_IN);
    gpio_set_dir(b1_pin, GPIO_IN);
    gpio_set_dir(b2_pin, GPIO_IN);

    // https://cdn-shop.adafruit.com/product-files/4640/n20+motors_C15011+6V.pdf
    gpio_pull_up(a1_pin);
    gpio_pull_up(a2_pin);
    gpio_pull_up(b1_pin);
    gpio_pull_up(b2_pin);

    gpio_set_irq_enabled_with_callback(a1_pin,
                                       GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
                                       true, gpio_irq_callback);

    gpio_set_irq_enabled_with_callback(b1_pin,
                                       GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
                                       true, gpio_irq_callback);

    gpio_set_irq_enabled_with_callback(a2_pin,
                                       GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
                                       true, gpio_irq_callback);

    gpio_set_irq_enabled_with_callback(b2_pin,
                                       GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
                                       true, gpio_irq_callback);
}

void Encoder::process(uint32_t encoder_time, bool is_enc1) {
    EncoderState *state;
    int encoded;
    if (is_enc1) {
        state = &enc_1;
        encoded = (gpio_get(a1_pin) << 1) | gpio_get(b1_pin);
    } else {
        state = &enc_2;
        encoded = (gpio_get(a2_pin) << 1) | gpio_get(b2_pin);
    }
    int sum = (state->last_encoded << 2) | encoded;

    if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
        state->encoder_value += 1;
    }

    if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
        state->encoder_value += 1;
    }
    state->last_encoded = encoded;

    if (debug_level > 0 && (encoder_time - last_print_time) > print_period) {
        last_print_time = encoder_time;
        printf("e1:%d e2:%d\n", enc_1.encoder_value, enc_2.encoder_value);
    }
}
