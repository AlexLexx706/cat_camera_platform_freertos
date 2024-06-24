#include "encoder.h"
#include <stdio.h>
#include "pico/time.h"
#include "utils/settings.h"


bool Encoder::init(
        int _a1_pin, int _b1_pin, int _a2_pin, int _b2_pin,
        gpio_irq_callback_t gpio_irq_callback,
        int task_prio, int stack_size) {

    for (int i = 0; i < sizeof(enc)/sizeof(enc[0]); i++) {
        //reading of scale from settings
        if (*reinterpret_cast<uint32_t*>(&settings->encoder_settings.scale[i]) != 0xffffffff) {
            enc[i].scale = settings->encoder_settings.scale[i];
        }
    }
    semaphore = xSemaphoreCreateBinary();
    if (semaphore == NULL) {
        printf("Encoder::init error: can`t create binary semaphore\n");
        return false;
    }

    if (xTaskCreate(task_handler, "encoder", stack_size, this, task_prio, &task) != pdPASS) {
        printf("Encoder::init error: can't create task\n");
        return false;
    }

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

    gpio_set_irq_enabled_with_callback(
        a1_pin, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, gpio_irq_callback);

    gpio_set_irq_enabled_with_callback(
        b1_pin, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, gpio_irq_callback);

    gpio_set_irq_enabled_with_callback(
        a2_pin, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, gpio_irq_callback);

    gpio_set_irq_enabled_with_callback(
        b2_pin, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, gpio_irq_callback);
    return true;
}

void Encoder::process() {
    EncoderState *state;
    int encoded;
    if (is_enc1) {
        state = enc;
        encoded = (gpio_get(a1_pin) << 1) | gpio_get(b1_pin);
    } else {
        state = &enc[1];
        encoded = (gpio_get(a2_pin) << 1) | gpio_get(b2_pin);
    }
    int sum = (state->last_encoded << 2) | encoded;

    if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
        state->row += 1;
        state->value = state->row * state->scale;
    }

    if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
        state->row -= 1;
        state->value = state->row * state->scale;
    }
    state->last_encoded = encoded;

    if (debug_level > 0 && (time - last_print_time) > print_period) {
        last_print_time = time;
        printf("%d %d %f %f\n", enc[0].row, enc[1].row, enc[0].value, enc[1].value);
    }
}


void Encoder::task_handler(void * _encoder) {
    Encoder & encoder(*reinterpret_cast<Encoder *>(_encoder));
    for (;;) {
        xSemaphoreTake(encoder.semaphore, portMAX_DELAY);
        encoder.process();
    }
}

void Encoder::irq_handler(uint gpio, BaseType_t & xHigherPriorityTaskWoken) {
    time = time_us_32();
    is_enc1 = (gpio == a1_pin || gpio == b1_pin);
    xSemaphoreGiveFromISR(semaphore, &xHigherPriorityTaskWoken);
}
