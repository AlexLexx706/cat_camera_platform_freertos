#include "encoder.h"
#include <stdio.h>
#include "pico/time.h"


bool Encoder::init(int _a1_pin, int _b1_pin, int _a2_pin, int _b2_pin,
                   gpio_irq_callback_t gpio_irq_callback,
                   int task_prio, int stack_size) {
    printf("Encoder::init task_prio:%d stack_size:%d\n", task_prio, stack_size);

    semaphore = xSemaphoreCreateBinary();
    if (semaphore == NULL) {
        printf("Encoder::init error: can`t create binary semaphore\n");
        return false;
    }

    if (xTaskCreate(thread_handler, "encoder", stack_size, this, task_prio, &task) != pdPASS) {
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
    printf("Encoder::init ok\n");
    return true;
}

void Encoder::process() {
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

    if (debug_level > 0 && (time - last_print_time) > print_period) {
        last_print_time = time;
        printf("e1:%d e2:%d\n", enc_1.encoder_value, enc_2.encoder_value);
    }
}


void Encoder::thread_handler(void * _encoder) {
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
