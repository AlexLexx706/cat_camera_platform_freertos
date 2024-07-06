#include "encoder.h"
#include <stdio.h>
#include "pico/time.h"
#include "utils/settings.h"


bool Encoder::init(
        int *_pins,
        gpio_irq_callback_t gpio_irq_callback,
        int task_prio,
        int stack_size) {

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
    for (int i = 0; i < sizeof(pins)/sizeof(pins[0][0]); i++) {
        reinterpret_cast<int *>(pins)[i] = _pins[i];
        gpio_init(_pins[i]);
        gpio_set_dir(_pins[i], GPIO_IN);
        gpio_pull_up(_pins[i]);
        gpio_set_irq_enabled_with_callback(
            _pins[i], GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, gpio_irq_callback);
    }
    return true;
}

void Encoder::process() {
    EncoderState &state = enc[enc_num];
    int encoded = (gpio_get(pins[enc_num][0]) << 1) | gpio_get(pins[enc_num][1]);
    int sum = (state.last_encoded << 2) | encoded;

    if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
        state.row += 1;
        state.value = state.row * state.scale;
    }

    if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
        state.row -= 1;
        state.value = state.row * state.scale;
    }
    state.last_encoded = encoded;

    if (debug_level > 0 && (time - last_print_time) > print_period) {
        last_print_time = time;
        printf("%d %d %d %f %f %f\n", enc[0].row, enc[1].row, enc[2].row, enc[0].value, enc[1].value, enc[2].value);
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
    if(gpio == pins[0][0] || gpio == pins[0][1]) {
        enc_num = 0;
    } else if(gpio == pins[1][0] || gpio == pins[1][1]) {
        enc_num = 1;
    } else {
        enc_num = 2;
    }
    xSemaphoreGiveFromISR(semaphore, &xHigherPriorityTaskWoken);
}
