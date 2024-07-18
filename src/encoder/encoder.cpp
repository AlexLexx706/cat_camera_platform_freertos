#include "encoder.h"
#include "pico/time.h"
#include "utils/settings.h"
#include <stdio.h>

bool Encoder::init(int *_pins, int task_prio, int stack_size) {

    for (int i = 0; i < sizeof(enc) / sizeof(enc[0]); i++) {
        // reading of scale from settings
        if (*reinterpret_cast<uint32_t *>(
                &settings->encoder_settings.scale[i]) != 0xffffffff) {
            enc[i].scale = settings->encoder_settings.scale[i];
        }
    }

    if (xTaskCreate(speed_task_handler, "encoder_speed", stack_size, this,
                    task_prio, &speed_task) != pdPASS) {
        printf("Encoder::init error: can't create steed task\n");
        return false;
    }

    for (int i = 0; i < sizeof(pins) / sizeof(pins[0][0]); i++) {
        reinterpret_cast<int *>(pins)[i] = _pins[i];
        gpio_init(_pins[i]);
        gpio_set_dir(_pins[i], GPIO_IN);
        gpio_pull_up(_pins[i]);
        gpio_set_irq_enabled(
            _pins[i],
            GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    }
    return true;
}

void Encoder::speed_task_handler(void *_encoder) {
    Encoder &encoder(*reinterpret_cast<Encoder *>(_encoder));

    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(encoder.speed_update_period_ms);
    xLastWakeTime = xTaskGetTickCount();

    for (;;) {
        encoder.process_speed();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void Encoder::process_speed() {
    uint32_t time = time_us_32();
    uint32_t dt = time - prev_time;
    prev_time = time;

    for (int i = 0; i < sizeof(enc) / sizeof(enc[1]); i++) {
        int32_t row = enc[i].row;
        enc[i].speed = (row - enc[i].prev_row) / float(dt);
        enc[i].prev_row = row;
    }

    if (debug_level == 1) {
        printf("%d %d %d\n", enc[0].row, enc[1].row, enc[2].row);
    } else if (debug_level == 2) {
        printf("%d %f\n", enc[2].row, enc[2].speed);
    }
}

void Encoder::irq_handler(uint gpio) {
    int enc_num;
    if (gpio == pins[0][0] || gpio == pins[0][1]) {
        enc_num = 0;
    } else if (gpio == pins[1][0] || gpio == pins[1][1]) {
        enc_num = 1;
    } else {
        enc_num = 2;
    }
    EncoderState &state = enc[enc_num];
    int encoded = (gpio_get(pins[enc_num][0]) << 1) | gpio_get(pins[enc_num][1]);
    int sum = (state.last_encoded << 2) | encoded;

    if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
        state.row += 1;
    }

    if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
        state.row -= 1;
    }
    state.last_encoded = encoded;
}
