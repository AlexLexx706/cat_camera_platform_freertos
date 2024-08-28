#ifndef _ENCODER_H_
#define _ENCODER_H_
#include "FreeRTOS.h"
#include "hardware/gpio.h"
#include <stdint.h>
#include "task.h"

/*
 * /encoder/0/row                       (set/print)
 * /encoder/0/scale                     (set/print)
 * /encoder/0/value                     (print)
 *
 * /encoder/1/row                       (set/print)
 * /encoder/1/scale                     (set/print)
 * /encoder/1/value                     (set/print)
 * /encoder/debug_level                 (set/print)
 *
 *  1:  enc_0_row
 *      enc_1_row
 *      enc_2_row
 *
 *  2:  enc_2_row
 *      enc_2_speed
 *
 * /encoder/state                       (print)
 * /encoder/save                        (set)
 */

class Encoder {
    struct EncoderState {
        int8_t last_encoded;
        int32_t row = 0;
        float scale = 1.f; // 0.00011860754739359914;
        float speed = 0;
        int32_t prev_row = 0;
    } enc[3];

    int pins[3][2];
    int debug_level = 0;
    int speed_update_period_ms = 40;
    uint32_t prev_time = 0;

    TaskHandle_t speed_task;
    static void speed_task_handler(void *_encoder);
    void process_speed();

 public:
    bool init(int *pins, int task_prio,
              int stack_size);

    void irq_handler(uint gpio);

    int32_t get_row(int num) const { return enc[num].row; }
    void set_row(int num, int32_t val) { enc[num].row = val; }

    float get_scale(int num) const { return enc[num].scale; }
    void set_scale(int num, float _scale) { enc[num].scale = _scale; }

    void set_value(int num, float value) {
        enc[num].row = int32_t(value / enc[num].scale);
    }
    float get_value(int num) const { return enc[num].row * enc[num].scale; }
    float get_speed(int num) const { return enc[num].speed; }

    void set_debug_level(int level) { debug_level = level; }
    int get_debug_level() const { return debug_level; }
};

extern Encoder encoder;

#endif //_ENCODER_H_