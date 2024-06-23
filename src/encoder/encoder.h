#ifndef _ENCODER_H_
#define _ENCODER_H_
#include "FreeRTOS.h"
#include "hardware/gpio.h"
#include "semphr.h"
#include <stdint.h>

class Encoder {
    struct EncoderState {
        int8_t last_encoded;
        int32_t row = 0;
        float scale = 0.00011860754739359914;
        float value = 0.f;
    } enc[2];

    int a1_pin;
    int b1_pin;
    int a2_pin;
    int b2_pin;

    uint32_t last_print_time = 0;
    uint32_t print_period = 100000;
    int debug_level = 1;

    // user in IRQ handler
    SemaphoreHandle_t semaphore;
    TaskHandle_t task;
    uint32_t time = 0;
    bool is_enc1 = false;

    // thread for handring IRQ
    static void thread_handler(void *_encoder);
    void process();

 public:
    bool init(int a1_pin, int b1_pin, int a2_pin, int b2_pin,
              gpio_irq_callback_t gpio_irq_callback, int task_prio,
              int stack_size);
    void irq_handler(uint gpio, BaseType_t &xHigherPriorityTaskWoken);

    int32_t get_row(int num) const { return enc[num].row; }
    void set_row(int num, int32_t val) { enc[num].row = val; }

    float get_scale(int num) const {return enc[num].scale;}
    void set_scale(int num, float _scale) {enc[num].scale = _scale;}

    void set_value(int num, float value) {
        enc[num].row = int32_t(value / enc[num].scale);
        enc[num].value = value;}
    float get_value(int num) const {return enc[num].value;}

    void set_debug_level(int level) {debug_level = level;}
    int get_debug_level() const { return debug_level;}
};

extern Encoder encoder;

#endif //_ENCODER_H_