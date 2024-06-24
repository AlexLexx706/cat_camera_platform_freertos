#include "controller.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "pico/time.h"
#include <math.h>
#include <stdio.h>

bool Controller::init(uint _int1, uint _int2, uint _int3, uint _int4, uint _en1,
                      uint _en2, int task_prio, int stack_size) {
    if (xTaskCreate(thread_handler, "controller", stack_size, this, task_prio,
                    &task) != pdPASS) {
        printf("Encoder::init error: can't create task\n");
        return false;
    }
    int1 = _int1;
    int2 = _int2;
    int3 = _int3;
    int4 = _int4;
    en1 = _en1;
    en2 = _en2;

    gpio_init(int1);
    gpio_init(int2);
    gpio_init(int3);
    gpio_init(int4);

    gpio_set_dir(int1, GPIO_OUT);
    gpio_set_dir(int2, GPIO_OUT);
    gpio_set_dir(int3, GPIO_OUT);
    gpio_set_dir(int4, GPIO_OUT);

    gpio_set_function(en1, GPIO_FUNC_PWM);
    gpio_set_function(en2, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(en1);
    pwm_set_wrap(slice_num, max_pwm_value);
    pwm_set_enabled(slice_num, true);

    return true;
}

void Controller::process() {
    uint32_t time;
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(10);

    float period = 10.f;
    uint32_t cur_time_ms;
    float control_value;
    uint16_t pwm_value;

    xLastWakeTime = xTaskGetTickCount();

    for (;;) {
        cur_time_ms = to_ms_since_boot(get_absolute_time());
        control_value = (sinf(cur_time_ms / 1000.f / period * M_PI));
        float tmp_control_value = control_value;

        if (control_value >= 0) {
            gpio_put(int1, 1);
            gpio_put(int2, 0);
            gpio_put(int3, 1);
            gpio_put(int4, 0);
        } else {
            tmp_control_value = -control_value;
            gpio_put(int1, 0);
            gpio_put(int2, 1);
            gpio_put(int3, 0);
            gpio_put(int4, 1);
        }
        // convert control value (0-1) to pwm_value
        pwm_value = tmp_control_value * max_control_value + min_pwm_value;
        pwm_set_gpio_level(en1, pwm_value);
        pwm_set_gpio_level(en2, pwm_value);

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void Controller::thread_handler(void *val) {
    reinterpret_cast<Controller *>(val)->process();
}