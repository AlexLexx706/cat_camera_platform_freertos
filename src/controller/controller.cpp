#include "controller.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "imu_processor/imu_porcessor.h"
#include "pico/time.h"
#include <math.h>

bool Controller::init(
        uint _int1, uint _int2,
        uint _int3, uint _int4,
        uint _en1,  uint _en2,
        uint _int5, uint _int6,
        int task_prio, int stack_size) {
    if (xTaskCreate(thread_handler, "controller", stack_size, this, task_prio,
                    &task) != pdPASS) {
        printf("Controller::init error: can't create task\n");
        return false;
    }
    int1 = _int1;
    int2 = _int2;
    int3 = _int3;
    int4 = _int4;
    int5 = _int5;
    int6 = _int6;
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

    gpio_set_function(int5, GPIO_FUNC_PWM);
    gpio_set_function(int6, GPIO_FUNC_PWM);

    uint slice_num = pwm_gpio_to_slice_num(en1);
    pwm_set_wrap(slice_num, max_pwm_value);
    pwm_set_enabled(slice_num, true);

    slice_num = pwm_gpio_to_slice_num(int5);
    pwm_set_wrap(slice_num, max_pwm_value);
    pwm_set_enabled(slice_num, true);
    return true;
}

void Controller::process() {
    uint32_t time;
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(period_ms);

    float period = 20.f;
    uint32_t cur_time_ms;
    float control_value;
    uint16_t pwm_value;

    xLastWakeTime = xTaskGetTickCount();

    for (;;) {
        motor_controller.process();

        if (active) {
            float cur_heading = imu_processor.get_angles().x2;
            float cur_target_heading = sin_test.is_active() ? sin_test.get_value() : target_heading;

            float heading_control = heading_pid.compute(cur_heading, cur_target_heading);

            float cur_speed = imu_processor.get_speed();
            float speed_control = -speed_pid.compute(cur_speed, target_speed);

            set_left_pwm(speed_control + heading_control);
            set_right_pwm(speed_control - heading_control);

            if (debug_level == 1) {
                printf("%f %f %f %f %f %f\n",
                       cur_heading,
                       cur_target_heading,
                       heading_pid.p_value,
                       heading_pid.d_value,
                       heading_pid.int_value,
                       heading_control);
            } else if (debug_level == 2) {
                printf(
                    "%f %f %f %f %f %f\n",
                    cur_speed,
                    target_speed,
                    speed_pid.p_value,
                    speed_pid.d_value,
                    speed_pid.int_value,
                    speed_control);
            } else if (debug_level == 3) {
                printf("%f %f %f %f %f %f %f %f %f %f %f %f %u\n",
                    cur_heading,
                    cur_target_heading,
                    heading_pid.p_value,
                    heading_pid.d_value,
                    heading_pid.int_value,
                    heading_control,
                    cur_speed,
                    target_speed,
                    speed_pid.p_value,
                    speed_pid.d_value,
                    speed_pid.int_value,
                    speed_control,
                    time_us_32());
            }

        } else {
            set_left_pwm(0);
            set_right_pwm(0);
        }
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void Controller::set_left_pwm(float pwm) {
    if (pwm >= 0) {
        gpio_put(int1, 1);
        gpio_put(int2, 0);
    } else {
        pwm = -pwm;
        gpio_put(int1, 0);
        gpio_put(int2, 1);
    }
    // convert control value (0-1) to pwm_value
    if (pwm > max_pwm_value) {
        pwm = max_pwm_value;
    }
    if (pwm > 1) {
        pwm += min_pwm_value;
    }
    pwm_set_gpio_level(en1, pwm);
}

void Controller::set_right_pwm(float pwm) {
    if (pwm >= 0) {
        gpio_put(int3, 1);
        gpio_put(int4, 0);
    } else {
        pwm = -pwm;
        gpio_put(int3, 0);
        gpio_put(int4, 1);
    }
    // convert control value (0-1) to pwm_value
    if (pwm > max_pwm_value) {
        pwm = max_pwm_value;
    }
    if (pwm > 1) {
        pwm += min_pwm_value;
    }
    pwm_set_gpio_level(en2, pwm);
}


void Controller::set_motor_pwm(float pwm) {
    if (pwm > 0.f) {
        if (pwm > max_pwm_value) {
            pwm = max_pwm_value;
        }
        pwm_set_gpio_level(int5, 0);
        pwm_set_gpio_level(int6, pwm);
    } else if (pwm < 0.f) {
        pwm = -pwm;
        if (pwm > max_pwm_value) {
            pwm = max_pwm_value;
        }
        pwm_set_gpio_level(int5, pwm);
        pwm_set_gpio_level(int6, 0);
    }
}


void Controller::thread_handler(void *val) {
    reinterpret_cast<Controller *>(val)->process();
}