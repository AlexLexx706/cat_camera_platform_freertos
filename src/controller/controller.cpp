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
    uint32_t cur_time_ms;
    float control_value;
    uint16_t pwm_value;

    xLastWakeTime = xTaskGetTickCount();

    for (;;) {
        // motor_controller.process();

        if (active) {
            uint32_t time = time_us_32();
            float dt = (time - last_time) / 1e6;

            float cur_yaw_rate = imu_processor.get_rate().x2;
            float cur_yaw = imu_processor.get_angles().x2;
            float cur_target_yaw_rate;
            float yaw_rate_control = 0;

            // using yaw profile speed control
            if (use_yaw_pos_control) {
                if (last_time != 0) {
                    cur_target_yaw_rate = yaw_speed_profile_generator.evaluate(dt, cur_yaw);

                    if (fabs(cur_target_yaw_rate - cur_yaw_rate) >= 6.) {
                       yaw_rate_control = yaw_rate_pid.compute(cur_yaw_rate, cur_target_yaw_rate);
                    }
                } else {
                    cur_target_yaw_rate = 0.;
                    yaw_rate_control = 0.;
                }
            // yaw rate control only
            } else {
                cur_target_yaw_rate = target_yaw_rate;
                yaw_rate_control = yaw_rate_pid.compute(cur_yaw_rate, cur_target_yaw_rate);
            }

            float cur_speed = imu_processor.get_speed();
            float cur_target_speed = 0;
            float speed_control = 0;

            // using position control with profile of speed
            if (use_pos_control) {
                if (last_time != 0) {
                    cur_pos += cur_speed * dt;
                    cur_target_speed = speed_profile_generator.evaluate(dt, cur_pos);

                    if (fabs(cur_target_speed) > 0.02) {
                        speed_control = speed_pid.compute(cur_speed, cur_target_speed);
                    }
                }
            // using speed control
            } else {
                cur_target_speed = target_speed;
                speed_control = speed_pid.compute(cur_speed, cur_target_speed);
            }

            set_left_pwm(speed_control + yaw_rate_control);
            set_right_pwm(speed_control - yaw_rate_control);
            last_time = time;

            //send debug
            if (debug_level) {
                uint32_t debug_dt = time - last_debug_time_us;

                if (debug_dt >= debug_period_us) {
                    last_debug_time_us = time - (debug_dt - debug_period_us);

                    //send level 1
                    if (debug_level == 1) {
                        printf("%f %f %f %f %f %f %f\n",
                            cur_yaw_rate,
                            cur_target_yaw_rate,
                            yaw_rate_pid.p_value,
                            yaw_rate_pid.int_value,
                            yaw_rate_pid.d_value,
                            yaw_rate_pid.feed_forward_value,
                            yaw_rate_control);
                    //send level 2
                    } else if (debug_level == 2) {
                        printf("%f %f %f %f %f\n",
                            dt,
                            cur_yaw,
                            yaw_speed_profile_generator.get_target_pos(),
                            cur_yaw_rate,
                            cur_target_yaw_rate);
                    } else if (debug_level == 3) {
                        printf(
                            "%f %f %f %f %f %f\n",
                            cur_speed,
                            cur_target_speed,
                            speed_pid.p_value,
                            speed_pid.d_value,
                            speed_pid.int_value,
                            speed_control);
                    }
                }
            }

            // float cur_heading = imu_processor.get_angles().x2;
            // float cur_target_heading = sin_test.is_active() ? sin_test.get_value() : target_heading;

            // float heading_control = heading_pid.compute(cur_heading, cur_target_heading);

            // float cur_speed = imu_processor.get_speed();
            // float speed_control = speed_pid.compute(cur_speed, target_speed);

            // set_left_pwm(speed_control + heading_control);
            // set_right_pwm(speed_control - heading_control);

            // if (debug_level == 1) {
            //     printf("%f %f %f %f %f %f\n",
            //            cur_heading,
            //            cur_target_heading,
            //            heading_pid.p_value,
            //            heading_pid.d_value,
            //            heading_pid.int_value,
            //            heading_control);
            // } else if (debug_level == 2) {
            //     printf(
            //         "%f %f %f %f %f %f\n",
            //         cur_speed,
            //         target_speed,
            //         speed_pid.p_value,
            //         speed_pid.d_value,
            //         speed_pid.int_value,
            //         speed_control);
            // } else if (debug_level == 3) {
            //     printf("%f %f %f %f %f %f %f %f %f %f %f %f %u\n",
            //         cur_heading,
            //         cur_target_heading,
            //         heading_pid.p_value,
            //         heading_pid.d_value,
            //         heading_pid.int_value,
            //         heading_control,
            //         cur_speed,
            //         target_speed,
            //         speed_pid.p_value,
            //         speed_pid.d_value,
            //         speed_pid.int_value,
            //         speed_control,
            //         time_us_32());
            // } else if (debug_level == 4) {
            //     printf("%f %f %f %f\n",
            //         cur_heading,
            //         cur_target_heading,
            //         cur_speed,
            //         target_speed);
            // }
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