#include <math.h>
#include "pico/time.h"
#include "controller/stepper_motor_controller.h"
#include <stdio.h>
#include "hardware/gpio.h"

#define HIGH 1
#define LOW 0

bool StepperMotorController::init (
        uint _dir_pin,
        uint _step_pin,
        uint _enable_pin,
        uint _start_btn_pin,
        uint _end_btn_pin,
        int task_prio,
        int stack_size) {

    if (xTaskCreate(thread_handler, "sm_controller", stack_size, this, task_prio,
                    &task) != pdPASS) {
        printf("StepperMotorController::init error: can't create task\n");
        return false;
    }

    dir_pin = _dir_pin;
    step_pin = _step_pin;
    enable_pin = _enable_pin;
    start_btn_pin = _start_btn_pin;
    end_btn_pin = _end_btn_pin;

    gpio_init(start_btn_pin);
    gpio_init(end_btn_pin);

    gpio_set_dir(start_btn_pin, false);
    gpio_set_dir(end_btn_pin, false);

    gpio_pull_up(start_btn_pin);
    gpio_pull_up(end_btn_pin);

    gpio_init(enable_pin);
    gpio_init(step_pin);
    gpio_init(dir_pin);

    gpio_set_dir(enable_pin, GPIO_OUT);
    gpio_set_dir(step_pin, GPIO_OUT);
    gpio_set_dir(dir_pin, GPIO_OUT);

    set_active(active);
    return true;
}

void StepperMotorController::thread_handler(void *val) {
    reinterpret_cast<StepperMotorController *>(val)->process();
}

#define SPEED_1 10000

void StepperMotorController::process() {
    uint32_t time = time_us_32();

    while (1) {
        bool start_btn = gpio_get(start_btn_pin);
        bool end_btn = gpio_get(end_btn_pin);

        if (!active) {
            sleep_ms(100);
            continue;
        }

        bool move = false;
        //move in positive direction
        if (target_speed > 0 ) {
            if (end_btn) {
                gpio_put(dir_pin, HIGH);
                move = true;
            } else if (loop_mode) {
                target_speed = -target_speed;
                gpio_put(dir_pin, LOW);
                move = true;
            }
        //move in negative direction
        } else if (target_speed < 0) {
            if (start_btn) {
                gpio_put(dir_pin, LOW);
                move = true;
            } else if (loop_mode) {
                target_speed = -target_speed;
                gpio_put(dir_pin, HIGH);
                move = true;
            }
        }

        if (move) {
            gpio_put(step_pin, HIGH);
            sleep_us(SPEED_1);
            gpio_put(step_pin, LOW);
            sleep_us(SPEED_1);   
        }
    }
}

void StepperMotorController::set_active(bool _active) {
    active = _active;
    gpio_put(enable_pin, active ? LOW: HIGH);
}
