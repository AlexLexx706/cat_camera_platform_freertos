#include "controller/stepper_motor_controller.h"
#include "hardware/gpio.h"
#include "pico/time.h"
#include <math.h>
#include <stdio.h>

#define HIGH 1
#define LOW 0

bool StepperMotorController::init(uint _dir_pin, uint _step_pin,
                                  uint _enable_pin, uint _start_btn_pin,
                                  uint _end_btn_pin, int task_prio,
                                  int stack_size) {

    if (xTaskCreate(thread_handler, "sm_controller", stack_size, this,
                    task_prio, &task) != pdPASS) {
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

    stepper = new AccelStepper(AccelStepper::DRIVER, _step_pin, _dir_pin);
    stepper->setPinsInverted(false, false, true);
    stepper->setEnablePin(_enable_pin);

    set_active(active);
    return true;
}

void StepperMotorController::thread_handler(void *val) {
    reinterpret_cast<StepperMotorController *>(val)->process();
}

#define SPEED_1 10000

void StepperMotorController::process() {
    int prev_res = 0;
    int prev_active = false;
    while (1) {
        uint32_t time = time_us_32();

        bool start_btn = gpio_get(start_btn_pin);
        bool end_btn = gpio_get(end_btn_pin);
        int res = int(start_btn) | (int(end_btn) << 1);

        if (prev_res != res) {
            printf("1. sb:%d eb:%d\n", start_btn, end_btn);
        }

        prev_res = res;
        bool cur_active = active;

        if (prev_active != cur_active) {
            if (cur_active) {
                printf("2. Step Active\n");
                stepper->enableOutputs();
            } else {
                printf("2. Step not Active\n");
                stepper->disableOutputs();
            }
        }
        prev_active = cur_active;

        if (!cur_active)
            sleep_ms(100);
        continue;
        // going to begin until not reach button
        if (state == 0) {
            // stop moving and set zero pos
            if (!start_btn) {
                printf("3. set zero pos\n");
                stepper->setCurrentPosition(0);
                state = 1;
                // move to the begin
            } else {
                stepper->setMaxSpeed(init_speed);
                stepper->setSpeed(-init_speed);
                stepper->runSpeed();
            }
            // waiting for reach end button
        } else if (state == 1) {
            // reach end pos, remember max position
            if (!end_btn) {
                max_steps = stepper->currentPosition();
                printf("3. remember max pos:%d\n", max_steps);
                // move to the center
                stepper->setMaxSpeed(400);
                stepper->setAcceleration(400);
                stepper->moveTo(max_steps / 2);
                printf("3. move center\n");
                stepper->runToPosition();
                printf("4. init complete\n");
                // stepper->disableOutputs();
                state = 2;
                start_time = time;
                printf("4. sin test\n");
                // move to the end
            } else {
                stepper->setMaxSpeed(init_speed);
                stepper->setSpeed(init_speed);
                stepper->runSpeed();
            }
        } else if (state == 2) {
            stepper->setMaxSpeed(800);
            float _time = (start_time - time) / 1e6;
            long pos =
                (max_steps / 2) - sin(_time * 2 * M_PI / 15.) * (max_steps / 3);
            stepper->moveTo(pos);
            stepper->run();
        }
    }
}

void StepperMotorController::set_active(bool _active) {
    active = _active;
    gpio_put(enable_pin, active ? LOW : HIGH);
}
