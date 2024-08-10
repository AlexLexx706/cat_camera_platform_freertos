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
    sin_test.set_active(false);
    sin_test.set_period(20);
    return true;
}

void StepperMotorController::thread_handler(void *val) {
    reinterpret_cast<StepperMotorController *>(val)->process();
}

#define SPEED_1 10000

void StepperMotorController::process() {
    int prev_res = 0;
    int prev_active = false;
    while (true) {
        uint32_t time = time_us_32();

        bool start_btn = gpio_get(start_btn_pin);
        bool end_btn = gpio_get(end_btn_pin);
        int res = int(start_btn) | (int(end_btn) << 1);

        if (debug_level == 1) {
            if (prev_res != res) {
                printf("1. sb:%d eb:%d\n", start_btn, end_btn);
            }
        }
        prev_res = res;

        // going to begin until not reach button
        if (state == 0) {
            // stop moving and set zero pos
            if (!start_btn) {
                if (debug_level == 1) {
                    printf("3. set zero pos\n");
                }
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
                if (debug_level == 1) {
                    printf("3. remember max pos:%d\n", max_steps);
                }
                // move to the center
                stepper->setMaxSpeed(400);
                stepper->setAcceleration(400);
                stepper->moveTo(max_steps / 2);
                ext_control.pos = stepper->targetPosition();
                sin_test.set_max_ampliture(ext_control.pos - 10);
                sin_test.set_ampliture(sin_test.get_max_ampliture());

                if (debug_level == 1) {
                    printf("3. move center\n");
                }
                stepper->runToPosition();
                if (debug_level == 1) {
                    printf("4. init complete\n");
                }
                state = 2;
                // move to the end
            } else {
                stepper->setMaxSpeed(init_speed);
                stepper->setSpeed(init_speed);
                stepper->runSpeed();
            }
            // ready to go
        } else if (state == 2) {
            // start detected
            if (!start_btn) {
                stepper->setCurrentPosition(0);
                stepper->setMaxSpeed(init_speed);
                stepper->setSpeed(init_speed);
                // move until not
                while (!(start_btn = gpio_get(start_btn_pin))) {
                    stepper->runSpeed();
                }
                stepper->setSpeed(0);
            }

            // end detected
            if (!end_btn) {
                stepper->setCurrentPosition(max_steps);
                stepper->setMaxSpeed(-init_speed);
                stepper->setSpeed(-init_speed);
                // move until not
                while (!(end_btn = gpio_get(end_btn_pin))) {
                    stepper->runSpeed();
                }
                stepper->setSpeed(0);
            }

            // sin tets
            if (sin_test.is_active()) {
                float value = sin_test.get_value() + max_steps / 2;
                stepper->moveTo(value);
                // external control
            } else {
                stepper->moveTo(ext_control.pos);
                stepper->setMaxSpeed(ext_control.max_speed);
                stepper->setAcceleration(ext_control.acceleration);
            }
            stepper->run();
        }
    }
}

void StepperMotorController::moveTo(long absolute) {
    assert(stepper);
    if (absolute > max_steps) {
        absolute = max_steps;
    } else if (absolute < 0) {
        absolute = 0;
    }
    ext_control.pos = absolute;
}
void StepperMotorController::set_max_speed(float speed) {
    assert(stepper);
    ext_control.max_speed = speed;
}

float StepperMotorController::get_max_speed() const {
    assert(stepper);
    return stepper->maxSpeed();
}

void StepperMotorController::set_acceleration(float acceleration) {
    assert(stepper);
    ext_control.acceleration = acceleration;
}

float StepperMotorController::get_acceleration() const {
    assert(stepper);
    return stepper->acceleration();
}

float StepperMotorController::get_speed() const {
    assert(stepper);
    return stepper->speed();
}

long StepperMotorController::get_distance_to_go() {
    assert(stepper);
    return stepper->distanceToGo();
}

long StepperMotorController::get_target_position() {
    assert(stepper);
    return stepper->targetPosition();
}

long StepperMotorController::get_current_position() {
    assert(stepper);
    return stepper->currentPosition();
}

void StepperMotorController::enable_output(bool enable) {
    assert(stepper);
    if (enable != _enable_output) {
        _enable_output = enable;
        if (_enable_output) {
            stepper->enableOutputs();
        } else {
            stepper->disableOutputs();
        }
    }
}
bool StepperMotorController::is_output_enabled() const {
    return _enable_output;
}
bool StepperMotorController::is_running() const {
    assert(stepper);
    return stepper->isRunning();
}
