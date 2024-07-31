#ifndef _STEPPER_MOTOR_CONTROLLER_H_
#define _STEPPER_MOTOR_CONTROLLER_H_
#include "FreeRTOS.h"
#include <stdint.h>
#include <task.h>
#include "AccelStepper.h"

class StepperMotorController {
    TaskHandle_t task;
    float target_speed = 1.;
    bool active = true;
    int debug_level = 1;
    uint dir_pin = 0;
    uint step_pin = 0;
    uint enable_pin = 0;
    uint start_btn_pin = 0;
    uint end_btn_pin = 0;
    bool loop_mode = true;
    static void thread_handler(void *);

public:
    bool init (
        uint _dir_pin,
        uint _step_pin,
        uint _enable_pin,
        uint _start_btn_pin,
        uint _end_btn_pin,
        int task_prio,
        int stack_size);

    void process();

    bool is_active() const {return active;}
    void set_active(bool _active);

    float get_target_speed() const {return target_speed;}
    void set_target_speed(float _target_speed) {target_speed = _target_speed;}

    void set_debug_level(int _debug_level) {debug_level = _debug_level;}
    int get_debug_level() const {return debug_level;}
};
#endif //_STEPPER_MOTOR_CONTROLLER_H_