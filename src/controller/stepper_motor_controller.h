#ifndef _STEPPER_MOTOR_CONTROLLER_H_
#define _STEPPER_MOTOR_CONTROLLER_H_
#include "AccelStepper.h"
#include "FreeRTOS.h"
#include "utils/sin_test.h"
#include <stdint.h>
#include <task.h>

class StepperMotorController {
    TaskHandle_t task;
    int debug_level = 1;
    uint dir_pin = 0;
    uint step_pin = 0;
    uint enable_pin = 0;
    uint start_btn_pin = 0;
    uint end_btn_pin = 0;
    AccelStepper *stepper = nullptr;
    int state = 0;
    long max_steps = 0;
    float init_speed = 100;
    SinTets sin_test;
    bool _enable_output = true;
    static void thread_handler(void *);

    struct ExtControl
    {
        long pos = 10;
        float max_speed = 400;
        float acceleration = 400;
    } ext_control;

 public:
    bool init(uint _dir_pin, uint _step_pin, uint _enable_pin,
              uint _start_btn_pin, uint _end_btn_pin, int task_prio,
              int stack_size);
    void process();
    SinTets &get_sin_test() { return sin_test; }

    void move_to(long absolute);
    void set_max_speed(float speed);
    float get_max_speed() const;
    void set_acceleration(float acceleration);
    float get_acceleration() const;
    float get_speed() const;
    long get_distance_to_go();
    long get_target_position();
    long get_current_position();
    void enable_output(bool enable);
    bool is_output_enabled() const;
    bool is_running() const;

    void set_debug_level(int _debug_level) { debug_level = _debug_level; }
    int get_debug_level() const { return debug_level; }
};

extern StepperMotorController stepper_motor_controller;
#endif //_STEPPER_MOTOR_CONTROLLER_H_