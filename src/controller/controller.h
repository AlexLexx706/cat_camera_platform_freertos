#ifndef _CONTROLLER_H_
#define _CONTROLLER_H_
#include "FreeRTOS.h"
#include "utils/simple_pid.h"
#include <stdint.h>
#include <task.h>
#include <stdio.h>
#include "controller/motor_controller.h"
#include "utils/sin_test.h"

class Controller {
    TaskHandle_t task;
    uint16_t max_pwm_value = 16384;
    uint16_t min_pwm_value = 9000;
    uint16_t max_control_value = max_pwm_value - min_pwm_value;
    uint int1;
    uint int2;
    uint int3;
    uint int4;
    uint en1;
    uint en2;
    uint int5;
    uint int6;

    int period_ms = 10;
    bool active = false;
    float target_heading = 0.f;
    SimplePID heading_pid;
    int debug_level = 0;

    float target_speed = 0.;
    SimplePID speed_pid;

    SinTets sin_test;
    MotorController motor_controller;
private:
    friend MotorController;
    void process();
    static void thread_handler(void *);
    void set_left_pwm(float pwm);
    void set_right_pwm(float pwm);
    void set_motor_pwm(float pwm);

 public:
    Controller()
        : heading_pid(380, 10, 960, 1900, 0), speed_pid(0, 600, 0, 5000, 15600),
          motor_controller(*this) {}

    bool init(uint int1, uint int2, uint int3, uint int4, uint en1, uint en2,
              uint int5, uint int6, int task_prio, int stack_size);
    void set_active(bool _active) { active = _active; }
    bool get_aclive() const { return active; }

    void set_target_heading(float heading) { target_heading = heading; }
    float get_target_heading() const { return target_heading; }

    SimplePID &get_heading_pid() { return heading_pid; }

    void set_debug_level(int level) { debug_level = level; }
    int get_debug_level() const { return debug_level; }

    void set_min_pwm(uint16_t value) { min_pwm_value = value; }
    uint16_t get_min_pwm() const { return min_pwm_value; }

    SinTets &get_sin_test() { return sin_test; }

    SimplePID &get_speed_pid() { return speed_pid; }

    float get_target_speed() const { return target_speed; }
    void set_target_speed(float speed) { target_speed = speed; }
    MotorController & get_motor_controller() {return motor_controller;}
};

extern Controller controller;
#endif //_CONTROLLER_H_