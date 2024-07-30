#ifndef _MOTOR_CONTROLLER_H_
#define _MOTOR_CONTROLLER_H_
#include <stdint.h>
#include <stdio.h>
#include "utils/simple_pid.h"
#include "utils/sin_test.h"

class MotorController {
    SimplePID pid;
    SinTets sin_test;
    class Controller &controller;

    float target_speed = 0.;
    bool active = true;
    int debug_level = 0;
public:
    MotorController(Controller &_controller);

    void process();

    SimplePID & get_pid() {return pid;};
    SinTets & get_sin_test() {return sin_test;};

    bool is_active() const {return active;}
    void set_active(bool _active) {active = _active;}

    float get_target_speed() const {return target_speed;}
    void set_target_speed(float _target_speed) {target_speed = _target_speed;}

    void set_debug_level(int _debug_level) {debug_level = _debug_level;}
    int get_debug_level() const {return debug_level;}
};
#endif //_MOTOR_CONTROLLER_H_