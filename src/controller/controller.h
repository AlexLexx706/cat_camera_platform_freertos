#ifndef _CONTROLLER_H_
#define _CONTROLLER_H_
#include "FreeRTOS.h"
#include "utils/simple_pid.h"
#include <stdint.h>
#include <task.h>

class Controller {
    TaskHandle_t task;
    uint16_t max_pwm_value = 16384;
    uint16_t min_pwm_value = 6000;
    uint16_t max_control_value = max_pwm_value - min_pwm_value;
    uint int1;
    uint int2;
    uint int3;
    uint int4;
    uint en1;
    uint en2;

    int period_ms = 10;
    bool active = false;
    float target_heading = 0.f;
    SimplePID heading_pid;
    int debug_level = 0;

    void process();
    static void thread_handler(void *);
    void set_left_pwm(float pwm);
    void set_right_pwm(float pwm);

 public:
    Controller() : heading_pid(500, 50, 100, 10000) {}

    bool init(uint int1, uint int2, uint int3, uint int4, uint en1, uint en2,
              int task_prio, int stack_size);
    void set_active(bool _active) { active = _active; }
    bool get_aclive() const { return active; }

    void set_target_heading(float heading) { target_heading = heading; }
    float get_target_heading() const { return target_heading; }

    SimplePID &get_heading_pid() { return heading_pid; }

    void set_debug_level(int level) { debug_level = level; }
    int get_debug_level() const { return debug_level; }
};

extern Controller controller;
#endif //_CONTROLLER_H_