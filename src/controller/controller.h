#ifndef _CONTROLLER_H_
#define _CONTROLLER_H_
#include "FreeRTOS.h"
#include "pico/time.h"
#include "utils/simple_pid.h"
#include <math.h>
#include <stdint.h>
#include <task.h>

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

    int period_ms = 10;
    bool active = false;
    float target_heading = 0.f;
    SimplePID heading_pid;
    int debug_level = 0;

    class SinTets {
        uint32_t start_time = 0;
        bool active = false;
        float period = 10;
        float amplitude = 90.;
        float value = 0.;

     public:
        void set_active(bool _active) {
            if (_active != active && _active) {
                start_time = time_us_32();
            }
            active = _active;
        }
        bool is_active() const { return active; }

        void set_period(float _period) { period = _period; }
        float get_period() const { return period; }

        void set_ampliture(float _amplitude) { amplitude = _amplitude; }
        float get_ampliture() const { return amplitude; }

        float get_value() const {
            if (!active) {
                return 0.;
            }
            return sin((time_us_32() - start_time) / 1e6 / period * M_PI) *
                   amplitude;
        }
    } sin_test;

    void process();
    static void thread_handler(void *);
    void set_left_pwm(float pwm);
    void set_right_pwm(float pwm);

 public:
    Controller() : heading_pid(380, 10, 960, 1900) {}

    bool init(uint int1, uint int2, uint int3, uint int4, uint en1, uint en2,
              int task_prio, int stack_size);
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
};

extern Controller controller;
#endif //_CONTROLLER_H_