#ifndef _CONTROLLER_H_
#define _CONTROLLER_H_
#include "FreeRTOS.h"
#include "utils/simple_pid.h"
#include <stdint.h>
#include <task.h>
#include <stdio.h>
#include "controller/motor_controller.h"
#include "utils/sin_test.h"
#include "utils/speed_profile_genarator.h"

/*
 * /controller/active (set/print)
 * /controller/target_heading (set/print)
 * /controller/heading_pid/p (set/print)
 * /controller/heading_pid/i (set/print)
 * /controller/heading_pid/d (set/print)
 * /controller/heading_pid/max_int (set/print)
 * /controller/debug_level (set/print)
 *
 *  1:  cur_heading
 *      cur_target_heading
 *      heading_pid.p_value
 *      heading_pid.d_value
 *      heading_pid.int_value
 *      heading_control
 *
 *  2:  cur_speed
 *      target_speed
 *      speed_pid.p_value
 *      speed_pid.d_value
 *      speed_pid.int_value
 *      speed_control
 *
 *  3:  cur_heading
 *      cur_target_heading
 *      heading_pid.p_value
 *      heading_pid.d_value
 *      heading_pid.int_value
 *      heading_control
 *      cur_speed
 *      target_speed
 *      speed_pid.p_value
 *      speed_pid.d_value
 *      speed_pid.int_value
 *      speed_control
 *      time_us_32
 *
 *  4:  cur_heading
 *      cur_target_heading
 *      cur_speed
 *      target_speed
 *
 * /controller/sin_test/active          (set/print)
 * /controller/sin_test/period          (set/print) sec
 * /controller/sin_test/amplitude       (set/print)
 * /controller/sin_test/value           (print)

 * /controller/speed_pid/p              (set/print)
 * /controller/speed_pid/i              (set/print)
 * /controller/speed_pid/d              (set/print)
 * /controller/speed_pid/max_int        (set/print)
 * /controller/target_speed             (set/print)

 * /controller/spg/max_speed            (set/print)
 * /controller/spg/accel                (set/print)
 * /controller/spg/is_ready             (set/print)
 * /controller/spg/use                  (set/print)
 * /controller/spg/target               (set/print)
 * /controller/pos                      (set/print)

 * /controller/yaw_rate_pid/p              (set/print)
 * /controller/yaw_rate_pid/i              (set/print)
 * /controller/yaw_rate_pid/d              (set/print)
 * /controller/yaw_rate_pid/max_int        (set/print)
 * /controller/yaw_rate_pid/feed_forward   (set/print)
 * /controller/target_yaw_rate             (set/print)


 * /controller/yaw_spg/max_speed             (set/print)
 * /controller/yaw_spg/accel                 (set/print)
 * /controller/yaw_spg/is_ready              (print)
 * /controller/yaw_spg/target                (print)
 * /controller/yaw_spg/use                   (set/print)
 */
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
    bool active = true;
    float target_heading = 0.f;
    SimplePID heading_pid;
    int debug_level = 0;

    float target_speed = 0.;
    SimplePID speed_pid;
    bool use_pos_control = true;
    SpeedProfileGenerator speed_profile_generator;
    float cur_pos = 0.;

    SinTets sin_test;
    MotorController motor_controller;
    float target_yaw_rate = 0.;
    SimplePID yaw_rate_pid;
    uint32_t last_debug_time_us = 0;
    uint32_t debug_period_us = 50000;

    SpeedProfileGenerator yaw_speed_profile_generator;
    bool use_yaw_pos_control = true;
    uint32_t last_time = 0;

private:
    friend MotorController;
    void process();
    static void thread_handler(void *);
    void set_left_pwm(float pwm);
    void set_right_pwm(float pwm);
    void set_motor_pwm(float pwm);

 public:
    Controller():
            heading_pid(380, 10, 960, 1900, 0),
            speed_pid(0, 600, 0, 5000, 15600),
            motor_controller(*this),
            yaw_rate_pid(20, 0.5, 20, 1000, 10) {
        yaw_speed_profile_generator.set_accel(90);
        yaw_speed_profile_generator.set_max_speed(90);

        speed_profile_generator.set_accel(0.1);
        speed_profile_generator.set_max_speed(1.);
    }

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

    void set_target_yaw_rate(float rate) {target_yaw_rate = rate;}

    float get_target_yaw_rate() const {return target_yaw_rate;}

    SimplePID & get_yaw_rate_pid() {return yaw_rate_pid; }

    SpeedProfileGenerator & get_yaw_speed_profile_generator() {return yaw_speed_profile_generator;}

    bool is_use_yaw_pos_control() { return use_yaw_pos_control;}
    void set_use_yaw_pos_control(bool use) {use_yaw_pos_control = use;}

    SpeedProfileGenerator & get_speed_profile_generator() {return speed_profile_generator;}
    void set_use_pos_control(bool use) {use_pos_control = use;}
    bool is_use_pos_control() const {return use_pos_control;}
    void set_cur_pos(float pos) {cur_pos = pos;}
    float get_cur_pos() {return cur_pos;}

};

extern Controller controller;
#endif //_CONTROLLER_H_