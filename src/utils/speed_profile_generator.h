#ifndef __SPEED_PROFILE_GENERATOR_H_
#define __SPEED_PROFILE_GENERATOR_H_

#include <math.h>
#include <stdio.h>

class SpeedProfileGenerator {
    enum {
        ACCEL,
        CRUISE,
        DECEL,
        TARGET
    } state = ACCEL;

    float max_speed = 1.;
    float accel = 1.;
    float decel_distance = 0.5;
    float speed = 0.;
    float speed_sign = 1.;
    float last_speed = 0.;
    float target_pos = 0.;
    float current_pos = 0.;

    float next_target_pos = 0.;
    float next_speed_sign = 0.;

public:
    void set_max_speed(float _speed) {
        if (max_speed == _speed || _speed < 0.) {
            return;
        }
        max_speed = _speed;
        decel_distance = 0.5 * max_speed * max_speed / accel;
    }

    float get_max_speed() const {return max_speed;}

    void set_accel(float _accel) {
        if (_accel == accel || _accel < 0.) {
            return;
        }
        accel = _accel;
        decel_distance = 0.5 * max_speed * max_speed / accel;
    }

    float get_accel() const {return accel;}

    float get_speed() const {return last_speed;}

    void set_target_pos(float pos) {
        float cur_speed_sign = pos - current_pos >= 0. ? 1. : -1;

        // try soft interrupt movement
        if (speed_sign != cur_speed_sign) {
            float cur_target_pos = current_pos + (0.5 * speed * speed / accel) * speed_sign;
            target_pos = cur_target_pos;
            next_target_pos = pos;
            next_speed_sign = cur_speed_sign;
            state = DECEL;
        // continue to accelerate
        } else {
            state = ACCEL;
            target_pos = pos;
            next_target_pos = pos;
            speed_sign = cur_speed_sign;
            next_speed_sign = speed_sign;
        }
    }
    float get_target_pos() const {return target_pos;}

    bool is_ready() {return state == TARGET;}

    float evaluate(float dt, float _current_pos) {
        current_pos = _current_pos;
        float distance = (target_pos - current_pos) * speed_sign;

        if (distance < 0.) {
            speed = 0.;
            //continue to move next pos
            if (target_pos != next_target_pos) {
                target_pos = next_target_pos;
                speed_sign = next_speed_sign;
                distance = (target_pos - current_pos) * speed_sign;
                state = ACCEL;
            // stop
            } else {
                state = TARGET;
            }
        }
        if (state == ACCEL) {
            speed = speed + accel * dt;

            if (speed >= max_speed) {
                speed = max_speed;
                state = CRUISE;
            }
            if (distance <= decel_distance) {
                float v_exp = sqrt(2 * accel * distance);
                if (v_exp < speed) {
                    state = DECEL;
                    speed = v_exp;
                }
            }
        } else if (state == CRUISE) {
            if (distance <= decel_distance) {
                state = DECEL;
            }
        } else if (state == DECEL) {
            speed = sqrt(2 * accel * distance);
        }
        last_speed = speed * speed_sign;
        return last_speed;
    }
};
#endif 