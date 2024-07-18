#include "pico/time.h"
#include <math.h>
#include "controller/motor_controller.h"
#include "controller/controller.h"
#include "encoder/encoder.h"


MotorController::MotorController(Controller &_controller)
    : pid(0.1, 0, 0, 1900, 0), controller(_controller) {
    sin_test.set_ampliture(16384);
    sin_test.set_period(30);
    sin_test.set_active(true);
}

void MotorController::process() {
    uint32_t time = time_us_32();
    if (!active) {
        controller.set_motor_pwm(0.);
        return;
    }
    float cur_target = sin_test.is_active() ? sin_test.get_value() : target_speed;
    float enc_speed = encoder.get_speed(2);
    float pwm = pid.compute(enc_speed, cur_target);

    controller.set_motor_pwm(pwm);
    if (debug_level == 1) {
        printf(
            "%f %f %f %f %f %f %f %f %f\n",
            cur_target,
            enc_speed,
            pwm,
            pid.p_value,
            pid.int_value,
            pid.d_value,
            pid.max_int,
            pid.feed_forward,
            pid.backlash);
    }
}
