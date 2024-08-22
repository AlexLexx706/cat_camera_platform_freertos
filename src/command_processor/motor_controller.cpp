#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "controller/controller.h"
#include "command_processor/command_processor.h"

/*
 * /motor_controller/active                   (set/print)
 * /motor_controller/debug_level,0            (set/print): 1
 *
 * /motor_controller/sin_test/active          (set/print)
 * /motor_controller/sin_test/period          (set/print) sec
 * /motor_controller/sin_test/amplitude       (set/print)
 * /motor_controller/sin_test/value           (print)

 * /motor_controller/speed_pid/p              (set/print)
 * /motor_controller/speed_pid/i              (set/print)
 * /motor_controller/speed_pid/d              (set/print)
 * /motor_controller/speed_pid/max_int        (set/print)
 * /motor_controller/speed_pid/feed_forward   (set/print)
 * /motor_controller/speed_pid/backlash       (set/print)
 *
 * /motor_controller/target_speed             (set/print)
 */
void process_motor_controller(
        const char *prefix, const char *cmd,
        const char *parameter, const char *value) {

    MotorController & motor_controller = controller.get_motor_controller();


    if (strcmp(cmd, "set") == 0) {
        if (value != nullptr) {
            if (strcmp(parameter, "active") == 0) {
                motor_controller.set_active(atoi(value));
                print_re(prefix, "");
            } else if (strcmp(parameter, "debug_level") == 0) {
                motor_controller.set_debug_level(atoi(value));
                print_re(prefix, "");
            } else if (strcmp(parameter, "sin_test/active") == 0) {
                motor_controller.get_sin_test().set_active(atoi(value));
                print_re(prefix, "");
            } else if (strcmp(parameter, "sin_test/period") == 0) {
                motor_controller.get_sin_test().set_period(atof(value));
                print_re(prefix, "");
            } else if (strcmp(parameter, "sin_test/amplitude") == 0) {
                motor_controller.get_sin_test().set_ampliture(atof(value));
                print_re(prefix, "");
            } else if (strcmp(parameter, "speed_pid/p") == 0) {
                motor_controller.get_pid().p = atof(value);
                print_re(prefix, "");
            } else if (strcmp(parameter, "speed_pid/i") == 0) {
                motor_controller.get_pid().i = atof(value);
                print_re(prefix, "");
            } else if (strcmp(parameter, "speed_pid/d") == 0) {
                motor_controller.get_pid().d = atof(value);
                print_re(prefix, "");
            } else if (strcmp(parameter, "speed_pid/max_int") == 0) {
                motor_controller.get_pid().max_int = atof(value);
                print_re(prefix, "");
            } else if (strcmp(parameter, "speed_pid/feed_forward") == 0) {
                motor_controller.get_pid().feed_forward = atof(value);
                print_re(prefix, "");
            } else if (strcmp(parameter, "speed_pid/backlash") == 0) {
                motor_controller.get_pid().backlash = atof(value);
                print_re(prefix, "");
            } else if (strcmp(parameter, "target_speed") == 0) {
                motor_controller.set_target_speed(atof(value));
                print_re(prefix, "");
            } else {
                print_er(prefix, "{6,wrong parameter}");
            }
        } else {
            print_er(prefix, "{7,wrong value}");
        }
    } else if (strcmp(cmd, "print") == 0) {
        if (strcmp(parameter, "active") == 0) {
            char buffer[32];
            snprintf(buffer, sizeof(buffer), "%d", motor_controller.is_active());
            print_re(prefix, buffer);
        } else if (strcmp(parameter, "debug_level") == 0) {
            char buffer[32];
            snprintf(buffer, sizeof(buffer), "%d", motor_controller.get_debug_level());
            print_re(prefix, buffer);
        } else if (strcmp(parameter, "sin_test/active") == 0) {
            char buffer[32];
            snprintf(buffer, sizeof(buffer), "%d", motor_controller.get_sin_test().is_active());
            print_re(prefix, buffer);
        } else if (strcmp(parameter, "sin_test/period") == 0) {
            char buffer[32];
            snprintf(buffer, sizeof(buffer), "%f", motor_controller.get_sin_test().get_period());
            print_re(prefix, buffer);
        } else if (strcmp(parameter, "sin_test/amplitude") == 0) {
            char buffer[32];
            snprintf(buffer, sizeof(buffer), "%f", motor_controller.get_sin_test().get_ampliture());
            print_re(prefix, buffer);
        } else if (strcmp(parameter, "sin_test/value") == 0) {
            char buffer[32];
            snprintf(buffer, sizeof(buffer), "%f", motor_controller.get_sin_test().get_value());
            print_re(prefix, buffer);
        } else if (strcmp(parameter, "speed_pid/p") == 0) {
            char buffer[32];
            snprintf(buffer, sizeof(buffer), "%f", motor_controller.get_pid().p);
            print_re(prefix, buffer);
        } else if (strcmp(parameter, "speed_pid/i") == 0) {
            char buffer[32];
            snprintf(buffer, sizeof(buffer), "%f", motor_controller.get_pid().i);
            print_re(prefix, buffer);
        } else if (strcmp(parameter, "speed_pid/d") == 0) {
            char buffer[32];
            snprintf(buffer, sizeof(buffer), "%f", motor_controller.get_pid().d);
            print_re(prefix, buffer);
        } else if (strcmp(parameter, "speed_pid/max_int") == 0) {
            char buffer[32];
            snprintf(buffer, sizeof(buffer), "%f", motor_controller.get_pid().max_int);
            print_re(prefix, buffer);
        } else if (strcmp(parameter, "speed_pid/feed_forward") == 0) {
            char buffer[32];
            snprintf(buffer, sizeof(buffer), "%f", motor_controller.get_pid().feed_forward);
            print_re(prefix, buffer);
        } else if (strcmp(parameter, "speed_pid/backlash") == 0) {
            char buffer[32];
            snprintf(buffer, sizeof(buffer), "%f", motor_controller.get_pid().backlash);
            print_re(prefix, buffer);
        } else if (strcmp(parameter, "target_speed") == 0) {
            char buffer[32];
            snprintf(buffer, sizeof(buffer), "%f", motor_controller.get_target_speed());
            print_re(prefix, buffer);
        } else {
            print_er(prefix, "{6,wrong parameter}");
        }
    } else {
        print_er(prefix, "{8,wrong command}");
    }
}
