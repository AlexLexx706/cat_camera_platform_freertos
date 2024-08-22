#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "controller/controller.h"
#include "command_processor/command_processor.h"
#include "controller/stepper_motor_controller.h"

/*
 * /stepper_motor_controller/move (set)
 * /stepper_motor_controller/max_speed (set/print)
 * /stepper_motor_controller/acceleration (set/print)
 * /stepper_motor_controller/speed (print)
 * /stepper_motor_controller/distance_to_go (print)
 * /stepper_motor_controller/target_position (print)
 * /stepper_motor_controller/current_position (print)
 * /stepper_motor_controller/output (set/print)
 * /stepper_motor_controller/running (print)
 * /stepper_motor_controller/debug_level (set/print)
 */


void process_stepper_motor_controller(
        const char *prefix, const char *cmd,
        const char *parameter, const char *value) {
    if (strcmp(cmd, "set") == 0) {
        if (value != nullptr) {
            if (strcmp(parameter, "move") == 0) {
                stepper_motor_controller.move_to(atoi(value));
                print_re(prefix, "");
            } else if (strcmp(parameter, "max_speed") == 0) {
                stepper_motor_controller.set_max_speed(atof(value));
                print_re(prefix, "");
            } else if (strcmp(parameter, "acceleration") == 0) {
                stepper_motor_controller.set_acceleration(atof(value));
                print_re(prefix, "");
            } else if (strcmp(parameter, "output") == 0) {
                stepper_motor_controller.enable_output(bool(atoi(value)));
                print_re(prefix, "");
            } else if (strcmp(parameter, "debug_level") == 0) {
                stepper_motor_controller.set_debug_level(atoi(value));
                print_re(prefix, "");
            } else {
                print_er(prefix, "{6,wrong parameter}");
            }
        } else {
            print_er(prefix, "{7,wrong value}");
        }
    } else if (strcmp(cmd, "print") == 0) {
        if (strcmp(parameter, "max_speed") == 0) {
            char buffer[32];
            snprintf(buffer, sizeof(buffer), "%f", stepper_motor_controller.get_max_speed());
            print_re(prefix, buffer);
        } else if (strcmp(parameter, "acceleration") == 0) {
            char buffer[32];
            snprintf(buffer, sizeof(buffer), "%f", stepper_motor_controller.get_acceleration());
            print_re(prefix, buffer);
        } else if (strcmp(parameter, "speed") == 0) {
            char buffer[32];
            snprintf(buffer, sizeof(buffer), "%f", stepper_motor_controller.get_speed());
            print_re(prefix, buffer);
        } else if (strcmp(parameter, "distance_to_go") == 0) {
            char buffer[32];
            snprintf(buffer, sizeof(buffer), "%d", stepper_motor_controller.get_distance_to_go());
            print_re(prefix, buffer);
        } else if (strcmp(parameter, "target_position") == 0) {
            char buffer[32];
            snprintf(buffer, sizeof(buffer), "%d", stepper_motor_controller.get_target_position());
            print_re(prefix, buffer);
        } else if (strcmp(parameter, "current_position") == 0) {
            char buffer[32];
            snprintf(buffer, sizeof(buffer), "%d", stepper_motor_controller.get_current_position());
            print_re(prefix, buffer);
        } else if (strcmp(parameter, "output") == 0) {
            char buffer[32];
            snprintf(buffer, sizeof(buffer), "%d", stepper_motor_controller.is_output_enabled());
            print_re(prefix, buffer);
        } else if (strcmp(parameter, "running") == 0) {
            char buffer[32];
            snprintf(buffer, sizeof(buffer), "%d", stepper_motor_controller.is_running());
            print_re(prefix, buffer);
        } else if (strcmp(parameter, "debug_level") == 0) {
            char buffer[32];
            snprintf(buffer, sizeof(buffer), "%d", stepper_motor_controller.get_debug_level());
            print_re(prefix, buffer);
        } else {
            print_er(prefix, "{6,wrong parameter}");
        }
    } else {
        print_er(prefix, "{8,wrong command}");
    }
}
