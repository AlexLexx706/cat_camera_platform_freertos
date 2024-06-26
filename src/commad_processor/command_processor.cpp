#include "command_processor.h"
#include "encoder/encoder.h"
#include "flash_store/flash_store.h"
#include "imu_processor/imu_porcessor.h"
#include "pico/stdlib.h"
#include "utils/cmd_parser.h"
#include "utils/settings.h"
#include <hardware/flash.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define VERSION "0.1"
#define AUTHOR "Alexlexx1@gmai.com"

static CommandParser command_parser;
Settings *settings = reinterpret_cast<Settings *>(FlashStore::store_ptr);

// write error to port
static void print_er(const char *prefix, const char *msg) {
    assert(prefix);
    assert(msg);

    char buffer[60];
    int prefix_len = strlen(prefix);
    int len;
    if (prefix_len) {
        len = snprintf(buffer, sizeof(buffer), "ER%03X%%%s%%%s\r\n",
                       strlen(msg) + prefix_len + 2, prefix, msg);
    } else {
        len =
            snprintf(buffer, sizeof(buffer), "ER%03X%s\r\n", strlen(msg), msg);
    }
    puts_raw(buffer);
    stdio_flush();
}

// write responce msg
static void print_re(const char *prefix, const char *msg) {
    assert(prefix);
    assert(msg);

    char buffer[60];
    int prefix_len = strlen(prefix);
    int msg_len = strlen(msg);
    int len;

    if (prefix_len) {
        len = snprintf(buffer, sizeof(buffer), "RE%03X%%%s%%%s\r\n",
                       strlen(prefix) + msg_len + 2, prefix, msg);

        puts_raw(buffer);
    } else if (msg_len) {
        len = snprintf(buffer, sizeof(buffer), "RE%03X%s\r\n", msg_len, msg);
        puts_raw(buffer);
    }
    stdio_flush();
}

constexpr auto imu_path = "/imu/";
constexpr auto imu_path_len = strlen(imu_path);

/*
 * /imu/clb/gyro_bias/start             (set)
 * /imu/clb/gyro_bias/stop              (set)
 * /imu/clb/gyro_bias/save              (set)
 * /imu/clb/gyro_bias/state             (print)
 * /imu/gyro_bias/x                     (set/print)
 * /imu/gyro_bias/y                     (set/print)
 * /imu/gyro_bias/z                     (set/print)
 * /imu/debug_level                     (set/print)
 * /imu/heading                         (set/print)
 *
 * /imu/wheel_base                      (set/print)
 * /imu/encoder_omega                   (print)
 * /imu/encoder_heading                 (set/print)
 * /imu/encoder_pos/x                   (set/print)
 * /imu/encoder_pos/y                   (set/print)

 */
static void process_imu(const char *prefix, const char *cmd,
                        const char *parameter, const char *value) {
    if (strcmp(cmd, "set") == 0) {
        if (strcmp(parameter, "clb/gyro_bias/start") == 0) {
            imu_processor.start_bias_calibration();
            print_re(prefix, "");
        } else if (strcmp(parameter, "clb/gyro_bias/stop") == 0) {
            imu_processor.stop_bias_calibration();
            print_re(prefix, "");
        } else if (strcmp(parameter, "clb/gyro_bias/save") == 0) {
            char buffer[FLASH_PAGE_SIZE];
            // copy current memory state
            memcpy(buffer, settings, sizeof(buffer));
            reinterpret_cast<Settings *>(buffer)->gyro_bias_settings.bias =
                imu_processor.get_bias();
            FlashStore::save(reinterpret_cast<const uint8_t *>(buffer),
                             sizeof(buffer));
            print_re(prefix, "");
        } else if (strcmp(parameter, "wheel_base") == 0) {
            if (value == nullptr) {
                print_er(prefix, "{7,wrong value}");
            } else {
                float base = atof(value);
                imu_processor.set_wheel_base(base);
                print_re(prefix, "");
            }
        } else if (strcmp(parameter, "encoder_heading") == 0) {
            if (value == nullptr) {
                print_er(prefix, "{7,wrong value}");
            } else {
                float heading = atof(value);
                imu_processor.set_encoder_heading(heading);
                print_re(prefix, "");
            }
        } else if (strcmp(parameter, "encoder_pos/x") == 0) {
            if (value == nullptr) {
                print_er(prefix, "{7,wrong value}");
            } else {
                float x = atof(value);
                Vector3D pos = imu_processor.get_encoder_pos();
                pos.x0 = x;
                imu_processor.set_encoder_pos(pos);
                print_re(prefix, "");
            }
        } else if (strcmp(parameter, "encoder_pos/y") == 0) {
            if (value == nullptr) {
                print_er(prefix, "{7,wrong value}");
            } else {
                float y = atof(value);
                Vector3D pos = imu_processor.get_encoder_pos();
                pos.x1 = y;
                imu_processor.set_encoder_pos(pos);
                print_re(prefix, "");
            }
        } else if (strcmp(parameter, "debug_level") == 0) {
            if (value == nullptr) {
                print_er(prefix, "{7,wrong value}");
            } else {
                int level = atoi(value);
                imu_processor.set_debug_level(level);
                print_re(prefix, "");
            }
        } else if (strcmp(parameter, "heading") == 0) {
            if (value == nullptr) {
                print_er(prefix, "{7,wrong value}");
            } else {
                float heading = atof(value);
                Vector3D angles = imu_processor.get_angles();
                angles.x2 = heading;
                imu_processor.set_angles(angles);
                print_re(prefix, "");
            }
        } else if (strcmp(parameter, "x") == 0) {
            if (value == nullptr) {
                print_er(prefix, "{7,wrong value}");
            } else {
                float x = atof(value);
                Vector3D pos = imu_processor.get_pos();
                pos.x0 = x;
                imu_processor.set_pos(pos);
                print_re(prefix, "");
            }
        } else if (strcmp(parameter, "y") == 0) {
            if (value == nullptr) {
                print_er(prefix, "{7,wrong value}");
            } else {
                float y = atof(value);
                Vector3D pos = imu_processor.get_pos();
                pos.x1 = y;
                imu_processor.set_pos(pos);
                print_re(prefix, "");
            }
        } else if (strcmp(parameter, "gyro_bias/x") == 0) {
            if (value == nullptr) {
                print_er(prefix, "{7,wrong value}");
            } else {
                Vector3D cur_bias = imu_processor.get_bias();
                cur_bias.x0 = atof(value);
                imu_processor.set_bias(cur_bias);
                print_re(prefix, "");
            }
        } else if (strcmp(parameter, "gyro_bias/y") == 0) {
            if (value == nullptr) {
                print_er(prefix, "{7,wrong value}");
            } else {
                Vector3D cur_bias = imu_processor.get_bias();
                cur_bias.x1 = atof(value);
                imu_processor.set_bias(cur_bias);
                print_re(prefix, "");
            }
        } else if (strcmp(parameter, "gyro_bias/z") == 0) {
            if (value == nullptr) {
                print_er(prefix, "{7,wrong value}");
            } else {
                Vector3D cur_bias = imu_processor.get_bias();
                cur_bias.x2 = atof(value);
                imu_processor.set_bias(cur_bias);
                print_re(prefix, "");
            }
        } else {
            print_er(prefix, "{6,wrong parameter}");
        }
    } else if (strcmp(cmd, "print") == 0) {
        if (strcmp(parameter, "clb/gyro_bias/state") == 0) {
            if (imu_processor.is_bias_clb_on()) {
                print_re(prefix, "on");
            } else if (*reinterpret_cast<uint32_t *>(
                           &settings->gyro_bias_settings) != 0xffffffff) {
                print_re(prefix, "clear");
            } else if (settings->gyro_bias_settings.bias !=
                       imu_processor.get_bias()) {
                print_re(prefix, "not_sync");
            } else {
                print_re(prefix, "sync");
            }
        } else if (strcmp(parameter, "heading") == 0) {
            char buffer[32];
            snprintf(buffer, sizeof(buffer), "%f",
                     imu_processor.get_angles().x2);
            print_re(prefix, buffer);
        } else if (strcmp(parameter, "wheel_base") == 0) {
            char buffer[32];
            snprintf(buffer, sizeof(buffer), "%f", imu_processor.get_wheel_base());
            print_re(prefix, buffer);
        } else if (strcmp(parameter, "encoder_omega") == 0) {
            char buffer[32];
            snprintf(buffer, sizeof(buffer), "%f", imu_processor.get_encoder_omega());
            print_re(prefix, buffer);
        } else if (strcmp(parameter, "encoder_heading") == 0) {
            char buffer[32];
            snprintf(buffer, sizeof(buffer), "%f", imu_processor.get_encoder_heading());
            print_re(prefix, buffer);
        } else if (strcmp(parameter, "encoder_pos/x") == 0) {
            char buffer[32];
            snprintf(buffer, sizeof(buffer), "%f", imu_processor.get_encoder_pos().x0);
            print_re(prefix, buffer);
        } else if (strcmp(parameter, "encoder_pos/y") == 0) {
            char buffer[32];
            snprintf(buffer, sizeof(buffer), "%f", imu_processor.get_encoder_pos().x1);
            print_re(prefix, buffer);
        } else if (strcmp(parameter, "x") == 0) {
            char buffer[32];
            snprintf(buffer, sizeof(buffer), "%f", imu_processor.get_pos().x0);
            print_re(prefix, buffer);
        } else if (strcmp(parameter, "y") == 0) {
            char buffer[32];
            snprintf(buffer, sizeof(buffer), "%f", imu_processor.get_pos().x1);
            print_re(prefix, buffer);
        } else if (strcmp(parameter, "debug_level") == 0) {
            char buffer[32];
            snprintf(buffer, sizeof(buffer), "%d",
                     imu_processor.get_debug_level());
            print_re(prefix, buffer);
        } else if (strcmp(parameter, "gyro_bias/x") == 0) {
            char buffer[32];
            snprintf(buffer, sizeof(buffer), "%f", imu_processor.get_bias().x0);
            print_re(prefix, buffer);
        } else if (strcmp(parameter, "gyro_bias/y") == 0) {
            char buffer[32];
            snprintf(buffer, sizeof(buffer), "%f", imu_processor.get_bias().x1);
            print_re(prefix, buffer);
        } else if (strcmp(parameter, "gyro_bias/z") == 0) {
            char buffer[32];
            snprintf(buffer, sizeof(buffer), "%f", imu_processor.get_bias().x2);
            print_re(prefix, buffer);
        } else {
            print_er(prefix, "{6,wrong parameter}");
        }
    } else {
        print_er(prefix, "{8,wrong command}");
    }
}

constexpr auto encoder_path = "/encoder/";
constexpr auto encoder_path_len = strlen(encoder_path);

/*
 * /encoder/0/row                       (set/print)
 * /encoder/0/scale                     (set/print)
 * /encoder/0/value                     (print)
 *
 * /encoder/1/row                       (set/print)
 * /encoder/1/scale                     (set/print)
 * /encoder/1/value                     (set/print)
 * /encoder/debug_level                 (set/print)
 * /encoder/state                       (print)
 * /encoder/save                        (set)
 */
static void process_encoder(const char *prefix, const char *cmd,
                            const char *parameter, const char *value) {
    if (strcmp(cmd, "set") == 0) {
        if (value != nullptr) {
            int enc = -1;
            if (strncmp(parameter, "0/", 2) == 0) {
                enc = 0;
                parameter = &parameter[2];
            } else if (strncmp(parameter, "1/", 2) == 0) {
                enc = 1;
                parameter = &parameter[2];
            } else if (strncmp(parameter, "2/", 2) == 0) {
                enc = 2;
                parameter = &parameter[2];
            }
            if (enc != -1) {
                if (strcmp(parameter, "row") == 0) {
                    encoder.set_row(enc, atof(value));
                    print_re(prefix, "");
                } else if (strcmp(parameter, "scale") == 0) {
                    encoder.set_scale(enc, atof(value));
                    print_re(prefix, "");
                } else if (strcmp(parameter, "value") == 0) {
                    encoder.set_value(enc, atof(value));
                    print_re(prefix, "");
                } else if (strcmp(parameter, "save") == 0) {
                    char buffer[FLASH_PAGE_SIZE];
                    // copy current memory state
                    memcpy(buffer, settings, sizeof(buffer));
                    reinterpret_cast<Settings *>(buffer)
                        ->encoder_settings.scale[enc] = encoder.get_scale(enc);
                    FlashStore::save(reinterpret_cast<const uint8_t *>(buffer),
                                     sizeof(buffer));
                    print_re(prefix, "");
                } else {
                    print_er(prefix, "{6,wrong parameter}");
                }
            } else if (strcmp(parameter, "debug_level") == 0) {
                encoder.set_debug_level(atoi(value));
            } else {
                print_er(prefix, "{6,wrong parameter}");
            }
        } else {
            print_er(prefix, "{7,wrong value}");
        }
    } else if (strcmp(cmd, "print") == 0) {
        int enc = -1;
        if (strncmp(parameter, "0/", 2) == 0) {
            enc = 0;
            parameter = &parameter[2];
        } else if (strncmp(parameter, "1/", 2) == 0) {
            enc = 1;
            parameter = &parameter[2];
        } else if (strncmp(parameter, "2/", 2) == 0) {
            enc = 2;
            parameter = &parameter[2];
        }

        if (enc != -1) {
            if (strcmp(parameter, "row") == 0) {
                char buffer[32];
                snprintf(buffer, sizeof(buffer), "%d", encoder.get_row(enc));
                print_re(prefix, buffer);
            } else if (strcmp(parameter, "scale") == 0) {
                char buffer[32];
                snprintf(buffer, sizeof(buffer), "%f", encoder.get_scale(enc));
                print_re(prefix, buffer);
            } else if (strcmp(parameter, "value") == 0) {
                char buffer[32];
                snprintf(buffer, sizeof(buffer), "%f", encoder.get_value(enc));
                print_re(prefix, buffer);
            } else if (strcmp(parameter, "state") == 0) {
                if (*reinterpret_cast<uint32_t *>(
                        &settings->encoder_settings.scale[enc]) == 0xffffffff) {
                    print_re(prefix, "clear");
                } else if (encoder.get_scale(enc) ==
                           settings->encoder_settings.scale[enc]) {
                    print_re(prefix, "sync");
                } else {
                    print_re(prefix, "not_sync");
                }
            } else {
                print_er(prefix, "{6,wrong parameter}");
            }
        } else if (strcmp(parameter, "debug_level") == 0) {
            char buffer[32];
            snprintf(buffer, sizeof(buffer), "%d", encoder.get_debug_level());
            print_re(prefix, buffer);
        } else {
            print_er(prefix, "{6,wrong parameter}");
        }
    } else {
        print_er(prefix, "{8,wrong command}");
    }
}

static void command_parser_cmd_cb(const char *prefix, const char *cmd,
                                  const char *parameter, const char *value) {
    // echo command
    if (!strlen(prefix) && !strlen(cmd)) {
        print_re(prefix, "%%");
        return;
    }
    if (parameter != nullptr) {
        // process gyro_bias
        if (strncmp(parameter, imu_path, imu_path_len) == 0) {
            process_imu(prefix, cmd, &parameter[imu_path_len], value);
        } else if (strncmp(parameter, encoder_path, encoder_path_len) == 0) {
            process_encoder(prefix, cmd, &parameter[encoder_path_len], value);
            // print command
        } else if (strcmp(cmd, "print") == 0) {
            // print current version
            if (strcmp(parameter, "/version") == 0) {
                print_re(prefix, VERSION);
                // print author
            } else if (strcmp(parameter, "/author") == 0) {
                print_re(prefix, AUTHOR);
                // print author
            } else {
                print_er(prefix, "{6,wrong parameter}");
            }
            // set commands
        } else if (strcmp(cmd, "set") == 0) {
            if (strcmp(parameter, "/flash/erase") == 0) {
                FlashStore::erase();
                print_re(prefix, "");
            } else {
                print_er(prefix, "{6,wrong parameter}");
            }
        } else {
            print_er(prefix, "{8,wrong command}");
        }
    } else {
        print_er(prefix, "{6,wrong parameter}");
    }
}

namespace CommandProcessor {
void init() { command_parser.set_callback(print_er, command_parser_cmd_cb); }

void process(char symbol) { command_parser.process_symbol(symbol); }
} // namespace CommandProcessor