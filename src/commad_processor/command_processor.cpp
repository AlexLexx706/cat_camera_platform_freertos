#include "command_processor.h"
#include "utils/cmd_parser.h"
#include <string.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include <stdlib.h>
#include "flash_store/flash_store.h"
#include <hardware/flash.h>


#define VERSION "0.1"
#define AUTHOR "Alexlexx1@gmai.com"

static int mode;
static CommandParser command_parser;

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

/**
 * Commands:
 * /par/imu/calb/bias/start
 * /par/imu/calb/bias/stop
 * /par/imu/calb/bias/clear
 * /par/imu/calb/bias/state
 * /par/imu/calb/bias/x
 * /par/imu/calb/bias/y
 * /par/imu/calb/bias/z
 *
 * /par/imu/calb/scale/start
 * /par/imu/calb/scale/stop
 * /par/imu/calb/scale/clear
 * /par/imu/calb/scale/state
 * /par/imu/calb/scale/z
 
 * /par/flash/reset
 * /par/flash
 */

static uint8_t tmp_buffer[FLASH_PAGE_SIZE];

static void command_parser_cmd_cb(const char *prefix, const char *cmd,
                                  const char *parameter, const char *value) {
    // echo command
    if (!strlen(prefix) && !strlen(cmd)) {
        print_re(prefix, "%%");
        return;
    }

    // cheking print command
    if (strcmp(cmd, "print") == 0) {
        // print current version
        if (strcmp(parameter, "/par/version") == 0) {
            print_re(prefix, VERSION);
            // print author
        } else if (strcmp(parameter, "/par/author") == 0) {
            print_re(prefix, AUTHOR);
            // print current mode: auto or manual
        } else if (strcmp(parameter, "/par/mode") == 0) {
            char buffer[10];
            snprintf(buffer, sizeof(buffer), "%d", mode);
            print_re(prefix, buffer);
        } else if (strcmp(parameter, "/par/flash") == 0) {
            FlashStore::read(tmp_buffer, sizeof(tmp_buffer));
            print_re(prefix, reinterpret_cast<char *>(tmp_buffer));
        } else {
            print_er(prefix, "{6,wrong parameter}");
        }
        // process set commands
    } else if (strcmp(cmd, "set") == 0) {
        // wrong value
        if (value == nullptr) {
            print_er(prefix, "{7,wrong value}");
            return;
        }
        // set mode
        if (strcmp(parameter, "/par/mode") == 0) {
            int _mode = atoi(value);
            if (_mode == 0) {
                mode = 0;
                print_re(prefix, "");
            } else if (_mode == 1) {
                mode = 1;
                print_re(prefix, "");
            } else {
                print_er(prefix, "{7,wrong value}");
            }
            // set current finger state for manual mode
        } else if (strcmp(parameter, "/par/flash/reset") == 0) {
            int _mode = atoi(value);
            if (_mode == 1) {
                FlashStore::clear();
                print_re(prefix, "");
            } else {
                print_er(prefix, "{7,wrong value}");
            }
        } else if (strcmp(parameter, "/par/flash") == 0) {
            strcpy(reinterpret_cast<char*>(tmp_buffer), value); 
            FlashStore::save(tmp_buffer, sizeof(tmp_buffer));
            print_re(prefix, "");
        } else {
            print_er(prefix, "{6,wrong parameter}");
        }
        // wrong command
    } else {
        print_er(prefix, "{8,wrong command}");
    }
}

namespace CommandProcessor {
void init() { command_parser.set_callback(print_er, command_parser_cmd_cb); }

void process(char symbol) { command_parser.process_symbol(symbol); }
} // namespace CommandProcessor