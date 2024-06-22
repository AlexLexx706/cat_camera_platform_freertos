#ifndef COMMAND_PROCESSOR_H
#define COMMAND_PROCESSOR_H

/**
 * Commands:
 * /version                         (print)
 * /author                          (print)
 *
 * /clb/gyro_bias/start             (set)
 * /clb/gyro_bias/stop              (set)
 * /clb/gyro_bias/save              (set)
 * /clb/gyro_bias/debug_level       (set/print)
 * /clb/gyro_bias/state             (print)
 * /clb/gyro_bias/x                 (set/print)
 * /clb/gyro_bias/y                 (set/print)
 * /clb/gyro_bias/z                 (set/print)
 * /clb/gyro_bias/heading           (set/print)
 *
 * /clb/gyro_scale/start            (set)
 * /clb/gyro_scale/stop             (set)
 * /clb/gyro_scale/state            (print)
 * /clb/gyro_scale/z                (set/print)
 *
 * /flash/erase                     (set)
 */
namespace CommandProcessor {
    void init();
    void process(char symbol);
};
#endif //COMMAND_PROCESSOR_H