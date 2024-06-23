#ifndef COMMAND_PROCESSOR_H
#define COMMAND_PROCESSOR_H

/**
 * Commands:
 * /version                             (print)
 * /author                              (print)
 *
 * /imu/clb/gyro_bias/start             (set)
 * /imu/clb/gyro_bias/stop              (set)
 * /imu/clb/gyro_bias/save              (set)
 * /imu/clb/gyro_bias/debug_level       (set/print)
 * /imu/clb/gyro_bias/state             (print)
 * /imu/gyro_bias/x                     (set/print)
 * /imu/gyro_bias/y                     (set/print)
 * /imu/gyro_bias/z                     (set/print)
 * /imu/heading                         (set/print)
 *
 * /encoder/0/row                       (set/print)
 * /encoder/0/scale                     (set/print)
 * /encoder/0/value                     (set/print)
 *
 * /encoder/1/row                       (set/print)
 * /encoder/1/scale                     (set/print)
 * /encoder/1/value                     (set/print)
 * /encoder/debug_level                 (set/print)
 *
 * /flash/erase                     (set)
 */

/*
set,/encoder/0/row,0
set,/encoder/1/row,0
print,/encoder/0/row
print,/encoder/1/row
*/
namespace CommandProcessor {
    void init();
    void process(char symbol);
};
#endif //COMMAND_PROCESSOR_H