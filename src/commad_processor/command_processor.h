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
 * /encoder/0/state                     (print)
 * /encoder/0/save                      (set)
 *
 * /encoder/1/row                       (set/print)
 * /encoder/1/scale                     (set/print)
 * /encoder/1/value                     (set/print)
 * /encoder/1/state                     (print)
 * /encoder/1/save                      (set)
 *
 * /encoder/2/row                       (set/print)
 * /encoder/2/scale                     (set/print)
 * /encoder/2/value                     (set/print)
 * /encoder/2/state                     (print)
 * /encoder/2/save                      (set)

 * /encoder/debug_level                 (set/print)
 *
 * /flash/erase                         (set)
 */

/*
set,/encoder/0/row,0
set,/encoder/1/row,0
print,/encoder/0/row
print,/encoder/1/row

set,/encoder/0/scale,0.00011860754739359914
print,/encoder/0/scale
set,/encoder/0/save
print,/encoder/0/state

set,/encoder/1/scale,0.00011860754739359914
print,/encoder/1/scale
set,/encoder/1/save
print,/encoder/1/state
*/
namespace CommandProcessor {
void init();
void process(char symbol);
}; // namespace CommandProcessor
#endif // COMMAND_PROCESSOR_H