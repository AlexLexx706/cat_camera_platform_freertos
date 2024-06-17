#ifndef IMU_PORCESSOR_H
#define IMU_PORCESSOR_H
#include "ICM42688/ICM42688.h"
#include "hardware/gpio.h"

class IMUProcessor {

    ICM42688 imu;
    uint32_t last_packet_time = 0;

 public:
    enum State { Ide = 0, BiasClb, ScaleClb } state = Ide;

    float gyro_rate[3] = {0.f};
    float angles[3] = {0.f};
    float gyro_bias[3] = {0.f};
    float gyro_scale[3] = {1.f, 1.f, 1.f};
    float res_gyro_bias[3] = {0.f};

    void init(int irq_pin, gpio_irq_callback_t gpio_irq_callback);
    void process(uint32_t packet_time);
    void start_bias_calibration();
    bool stop_bias_calibration();
    bool is_bias_in_progress() const { return state == BiasClb; }
};
#endif // IMU_PORCESSOR_H
