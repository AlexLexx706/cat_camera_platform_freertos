#ifndef IMU_PORCESSOR_H
#define IMU_PORCESSOR_H
#include "ICM42688/ICM42688.h"
#include "hardware/gpio.h"
#include "utils/Vector3d.h"

class IMUProcessor {

    ICM42688 imu;
    uint32_t last_packet_time = 0;

 public:
    enum State { Ide = 0, BiasClb, ScaleClb } state = Ide;

    Vector3D gyro_rate = {0.f};
    Vector3D angles = {0.f};
    Vector3D gyro_bias = {0.f};
    Vector3D gyro_scale = {1.f, 1.f, 1.f};
    Vector3D res_gyro_bias = {0.f};

    void init(int irq_pin, gpio_irq_callback_t gpio_irq_callback);
    void process(uint32_t packet_time);
    void start_bias_calibration();
    bool stop_bias_calibration();
    bool is_bias_in_progress() const { return state == BiasClb; }
    void set_bias(const Vector3D &bias) { gyro_bias = bias; }
};

#endif // IMU_PORCESSOR_H
