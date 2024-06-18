#include "imu_porcessor.h"
#include <math.h>
#include <stdio.h>

#define GYRO_BIAS_TAU 20.f

void IMUProcessor::init(int irq_pin, gpio_irq_callback_t gpio_irq_callback) {
    int status = imu.begin();
    gpio_set_irq_enabled_with_callback(irq_pin, GPIO_IRQ_EDGE_RISE, true,
                                       gpio_irq_callback);

    if (status < 0) {
        printf("imu initialization unsuccessful\n");
        printf("Check imu wiring or try cycling power\n");
        printf("Status: %d\n", status);
    }

    // set output data rate to 12.5 Hz
    imu.setAccelODR(ICM42688::odr1k);
    imu.setGyroODR(ICM42688::odr1k);
    imu.setGyroFS(ICM42688::dps250);
    imu.setAccelFS(ICM42688::gpm2);
    imu.enableDataReadyInterrupt();
}

void IMUProcessor::process(uint32_t packet_time) {
    // read the sensor
    imu.getAGT();

    if (last_packet_time != 0) {
        float dt = (packet_time - last_packet_time) / 1e6;
        Vector3D row_rate = {imu.gyrX(), imu.gyrY(), imu.gyrZ()};
        gyro_rate = row_rate - gyro_bias;

        // bias calibration
        if (state == BiasClb) {
            gyro_bias += (row_rate - gyro_bias) * (dt / GYRO_BIAS_TAU);
            // IDE state
        } else {
            // apply scale
            gyro_rate *= gyro_scale;

            // used for calculate bias
            if (fabs(gyro_rate[2]) < 0.1f) {
                // update gyro bias
                gyro_bias += (row_rate - gyro_bias) * (dt / GYRO_BIAS_TAU);
                // integrate angles
            } else {
                angles += gyro_rate * dt;
            }
        }
    }
    last_packet_time = packet_time;
}

void IMUProcessor::start_bias_calibration() {
    state = BiasClb;
    // reset biases
    gyro_bias = {0.};
}

bool IMUProcessor::stop_bias_calibration() {
    if (state != BiasClb) {
        return false;
    }
    res_gyro_bias = gyro_bias;
    state == Ide;
    return true;
}