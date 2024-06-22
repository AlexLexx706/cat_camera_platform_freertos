#include "imu_porcessor.h"
#include "utils/settings.h"
#include <math.h>
#include <stdio.h>

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

    bool giro_bias_init = true;
    // check initialization
    for (int i = 0; i < 3; i++) {
        if (reinterpret_cast<uint32_t *>(&settings->gyro_bias_settings)[i] ==
            0xffffffff) {
            giro_bias_init = false;
        }
    }
    printf("giro_bias_init:%d\n", giro_bias_init);

    // init giro bias from flash
    if (giro_bias_init) {
        set_bias(settings->gyro_bias_settings.bias);
    }

    imu.enableDataReadyInterrupt();
}

void IMUProcessor::process(uint32_t packet_time) {
    // read the sensor
    imu.getAGT();

    if (last_packet_time != 0) {
        float dt = (packet_time - last_packet_time) / 1e6;
        Vector3D row_rate = {imu.gyrX(), imu.gyrY(), imu.gyrZ()};

        // bias calibration
        if (state == BiasClb) {
            gyro_bias += (row_rate - gyro_bias) * (dt / gyro_bias_tau);

            //print debug
            if (debug_level > 0 && (packet_time - last_print_time) > print_period) {
                last_print_time = packet_time;
                printf("%f %f %f\n", gyro_bias.x0, gyro_bias.x1, gyro_bias.x2);
            }
            // IDE state
        } else {
            gyro_rate = row_rate - (gyro_bias + internal_gyro_bias);

            // apply scale
            gyro_rate *= gyro_scale;

            // used for calculate bias
            if (allow_dynamic_bias &&
                fabs(gyro_rate[2]) < bias_rate_threshold) {
                // update gyro bias
                internal_gyro_bias +=
                    ((row_rate - gyro_bias) - internal_gyro_bias) *
                    (dt / gyro_bias_tau);
                // integrate angles
            } else {
                angles += gyro_rate * dt;
            }
            //print debug
            if (debug_level > 0 && (packet_time - last_print_time) > print_period) {
                last_print_time = packet_time;
                printf(
                    "%f %f %f %f %f %f %f %f %f %f %f %f\n",
                    gyro_bias.x0, gyro_bias.x1, gyro_bias.x2,
                    internal_gyro_bias.x0, internal_gyro_bias.x1, internal_gyro_bias.x2,
                    angles.x0, angles.x1, angles.x2,
                    gyro_rate.x0, gyro_rate.x1, gyro_rate.x2);
            }

        }
    }
    last_packet_time = packet_time;
}

void IMUProcessor::start_bias_calibration() {
    state = BiasClb;
    gyro_bias = {0.};
}

bool IMUProcessor::stop_bias_calibration() {
    if (state != BiasClb) {
        return false;
    }
    state = Ide;
    return true;
}