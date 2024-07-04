#include "encoder/encoder.h"
#include "imu_porcessor.h"
#include "pico/time.h"
#include "utils/settings.h"
#include <math.h>
#include <stdio.h>

bool IMUProcessor::init(int irq_pin, gpio_irq_callback_t gpio_irq_callback,
                        int task_prio, int stack_size) {

    packet_semaphore = xSemaphoreCreateBinary();
    if (packet_semaphore == NULL) {
        printf("IMUProcessor::init can`t create Semaphore\n");
        return false;
    }

    if (xTaskCreate(task_handler, "imu", stack_size, this, task_prio, &task) !=
        pdPASS) {
        printf("IMUProcessor::init can`t create task\n");
        return false;
    }

    int status = imu.begin();
    if (status < 0) {
        printf("imu initialization unsuccessful\n");
        printf("Check imu wiring or try cycling power\n");
        printf("Status: %d\n", status);
        return false;
    }

    gpio_set_irq_enabled_with_callback(irq_pin, GPIO_IRQ_EDGE_RISE, true,
                                       gpio_irq_callback);

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

    // init giro bias from flash
    if (giro_bias_init) {
        set_bias(settings->gyro_bias_settings.bias);
    }

    imu.enableDataReadyInterrupt();
    return true;
}

void IMUProcessor::task_handler(void *value) {
    reinterpret_cast<IMUProcessor *>(value)->process();
}

void IMUProcessor::irq_handler(BaseType_t &xHigherPriorityTaskWoken) {
    packet_time_us = time_us_32();
    xSemaphoreGiveFromISR(packet_semaphore, &xHigherPriorityTaskWoken);
}

void IMUProcessor::process() {
    for (;;) {
        /* Block on the queue to wait for data to arrive. */
        xSemaphoreTake(packet_semaphore, portMAX_DELAY);

        // read the sensor
        imu.getAGT();

        if (last_packet_time_us != 0) {
            float dt = (packet_time_us - last_packet_time_us) / 1e6;
            Vector3D row_rate = {imu.gyrX(), imu.gyrY(), imu.gyrZ()};

            // bias calibration
            if (state == BiasClb) {
                gyro_bias += (row_rate - gyro_bias) * (dt / gyro_bias_tau);

                // print debug
                if (debug_level > 0 &&
                    (packet_time_us - last_print_time_us) > print_period_us) {
                    last_print_time_us = packet_time_us;
                    printf("%f %f %f\n", gyro_bias.x0, gyro_bias.x1,
                           gyro_bias.x2);
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

                // calculating speed from encoder
                if ((packet_time_us - last_encoder_time_us) >= speed_update_period_us) {
                    if (init_encoder) {
                        init_encoder = false;
                        last_encoder_pos[0] = encoder.get_value(0);
                        last_encoder_pos[1] = encoder.get_value(1);
                    } else {
                        float _encoder_pos[2];
                        _encoder_pos[0] = encoder.get_value(0);
                        _encoder_pos[1] = encoder.get_value(1);
                        float dt = (packet_time_us - last_encoder_time_us) / 1e6;

                        encoder_speed_ms[0] = (_encoder_pos[0] - last_encoder_pos[0]) / dt;
                        encoder_speed_ms[1] = -(_encoder_pos[1] - last_encoder_pos[1]) / dt;
                        mid_encoder_speed_ms = (encoder_speed_ms[0] + encoder_speed_ms[1]) / 2.f;

                        encoder_omega = (encoder_speed_ms[1] - encoder_speed_ms[0]) / wheel_base;
                        encoder_heading += encoder_omega * dt;

                        Vector3D dir = {cos(encoder_heading), sin(encoder_heading), 0.f};
                        encoder_pos += dir * mid_encoder_speed_ms * dt;

                        last_encoder_pos[0] = _encoder_pos[0];
                        last_encoder_pos[1] = _encoder_pos[1];
                    }
                    last_encoder_time_us = packet_time_us;
                }

                // calculating of position
                float angle = angles[2] / 180. * M_PI;
                Vector3D dir = {cos(angle), sin(angle), 0.f};
                pos += dir * mid_encoder_speed_ms * dt;

                // print debug
                if ((packet_time_us - last_print_time_us) > print_period_us) {
                    last_print_time_us = packet_time_us;

                    // gyro bias debug
                    if (debug_level == 1) {
                        printf("%f %f %f %f %f %f %f %f %f %f %f %f\n",
                               gyro_bias.x0, gyro_bias.x1, gyro_bias.x2,
                               internal_gyro_bias.x0, internal_gyro_bias.x1,
                               internal_gyro_bias.x2, angles.x0, angles.x1,
                               angles.x2, gyro_rate.x0, gyro_rate.x1,
                               gyro_rate.x2);
                    // encoder speed debug
                    } else if (debug_level == 2) {
                        printf("%f %f %f\n",
                            encoder_speed_ms[0],
                            encoder_speed_ms[1],
                            mid_encoder_speed_ms);
                    // position debug
                    } else if (debug_level == 3) {
                        printf("%f %f %f %f\n",
                            pos[0],
                            pos[1],
                            angles[2],
                            mid_encoder_speed_ms);
                    // enc position debug
                    } else if (debug_level == 4) {
                        float heading = encoder_heading / M_PI * 180.;
                        printf("%f %f %f %f %f, %f %f\n",
                            encoder_pos[0], encoder_pos[1],
                            heading, angles[2], mid_encoder_speed_ms,
                            encoder.get_value(0),
                            encoder.get_value(1));
                    }
                }
            }
        }
        last_packet_time_us = packet_time_us;
    }
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