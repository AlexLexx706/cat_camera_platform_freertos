#include "FreeRTOS.h"
#include "ICM42688/ICM42688.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include "semphr.h"
#include "task.h"
#include <math.h>
#include <stdio.h>

#define ICM42688_IRQ_PIN 22
#define GYRO_BIAS_TAU 20.f
volatile static uint32_t packet_time;
volatile static uint32_t last_packet_time = 0;
volatile static uint32_t dt = 0;
volatile static float gyro_rate[3] = {0.f};
volatile static float angles[3] = {0.f};
volatile static float gyro_bias[3] = {0.f};
// static float gyro_scale[3] = {1.f, 1.f, 1.0257511014952223f};
static float gyro_scale[3] = {1.f, 1.f, 1.f};

SemaphoreHandle_t imu_data_semaphore;
static ICM42688 IMU;

static void gpio_callback(uint gpio, uint32_t events) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // IMU Interrupt pin
    if (gpio == ICM42688_IRQ_PIN) {
        packet_time = time_us_32();
        xSemaphoreGiveFromISR(imu_data_semaphore, &xHigherPriorityTaskWoken);
    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

static void print_state(void *) {
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000 / 10));
        printf("%+6.5f %+6.3f %+6.3f\n", gyro_bias[2], gyro_rate[2], angles[2]);
    }
}

static void process_imu(void *) {
    for (;;) {
        /* Block on the queue to wait for data to arrive. */
        xSemaphoreTake(imu_data_semaphore, portMAX_DELAY);
        // read the sensor
        IMU.getAGT();

        if (last_packet_time != 0) {
            dt = (packet_time - last_packet_time);
            last_packet_time = packet_time;
            float dt_f = dt / 1e6;
            float row_gyro_rate[3];
            row_gyro_rate[0] = IMU.gyrX() * gyro_scale[0];
            row_gyro_rate[1] = IMU.gyrY() * gyro_scale[1];
            row_gyro_rate[2] = IMU.gyrZ() * gyro_scale[2];

            gyro_rate[0] = row_gyro_rate[0] - gyro_bias[0];
            gyro_rate[1] = row_gyro_rate[1] - gyro_bias[1];
            gyro_rate[2] = row_gyro_rate[2] - gyro_bias[2];

            // //used for calculate bias
            if (fabs(gyro_rate[2]) < 0.1f) {
                // update gyro bias
                float factor = dt_f / GYRO_BIAS_TAU;
                gyro_bias[0] += factor * (row_gyro_rate[0] - gyro_bias[0]);
                gyro_bias[1] += factor * (row_gyro_rate[1] - gyro_bias[1]);
                gyro_bias[2] += factor * (row_gyro_rate[2] - gyro_bias[2]);
            } else {
                angles[0] += gyro_rate[0] * dt_f;
                angles[1] += gyro_rate[1] * dt_f;
                angles[2] += gyro_rate[2] * dt_f;
            }
        } else {
            last_packet_time = packet_time;
        }
    }
}

int main() {
    stdio_init_all();
    sleep_ms(2000);
    imu_data_semaphore = xSemaphoreCreateBinary();

    if (imu_data_semaphore != NULL) {
        // start communication with IMU
        int status = IMU.begin();
        gpio_set_irq_enabled_with_callback(ICM42688_IRQ_PIN, GPIO_IRQ_EDGE_RISE,
                                           true, &gpio_callback);

        if (status < 0) {
            printf("IMU initialization unsuccessful\n");
            printf("Check IMU wiring or try cycling power\n");
            printf("Status: %d\n", status);
        }

        // set output data rate to 12.5 Hz
        IMU.setAccelODR(ICM42688::odr1k);
        IMU.setGyroODR(ICM42688::odr1k);
        IMU.setGyroFS(ICM42688::dps250);
        IMU.setAccelFS(ICM42688::gpm2);
        IMU.enableDataReadyInterrupt();

        xTaskCreate(process_imu, "imu Task", 1024, NULL, 2, NULL);
        xTaskCreate(print_state, "test_task", 1024, NULL, 1, NULL);

        // enabling the data ready interrupt
        vTaskStartScheduler();
    }
    while (true) {
        printf("fail!!!\n");
    }
}