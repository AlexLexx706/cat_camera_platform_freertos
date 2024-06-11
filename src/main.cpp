#include "FreeRTOS.h"
#include "ICM42688/ICM42688.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include "semphr.h"
#include "task.h"
#include <math.h>
#include <stdio.h>

#define ENC_A1 8
#define ENC_B1 9

#define ENC_A2 10
#define ENC_B2 11

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

typedef struct EncoderState {
    int8_t last_encoded;
    int32_t encoder_value;
} EncoderState;

static EncoderState enc_1 = {};
static EncoderState enc_2 = {};

static SemaphoreHandle_t imu_data_semaphore;
static SemaphoreHandle_t encoder_semaphore;
volatile static bool is_enc1;
static ICM42688 IMU;

static void gpio_callback(uint gpio, uint32_t events) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // IMU Interrupt pin
    if (gpio == ICM42688_IRQ_PIN) {
        packet_time = time_us_32();
        xSemaphoreGiveFromISR(imu_data_semaphore, &xHigherPriorityTaskWoken);
        // process encoder
    } else {
        is_enc1 = (gpio == ENC_A1 || gpio == ENC_B1);
        xSemaphoreGiveFromISR(encoder_semaphore, &xHigherPriorityTaskWoken);
    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

static void process_encoder(void *) {
    for (;;) {
        xSemaphoreTake(encoder_semaphore, portMAX_DELAY);

        EncoderState *state;
        int encoded;
        if (is_enc1) {
            state = &enc_1;
            encoded = (gpio_get(ENC_A1) << 1) | gpio_get(ENC_B1);
        } else {
            state = &enc_2;
            encoded = (gpio_get(ENC_A2) << 1) | gpio_get(ENC_B2);
        }
        int sum = (state->last_encoded << 2) | encoded;

        if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011)
            state->encoder_value += 1;
        if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000)
            state->encoder_value -= 1;
        state->last_encoded = encoded;
    }
}

static void print_state(void *) {
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000 / 10));
        printf("%+6.5f %+6.3f %+6.3f %d %d\n", gyro_bias[2], gyro_rate[2],
               angles[2], enc_1.encoder_value, enc_2.encoder_value);
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
    encoder_semaphore = xSemaphoreCreateBinary();

    if (imu_data_semaphore != NULL && encoder_semaphore != NULL) {
        // init encoder
        gpio_init(ENC_A1);
        gpio_init(ENC_A2);
        gpio_init(ENC_B1);
        gpio_init(ENC_B2);

        gpio_set_dir(ENC_A1, GPIO_IN);
        gpio_set_dir(ENC_A2, GPIO_IN);
        gpio_set_dir(ENC_B1, GPIO_IN);
        gpio_set_dir(ENC_B2, GPIO_IN);

        // https://cdn-shop.adafruit.com/product-files/4640/n20+motors_C15011+6V.pdf
        gpio_pull_up(ENC_A1);
        gpio_pull_up(ENC_A2);
        gpio_pull_up(ENC_B1);
        gpio_pull_up(ENC_B2);

        gpio_set_irq_enabled_with_callback(
            ENC_A1, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true,
            &gpio_callback);

        gpio_set_irq_enabled_with_callback(
            ENC_B1, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true,
            &gpio_callback);

        gpio_set_irq_enabled_with_callback(
            ENC_A2, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true,
            &gpio_callback);

        gpio_set_irq_enabled_with_callback(
            ENC_B2, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true,
            &gpio_callback);

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

        xTaskCreate(process_imu, "imu", 1024, NULL, 3, NULL);
        xTaskCreate(process_encoder, "encoder", 1024, NULL, 2, NULL);
        xTaskCreate(print_state, "print", 1024, NULL, 1, NULL);

        // enabling the data ready interrupt
        vTaskStartScheduler();
    }
    while (true) {
        printf("fail!!!\n");
    }
}