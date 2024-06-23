#include "FreeRTOS.h"
#include "VL53L0X/VL53L0X.h"
#include "hardware/gpio.h"
#include "imu_processor/imu_porcessor.h"
#include "encoder/encoder.h"
#include "commad_processor/command_processor.h"
#include "pico/binary_info.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include "semphr.h"
#include "task.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

#define ENC_A1 8
#define ENC_B1 9

#define ENC_A2 10
#define ENC_B2 11

#define ICM42688_IRQ_PIN 22

#define SDA_PIN 6
#define SCL_PIN 7

static VL53L0X sensor;
volatile static uint32_t mode = 0;

volatile static uint32_t packet_time;
volatile static uint32_t dt = 0;
volatile static uint16_t rf_range = 0;

static SemaphoreHandle_t imu_data_semaphore;

IMUProcessor imu_processor;
Encoder encoder;

static void gpio_callback(uint gpio, uint32_t events) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    // imu Interrupt pin
    if (gpio == ICM42688_IRQ_PIN) {
        packet_time = time_us_32();
        xSemaphoreGiveFromISR(imu_data_semaphore, &xHigherPriorityTaskWoken);
        // process encoder
    } else {
        encoder.irq_handler(gpio, xHigherPriorityTaskWoken);
    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

static void process_imu(void *) {
    for (;;) {
        /* Block on the queue to wait for data to arrive. */
        xSemaphoreTake(imu_data_semaphore, portMAX_DELAY);
        imu_processor.process(packet_time);
    }
}

static void process_range(void *) {
    // Start continuous back-to-back mode (take readings as
    // fast as possible). To use continuous timed mode
    // instead, provide a desired inter-measurement period in
    // ms (e.g. sensor.startContinuous(100)).
    sensor.startContinuous();

    while (1) {
        rf_range = sensor.readRangeContinuousMillimeters();
        if (sensor.timeoutOccurred()) {
            printf("range timeout\n");
        }
        vTaskDelay(pdMS_TO_TICKS(1000 / 10));
    }
}

static void process_cmd(void *) {
    int symbol;
    for (;;) {
        symbol = getchar_timeout_us(0);

        if (symbol == PICO_ERROR_TIMEOUT) {
            vTaskDelay(pdMS_TO_TICKS(1));
            // new symbol send back
        } else {
            CommandProcessor::process(symbol);
        }
    }
}

int main() {
    stdio_init_all();
    CommandProcessor::init();
    sleep_ms(5000);
    imu_data_semaphore = xSemaphoreCreateBinary();

    if (imu_data_semaphore != NULL &&
        encoder.init(ENC_A1, ENC_B1, ENC_A2, ENC_B2, gpio_callback, 2, 1024)) {

        // 2. init range-finder:
        i2c_init(&i2c1_inst, 400 * 1000);
        gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
        gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
        gpio_pull_up(SDA_PIN);
        gpio_pull_up(SCL_PIN);
        // Make the I2C pins available to picotool
        bi_decl(bi_2pins_with_func(SDA_PIN, SCL_PIN, GPIO_FUNC_I2C));

        sensor.setBus(&i2c1_inst);
        sensor.setTimeout(500);

        if (!sensor.init()) {
            printf("Failed to detect and initialize sensor!\n");
            while (1) {
                printf("Failed to detect and initialize sensor!\n");
                sleep_ms(1000);
            }
        }

        imu_processor.init(ICM42688_IRQ_PIN, gpio_callback);

        xTaskCreate(process_imu, "imu", 1024, NULL, 3, NULL);
        xTaskCreate(process_range, "rf", 1024, NULL, 1, NULL);
        xTaskCreate(process_cmd, "echo", 1024, NULL, 1, NULL);

        // enabling the data ready interrupt
        vTaskStartScheduler();
    }
    while (true) {
        printf("fail!!!\n");
    }
}