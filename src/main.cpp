#include "FreeRTOS.h"
#include "VL53L0X/VL53L0X.h"
#include "hardware/gpio.h"
#include "imu_processor/imu_porcessor.h"
#include "encoder/encoder.h"
#include "command_processor/command_processor.h"
#include "controller/controller.h"
#include "pico/binary_info.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include "semphr.h"
#include "task.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "controller/stepper_motor_controller.h"

#define ENC_A1 8
#define ENC_B1 9

#define ENC_A2 10
#define ENC_B2 11

#define ENC_A3 16
#define ENC_B3 17

#define ICM42688_IRQ_PIN 22

#define SDA_PIN 6
#define SCL_PIN 7

#define INT1 2
#define INT2 3

#define INT3 4
#define INT4 5

#define INT5 14
#define INT6 15

#define  EN1 0
#define  EN2 1


#define DIR_PIN 16
#define STEP_PIN 15
#define EN_PIN 14

#define BUTTON_END_PIN 26
#define BUTTON_START_PIN 27

#define UART_ID uart0
#define BAUD_RATE 576000

// We are using pins 0 and 1, but see the GPIO function select table in the
// datasheet for information on which other pins can be used.
#define UART_TX_PIN 12
#define UART_RX_PIN 13

static VL53L0X sensor;
volatile static uint32_t packet_time;
volatile static uint16_t rf_range = 0;

static SemaphoreHandle_t imu_data_semaphore;

IMUProcessor imu_processor;
Encoder encoder;
Controller controller;
StepperMotorController stepper_motor_controller;


static void gpio_callback(uint gpio, uint32_t events) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    // imu Interrupt pin
    if (gpio == ICM42688_IRQ_PIN) {
        imu_processor.irq_handler(xHigherPriorityTaskWoken);
    // process encoder
    } else {
        encoder.irq_handler(gpio);
    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
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

    int encoder_pins [] = {
        ENC_A1, ENC_B1,
        ENC_A2, ENC_B2,
        ENC_A3, ENC_B3};

    //set gpio irq callback
    if (imu_processor.init(ICM42688_IRQ_PIN, 4, 1024) &&
            encoder.init(encoder_pins, 3, 1024) &&
            controller.init(INT1, INT2, INT3, INT4, EN1, EN2, INT5, INT6, 2, 1024) &&
            stepper_motor_controller.init(DIR_PIN, STEP_PIN, EN_PIN, BUTTON_START_PIN, BUTTON_END_PIN, 2, 1024)) {
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

        gpio_set_irq_callback(gpio_callback);
        irq_set_enabled(IO_IRQ_BANK0, true);

        if (!sensor.init()) {
            printf("Failed to detect and initialize sensor!\n");
            while (1) {
                printf("Failed to detect and initialize sensor!\n");
                sleep_ms(1000);
            }
        }

        xTaskCreate(process_range, "rf", 1024, NULL, 1, NULL);
        xTaskCreate(process_cmd, "echo", 1024, NULL, 1, NULL);

        // enabling the data ready interrupt
        vTaskStartScheduler();
    }
    while (true) {
        printf("fail!!!\n");
    }
}