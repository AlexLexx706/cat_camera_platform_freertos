add_executable(${NAME}
        ICM42688/ICM42688.cpp
        VL53L0X/VL53L0X.cpp
        imu_processor/imu_processor.cpp
        commad_processor/command_processor.cpp
        commad_processor/motor_controller.cpp
        flash_store/flash_store.cpp
        main.cpp
        encoder/encoder.cpp
        controller/controller.cpp
        controller/motor_controller.cpp
        controller/stepper_motor_controller.cpp
        )

# Pull in our pico_stdlib which pulls in commonly used features
target_link_libraries(${NAME} 
	 pico_stdlib
     FreeRTOS-Kernel-Heap4
     freertos_config
     hardware_spi
     hardware_i2c
     hardware_pwm
     hardware_flash
	)
	
target_include_directories(${NAME} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
     )
     
target_compile_definitions(${NAME} PRIVATE
    configNUM_CORES=2
)

# create map/bin/hex file etc.
pico_add_extra_outputs(${NAME})

# enable usb output, disable uart output
pico_enable_stdio_usb(${NAME} 1)
pico_enable_stdio_uart(${NAME} 1)
target_compile_definitions(${NAME} PRIVATE
    PICO_DEFAULT_UART_RX_PIN=13
    PICO_DEFAULT_UART_TX_PIN=12
    PICO_DEFAULT_UART_BAUD_RATE=576000
)