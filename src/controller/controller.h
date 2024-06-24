#ifndef _CONTROLLER_H_
#define _CONTROLLER_H_
#include "FreeRTOS.h"
#include <stdint.h>
#include <task.h>

class Controller {
    TaskHandle_t task;
    uint16_t max_pwm_value = 16384;
    uint16_t min_pwm_value = 6000;
    uint16_t max_control_value = max_pwm_value - min_pwm_value;
    uint int1;
    uint int2;
    uint int3;
    uint int4;
    uint en1;
    uint en2;

    void process();
    static void thread_handler(void *);

 public:
    bool init(uint int1, uint int2, uint int3, uint int4, uint en1, uint en2,
              int task_prio, int stack_size);
};

extern Controller controller;
#endif //_CONTROLLER_H_