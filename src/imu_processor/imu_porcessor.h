#ifndef IMU_PORCESSOR_H
#define IMU_PORCESSOR_H
#include "FreeRTOS.h"
#include "ICM42688/ICM42688.h"
#include "hardware/gpio.h"
#include "semphr.h"
#include "task.h"
#include "utils/Vector3d.h"

class IMUProcessor {

    ICM42688 imu;
    uint32_t last_packet_time_us = 0;
    bool allow_dynamic_bias = true;
    float bias_rate_threshold = 0.1f;
    float gyro_bias_tau = 20.f;
    int debug_level = 0;
    uint32_t last_print_time_us = 0;
    uint32_t print_period_us = 100000;

    uint32_t speed_update_period_us = 20000;
    uint32_t last_encoder_time_us = 0;
    float last_encoder_pos[2];
    float encoder_speed_ms[2] = {0.f};
    float mid_encoder_speed_ms = 0.f;
    bool init_encoder = true;
    float wheel_base = 0.235f;
    float encoder_omega = 0.f;
    float encoder_heading = 0.f;
    Vector3D encoder_pos = {0.f};


    Vector3D pos = {0.f};

    enum State { Ide = 0, BiasClb, ScaleClb } state = Ide;
    Vector3D gyro_rate = {0.f};
    Vector3D angles = {0.f};
    Vector3D gyro_bias = {0.f};
    Vector3D internal_gyro_bias = {0.f};
    Vector3D gyro_scale = {1.f, 1.f, 1.f};

    uint32_t packet_time_us;
    SemaphoreHandle_t packet_semaphore;
    TaskHandle_t task;

    static void task_handler(void *);
    void process();

 public:
    bool init(int irq_pin, gpio_irq_callback_t gpio_irq_callback, int task_prio,
              int stack_size);
    void irq_handler(BaseType_t &xHigherPriorityTaskWoken);

    void start_bias_calibration();
    bool stop_bias_calibration();
    bool is_bias_clb_on() const { return state == BiasClb; }
    void set_bias(const Vector3D &bias) { gyro_bias = bias; }
    const Vector3D &get_bias() const { return gyro_bias; }
    void set_gyro_bias_tau(float tau) { gyro_bias_tau = tau; }
    float get_gyro_bias_tau() const { return gyro_bias_tau; }
    void set_debug_level(int _debug) { debug_level = _debug; }
    int get_debug_level() const { return debug_level; }
    void set_angles(const Vector3D &_angles) { angles = _angles; }
    const Vector3D &get_angles() const { return angles; }
    const Vector3D &get_pos() const { return pos; }
    void set_pos(const Vector3D &_pos) { pos = _pos; }

    void set_wheel_base(float base) {wheel_base = base;}
    float get_wheel_base() const {return wheel_base;}

    float get_encoder_omega() const {return encoder_omega;}
    void set_encoder_heading(float heading) {encoder_heading = heading;}
    float get_encoder_heading() const {return encoder_heading;}

    void set_encoder_pos(const Vector3D & _pos) {encoder_pos = _pos;}
    const Vector3D & get_encoder_pos() const {return encoder_pos;}

    float get_speed() const {return mid_encoder_speed_ms;}
};
extern IMUProcessor imu_processor;
#endif // IMU_PORCESSOR_H
