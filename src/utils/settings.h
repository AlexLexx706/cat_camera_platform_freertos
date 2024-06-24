#ifndef _SETTINGS_CAT_H_
#define _SETTINGS_CAT_H_
#include "Vector3d.h"
// what need to store:
// 1. gyro bias calibration (3 float)
// 2. gyro rate scale factor calibration (3 float)
// 3. encoders calibration (2 float)
// 4. rate PID controller settings (3 float)
// 5. speed PID controller settings (3 float)
// 6. vertical servo PID controller settings (3 float)
// 4. geometry settings (4 float base)
// max stored bytes: 4096 bytes

struct GyroBiasSettings {
    Vector3D bias;
};

struct GyroScaleSettings {
    Vector3D scale;
};

struct EncoderSettings {
    float scale[3];
};

struct PidSettings {
    float p;
    float i;
    float d;
};

struct GeometrySettings {
    float base;
    float debug[3];
};

struct Settings {
    GyroBiasSettings gyro_bias_settings;
    GyroScaleSettings gyro_scale_settings;
    EncoderSettings encoder_settings;
    PidSettings rate_pid_settings;
    PidSettings speed_pid_settings;
    PidSettings servo_pid_settings;
    GeometrySettings geometry_settings;
};

extern Settings *settings;

#endif //_SETTINGS_CAT_H_