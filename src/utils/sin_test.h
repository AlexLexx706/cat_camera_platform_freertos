#ifndef __SIN_TEST_H_
#define __SIN_TEST_H_
#include <math.h>
#include "pico/time.h"
#include <stdint.h>

class SinTets {
    uint32_t start_time = 0;
    bool active = false;
    float period = 10;
    float amplitude = 90.;
    float value = 0.;

    public:
    void set_active(bool _active) {
        if (_active != active && _active) {
            start_time = time_us_32();
        }
        active = _active;
    }
    bool is_active() const { return active; }

    void set_period(float _period) { period = _period; }
    float get_period() const { return period; }

    void set_ampliture(float _amplitude) { amplitude = _amplitude; }
    float get_ampliture() const { return amplitude; }

    float get_value() const {
        if (!active) {
            return 0.;
        }
        return sin((time_us_32() - start_time) / 1e6 / period * M_PI) * amplitude;
    }
};

#endif // __SIN_TEST_H_