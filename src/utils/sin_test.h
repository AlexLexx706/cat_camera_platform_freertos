#ifndef __SIN_TEST_H_
#define __SIN_TEST_H_
#include "pico/time.h"
#include <float.h>
#include <math.h>
#include <stdint.h>

class SinTets {
    uint32_t start_time = 0;
    bool active = false;
    float period = 10;
    float amplitude = 90.;
    float value = 0.;
    float max_ampliture = FLT_MAX;

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

    void set_ampliture(float _amplitude) {
        if (_amplitude > max_ampliture) {
            _amplitude = max_ampliture;
        }
        amplitude = _amplitude;
    }
    float get_ampliture() const { return amplitude; }

    void set_max_ampliture(float max) {
        if (max < 0.)
            return;

        max_ampliture = max;
        if (amplitude > max_ampliture) {
            amplitude = max_ampliture;
        }
    }
    float get_max_ampliture() const { return max_ampliture; }

    float get_value() const {
        if (!active) {
            return 0.;
        }
        return sin((time_us_32() - start_time) / 1e6 / period * M_PI) *
               amplitude;
    }
};

#endif // __SIN_TEST_H_