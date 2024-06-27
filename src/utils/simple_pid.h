#ifndef ALEX_PID_H
#define ALEX_PID_H
#include <math.h>

// Simple PID controller
struct SimplePID {
    float p;
    float i;
    float d;
    float old_error;
    float int_value;
    char first;
    float max_int;
	float d_value;
	float p_value;

 public:
    SimplePID(float _p, float _i, float _d, float _max_int):
		p(_p), i(_i), d(_d), old_error(0.f), first(1), max_int(_max_int) {}

    void reset() {
        int_value = 0.f;
        first = 1;
    }

    float compute(float current, float target) {
        // 9. process PID controller for rate
        float error = target - current;
        p_value = error * p;

        if (first == 0) {
            d_value = (error - old_error) * d;
        } else {
            d_value = 0.f;
            first = 0;
        }
        old_error = error;

        int_value += error * i;

        if (fabs(int_value) > max_int) {
            int_value = int_value > 0 ? max_int : -max_int;
        }
        return p_value + int_value + d_value;
    }
};

#endif
