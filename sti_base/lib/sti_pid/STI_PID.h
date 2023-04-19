#ifndef STI_PID_H
#define STI_PID_H

#include "Arduino.h"

class PID
{
    public:
        PID(float min_val, float max_val, float kp, float ki, float kd);
        double compute(float setpoint, float measured_value, bool enb_pid);
        void updateConstants(float kp, float ki, float kd);

    private:
        float min_val_;
        float max_val_;
        float kp_;
        float ki_;
        float kd_;
        double integral_;
        double derivative_;
        double prev_error_;
		double prev_pid_;
};

#endif
