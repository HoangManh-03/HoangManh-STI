#include<testMotionPlanning/pidControl.h>

namespace GoalControl{
    PID::PID(double p, double i, double d)
        : kp(p), ki(i), kd(d), integral(0), previous_error(0) {}
    
    double PID::calculate(double error) {
        
        integral += error;
        double derivative = error - previous_error;
        previous_error = error;
        double output = kp * error + ki * integral + kd * derivative;
        return output;
    }
}
