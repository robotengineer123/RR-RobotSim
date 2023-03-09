#include "winch_force_plugin/pid.h"
namespace WinchPlugin
{
    double PID::Compute(double set_point)
    {
        double sensor = SensorCallback();

        double error = set_point - sensor;
        double diff_error = error - prev_error_;
        integral += error;
        prev_error_ = error;

        double output = kp*error + kd*diff_error + ki*integral;
        return output;
    }
}