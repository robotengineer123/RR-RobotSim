#pragma once
#include <functional>

namespace WinchPlugin
{
    class PID
    {
    public:
        PID(){};
        PID(double kp, double kd, double ki, std::function<double()> sensor_callback)
            : kp(kp), kd(kd), ki(ki), SensorCallback(sensor_callback) {}
        void SetSensorCallback(std::function<double()> sensor_callback) { SensorCallback = sensor_callback; }
        double Compute(double set_point);

        double kp;
        double kd;
        double ki;
        
        double integral = 0;

    private:
        double prev_error_ = 0;

        std::function<double()> SensorCallback;
    };
}