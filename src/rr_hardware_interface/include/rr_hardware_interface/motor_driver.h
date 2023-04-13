#pragma once

class MotorDriver
{
public:
    virtual bool ReadState() = 0;
    virtual bool SendCommand() = 0;
};