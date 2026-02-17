#pragma once
#include "mbed.h"

class SR04 {
public:
    SR04(PinName trig, PinName echo);
    void trigger();
    bool update();      // trueなら測定完了
    float get_cm();

private:
    DigitalOut _trig;
    DigitalIn  _echo;
    Timer _timer;
    uint32_t _t_start, _t_end;
    bool _measuring;
};
