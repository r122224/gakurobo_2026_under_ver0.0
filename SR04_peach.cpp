#include "SR04_peach.h"

SR04::SR04(PinName trig, PinName echo)
    : _trig(trig), _echo(echo)
{
    _trig = 0;
    _measuring = false;
}

void SR04::trigger()
{
    _measuring = true;

    _trig = 1;
    wait_us(10);
    _trig = 0;

    _timer.reset();
    _timer.start();
}

bool SR04::update()
{
    if(!_measuring) return false;

    // echoが立ち上がるまで待つ
    if(_t_start == 0 && _echo.read() == 1){
        _t_start = _timer.read_us();
    }

    // echoが落ちたら終了
    if(_t_start != 0 && _echo.read() == 0){
        _t_end = _timer.read_us();
        _timer.stop();
        _measuring = false;
        return true;
    }
    return false;
}

float SR04::get_cm()
{
    uint32_t us = _t_end - _t_start;
    return us / 58.0f;
}
