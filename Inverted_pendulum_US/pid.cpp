#include "pid.h"

#define DIV_P 1000.0
#define DIV_I 1000.0
#define DIV_D 1000.0

Pid::Pid(float kp, float ki, float kd){
    _kp = kp/DIV_P;
    _ki = ki/DIV_I;
    _kd = kd/DIV_D;
    _sum_errors = 0;
    _previous_error = 0;
    _correction = 0;
}

void Pid::setKp(float kp){
    _kp = kp/DIV_P;
}

void Pid::setKi(float ki){
    _ki = ki/DIV_I;
}

void Pid::setKd(float kd){
    _kd = kd/DIV_D;
}


int16_t Pid::update(int16_t target, int16_t state){
    float error = target - state ;
    _sum_errors += error;
    _correction = _kp * error + _ki * _sum_errors + _kd * (error - _previous_error);
    _previous_error = error;
    return (int16_t) _correction;
}

void Pid::reset(){
    _sum_errors = 0;
    _previous_error = 0;
    _correction = 0;
}

void Pid::reset_I(){
    _sum_errors = 0;
}

void Pid::reset(float kp, float ki, float kd){
    _kp = kp/DIV_P;
    _ki = ki/DIV_I;
    _kd = kd/DIV_D;
    reset();
}
