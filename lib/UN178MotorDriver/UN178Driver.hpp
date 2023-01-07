#pragma once
#include <Arduino.h>
class UN178Driver {
public:
    UN178Driver();
    UN178Driver(unsigned char INA1,
                unsigned char INB1,
                unsigned char PWM1,
                unsigned char INA2,
                unsigned char INB2,
                unsigned char PWM2);
    void init();
    void setM1PWM(int pwm);
    void setM2PWM(int pwm);

private:
    unsigned char _INA1;
    unsigned char _INB1;
    unsigned char _PWM1;
    unsigned char _INA2;
    unsigned char _INB2;
    unsigned char _PWM2;                      
};