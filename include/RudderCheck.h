#pragma once

int ReadRudder();
void MoveRudder();
int GetSerialAngle();
int GetRadioAngle(int16_t pixhawk_channel);
void PrintRadioAngle(uint32_t print_delay, int16_t pixhawk_channel);

float PID_Proportional(float present_error);
float PID_Integral(float present_error);
void PID_rudder_control(int rudder_angle_desired);