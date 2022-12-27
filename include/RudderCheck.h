#pragma once

int ReadRudder();
void MoveRudder();
int GetSerialAngle();

float PID_Proportional(float present_error);
float PID_Integral(float present_error);
void PID_rudder_control(int rudder_angle_desired);