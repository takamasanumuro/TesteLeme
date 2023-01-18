#pragma once
#include "mavlink/mavlink.h"
#define PRINT_THROTTLE
enum MAVLink_options {
  rudder_pwm = 0,
  rudder_angle = 1,
  sail_pwm = 2,
  sail_angle = 3,
};

enum pixHawkChannelsEnum {
  rudder = 0, 
  throttle = 1,
  sail = 2, 
};

static constexpr uint8_t arduino_sys_id = 2;
static constexpr uint8_t arduino_comp_id = 1;
static constexpr uint8_t pixhawk_sys_id = 1;
static constexpr uint8_t pixhawk_comp_id = 1;
int ReadRudder();
void MoveRudder();
int GetSerialAngle();
void GetAllPixhawkReadings();
int16_t GetPixhawkReadings(pixHawkChannelsEnum pixhawk_channel);
int16_t GetAngleFromReading(pixHawkChannelsEnum pixhawk_channel);
void PrintRadioAngle(uint32_t print_delay, int16_t pixhawk_channel);
float PID_Proportional(float present_error);
float PID_Integral(float present_error);
void PID_rudder_control(int rudder_angle_desired);
void ThrottleControl(int16_t throttle_signal);
void MAVLinkToPixhawk(MAVLink_options option, float data);
void TestAnalogWrite(uint8_t pwm_test_pin);

