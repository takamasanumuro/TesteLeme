#include <Arduino.h>
#include "UN178Driver.hpp"
#include "RudderCheck.h"              
                                                                                                                                                                                                                                                                                                                                                                                            #include "DualVNH5019MotorShield.h" //drive motor
UN178Driver hBridge = UN178Driver(2, 3, 9, 7, 8, 10); // Initilizes those pins as outputs

constexpr uint8_t rudderPinPot = A0;
// Following constants were taken using a multimeter after setting the potentiometer to mid (5k) resistance and aligning the rudder to the center of the boat
constexpr int16_t rudderAngleOffset = -2;  // To align the rudder to the bow of the boat
constexpr int16_t rudderADCMinThreshold = 365;  // Rudder -37 degrees (Starboard)
constexpr int16_t rudderADCMaxThreshold = 796;  // Rudder 57 degrees (Port)
// Fix zero offset issue later
constexpr int16_t rudderMinAngle = -37; // Rudder -37 degrees (Starboard) 
constexpr int16_t rudderMaxAngle = 57;  // Rudder 57 degrees (Port)
constexpr int16_t rudderMaxPWM = 360;  
constexpr uint16_t rudderLowPixhawk = 993;
constexpr uint16_t rudderHighPixhawk = 1986;
constexpr uint8_t numberPixhawkOutputs = 2;
constexpr uint8_t pixHawkReadingPins[numberPixhawkOutputs] = {6, 5}; // Arduino pins that receive pixhawk output PWM to read with pulseIn() function
int16_t pixHawkReadingsPWM[numberPixhawkOutputs] = {1500}; // Array that stores the PWM readings from Pixhawk. Default initialized to middle servo trim of 1500us.

constexpr float proportionalConstant = 8.f;
constexpr float integralConstant = 3.f;
float integralSum = 0.0f;
uint32_t rudderBeginTime, rudderEndTime;
static int rudder_angle_test = 0;
static int rudder_output_pwm_test = 0;


void setup() {
  Serial.begin(115200);  // For PC communications
  Serial3.begin(115200);  // For Pixhawk communications  
  Serial.println("Boat Rudder Test");
  hBridge.init(); //
  pinMode(rudderPinPot, INPUT); // Rudder potentiometer
  for (auto &pixHawkReadingPin : pixHawkReadingPins) pinMode(pixHawkReadingPin, INPUT); // Pixhawk output PWM input to read with pulseIn() function
  rudderBeginTime = millis();
}

void loop() {
  GetAllPixhawkReadings();
  PID_rudder_control(GetAngleFromReading(rudder));
  ThrottleControl(GetPixhawkReadings(throttle));
  TestAnalogWrite(12);
  //PrintRadioAngle(3000, rudder);
  //MAVLinkToPixhawk(rudder_angle, rudder_angle_test);
  //MAVLinkToPixhawk(rudder_pwm, rudder_output_pwm_test); 
}


int ReadRudder() {
  int pot_rudder_ADC = analogRead(rudderPinPot);
  int pot_rudder_angle = map(pot_rudder_ADC, rudderADCMinThreshold, rudderADCMaxThreshold, rudderMinAngle, rudderMaxAngle);
  pot_rudder_angle += rudderAngleOffset;
  #ifdef PRINT_RUDDER
  static uint32_t rudder_read_timer = millis();
  if (millis() - rudder_read_timer < 3000) return pot_rudder_angle;
  rudder_read_timer = millis();
  Serial.print("Rudder ADC: ");   Serial.println(pot_rudder_ADC);
  Serial.print("Rudder angle: "); Serial.println(pot_rudder_angle);
  #endif
  /*Delete later*/ rudder_angle_test = pot_rudder_angle; /*Delete later*/
  return pot_rudder_angle;
}


int GetSerialAngle() {
  static char input_byte = '\0';
  static bool has_received_angle = false;
  static bool is_angle_negative = false;
  static int16_t angle_received_previous = 0;
  int angle_received = 0;
  
  while (Serial.available() > 0 && !has_received_angle) {
    delayMicroseconds(500);
    input_byte = Serial.read();
    Serial.print(input_byte);
    if (input_byte < '0' || input_byte > '9') {
      if ( input_byte == '-') {
        is_angle_negative = true;
        continue;
      }
      while (Serial.available() > 0) Serial.read();
      has_received_angle = true;
      angle_received_previous = angle_received;
      if (is_angle_negative) {
        angle_received_previous = -angle_received_previous;
        is_angle_negative = false;
      }
      break;
    }
    angle_received = angle_received*10 + input_byte - '0';
  }
  auto angle_received_previous_copy = angle_received_previous;
  angle_received_previous = constrain(angle_received_previous, rudderMinAngle, rudderMaxAngle);
  if (has_received_angle) {
    has_received_angle = false;
    Serial.print("Received angle: "); Serial.println(angle_received_previous_copy);
    Serial.print("Received angle constrained: "); Serial.println(angle_received_previous);
  }
  return angle_received_previous;
}

void GetAllPixhawkReadings() {
  for (int i = 0; i < numberPixhawkOutputs; i++) {
    pixHawkReadingsPWM[i] = pulseIn(pixHawkReadingPins[i], HIGH);
  }
}

int16_t GetPixhawkReadings(pixHawkChannelsEnum pixhawk_channel) {
  for (int i = 0; i < numberPixhawkOutputs; i++) {
    pixHawkReadingsPWM[i] = pulseIn(pixHawkReadingPins[i], HIGH);
  }
  return pixHawkReadingsPWM[pixhawk_channel];
}

int16_t GetAngleFromReading(pixHawkChannelsEnum pixhawk_channel) {
  pixHawkReadingsPWM[pixhawk_channel] = map(pixHawkReadingsPWM[pixhawk_channel], rudderLowPixhawk, rudderHighPixhawk, rudderMinAngle+10, rudderMaxAngle-10);
  return pixHawkReadingsPWM[pixhawk_channel];
}

void ThrottleControl(int16_t throttle_signal) {
  static constexpr int16_t throttle_buffer = 30;
  static constexpr int16_t throttle_trim = 1500;
  throttle_signal = constrain(throttle_signal, 1000, 2000);
  if (throttle_signal > throttle_trim + throttle_buffer) {
    map(throttle_signal, throttle_trim + throttle_buffer, 2000, 0, 380);
    hBridge.setM1PWM(throttle_signal);
  }
  else if (throttle_signal < throttle_trim - throttle_buffer) {
    map(throttle_signal, 1000, throttle_trim - throttle_buffer, 380, 0);
    throttle_signal = -throttle_signal;
    hBridge.setM1PWM(throttle_signal);
  }
  else {
    hBridge.setM1PWM(0);
  }
  #ifdef PRINT_THROTTLE
  static uint32_t throttle_read_timer = millis();
  if (millis() - throttle_read_timer < 1000) return;
  throttle_read_timer = millis();
  Serial.print("Throttle signal: "); Serial.println(throttle_signal);
  #endif
}

void PrintRadioAngle(uint32_t print_delay, int16_t pixhawk_channel) {
  static uint32_t print_timer = millis();
  if (millis() - print_timer < print_delay) return;
  print_timer = millis();
  switch(pixhawk_channel) {
    case rudder:
      Serial.print("Rudder angle: "); Serial.println(pixHawkReadingsPWM[pixhawk_channel]);
      break;
    case sail:
      Serial.print("Sail angle: "); Serial.println(pixHawkReadingsPWM[pixhawk_channel]);
      break;
  }
}

float PID_Proportional(float present_error) {
  return proportionalConstant*present_error;
}

float PID_Integral(float present_error) {
  rudderEndTime = millis();
  float cycle_time = static_cast<float>(rudderEndTime - rudderBeginTime)/1000.f;
  rudderBeginTime = millis();
  integralSum = integralSum + integralConstant*present_error*cycle_time;
  integralSum = constrain(integralSum, -rudderMaxPWM, rudderMaxPWM);
  return integralSum;
}

void PID_rudder_control(int rudder_angle_desired){  
  //Checks current rudder angle
  int rudder_angle_present = ReadRudder();
  //Calculates error
  int rudder_error = rudder_angle_desired - rudder_angle_present;
  //Calculates PID output
  int rudder_output_pwm = PID_Proportional(rudder_error) + PID_Integral(rudder_error);
  // Constrains output given H-bridge PWM output range,
  rudder_output_pwm = constrain(rudder_output_pwm, -rudderMaxPWM, rudderMaxPWM);
  rudder_output_pwm = -rudder_output_pwm; // Flips the sign of the output to match the rudder's direction
  /*Delete later*/ rudder_output_pwm_test = rudder_output_pwm; /*Delete later*/
  if (rudder_error > -2 && rudder_error < 2) rudder_output_pwm = 0;
  //hBridge.setM1PWM(rudder_output_pwm);
  hBridge.setM2PWM(rudder_output_pwm);  //-400 <-> +400
  static uint32_t rudder_log_timer = millis();
  if (millis() - rudder_log_timer < 500) return;
  rudder_log_timer = millis();
  Serial.print("\nRudder desired: "); Serial.println(rudder_angle_desired);
  Serial.print("Rudder present: "); Serial.println(rudder_angle_present);
  Serial.print("Rudder error: "); Serial.println(rudder_error);
  Serial.print("Rudder PWM: "); Serial.println(rudder_output_pwm);
}

//envia dados para a pixhawk usando o protocolo MAVLINK
void MAVLinkToPixhawk(MAVLink_options option, float data) {
  // Set up timer to publish at 1 Hz
  static uint32_t MAV_publish_timer = 0;
  static constexpr uint32_t MAV_publish_delay = 1000;
  if (millis() - MAV_publish_timer < MAV_publish_delay) return;
  MAV_publish_timer = millis();
   // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buffer[MAVLINK_MAX_PACKET_LEN];

  switch (option) { 
    case rudder_angle:
    {
      mavlink_msg_param_set_pack(arduino_sys_id, arduino_comp_id, &msg, pixhawk_sys_id, pixhawk_comp_id, "RUDDER_ANGLE", data, MAV_VAR_FLOAT);
      Serial.print("MAV: "); Serial.println(data);
    }
    break;

    case sail_angle:
    {
      mavlink_msg_param_set_pack(arduino_sys_id, arduino_comp_id, &msg, pixhawk_sys_id, pixhawk_comp_id,"SAIL_ANGLE", data, MAV_VAR_FLOAT);
      Serial.print("MAV: "); Serial.println(data);
    }
    break;

    case rudder_pwm:
    {
      mavlink_msg_param_set_pack(arduino_sys_id, arduino_comp_id, &msg, pixhawk_sys_id, pixhawk_comp_id,"RUDDER_PWM", data, MAV_VAR_FLOAT);
      Serial.print("MAV: "); Serial.println(data);
    }
    break;

    case sail_pwm:
    {
      mavlink_msg_param_set_pack(arduino_sys_id, arduino_comp_id, &msg, pixhawk_sys_id, pixhawk_comp_id,"SAIL_PWM", data, MAV_VAR_FLOAT);
      Serial.print("MAV: "); Serial.println(data);
    }
    break;
  }  
 
  // Copy the message to the send buffer
  uint16_t length = mavlink_msg_to_send_buffer(buffer, &msg);
  // Send the message with the standard UART send function
  Serial3.write(buffer, length);
}


void TestAnalogWrite(uint8_t pwm_test_pin) {
  static uint32_t pwm_test_timer = millis();
  static int16_t pwm_test_pwm = 0;
  static bool is_rising = true;
  if (millis() - pwm_test_timer < 80) return;
  pwm_test_timer = millis();
  if (is_rising) {
    pwm_test_pwm += 10;
    if (pwm_test_pwm > 255) is_rising = false;
  }
  else {
    pwm_test_pwm -= 10;
    if (pwm_test_pwm < 0) is_rising = true;
  }
  constrain(pwm_test_pwm, 0, 255);
  analogWrite(pwm_test_pin, pwm_test_pwm);
}







