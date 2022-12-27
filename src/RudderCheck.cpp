#include <Arduino.h>
#include <DualVNH5019MotorShield.h>
#include "RudderCheck.h"
                                                                                                                                                                                                                                                                                                                                                                                                           #include "DualVNH5019MotorShield.h" //drive motor
DualVNH5019MotorShield hBridge; // Controls the rudder and the winch

constexpr uint8_t rudderPin = A2;
// Following constants were taken using a multimeter after setting the potentiometer to mid (5k) resistance and aligning the rudder to the center of the boat
constexpr int16_t rudderAngleOffset = -2;  // To align the rudder to the bow of the boat
constexpr int16_t rudderADCMinThreshold = 365;  // Rudder -37 degrees (Starboard)
constexpr int16_t rudderADCMaxThreshold = 796;  // Rudder 57 degrees (Port)
constexpr int16_t rudderMinAngle = -37; // Rudder -37 degrees (Starboard)
constexpr int16_t rudderMaxAngle = 57;  // Rudder 57 degrees (Port)
constexpr int16_t rudderMaxPWM = 400;  

constexpr float proportionalConstant = 5.f;
constexpr float integralConstant = 2.5f;
float integralSum = 0.0f;
uint32_t rudderBeginTime, rudderEndTime;

void setup() {
  Serial.begin(115200);
  Serial.println("Boat Rudder Test");
  hBridge.init(); //
  pinMode(rudderPin, INPUT);
  rudderBeginTime = millis();
}

void loop() {
  //ReadRudder();
  //MoveRudder();
  //GetSerialAngle();
  PID_rudder_control(GetSerialAngle());
  
}

int ReadRudder() {
  int pot_rudder_ADC = analogRead(rudderPin);
  int pot_rudder_angle = map(pot_rudder_ADC, rudderADCMinThreshold, rudderADCMaxThreshold, rudderMinAngle, rudderMaxAngle);
  pot_rudder_angle += rudderAngleOffset;
  static uint32_t rudder_read_timer=millis();
  if (millis() - rudder_read_timer < 3000) return pot_rudder_angle;
  rudder_read_timer = millis();
  Serial.print("Rudder ADC: ");   Serial.println(pot_rudder_ADC);
  Serial.print("Rudder angle: "); Serial.println(pot_rudder_angle);
  return pot_rudder_angle;
}

void MoveRudder() {
  hBridge.setM2Speed(400);
  delay(1500);
  hBridge.setM2Speed(0);
  delay(1500);

  hBridge.setM2Speed(-400);
  delay(1500);
  hBridge.setM2Speed(0);
  delay(1500);
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
  if (rudder_error > -2 && rudder_error < 2) rudder_output_pwm = 0;
  hBridge.setM2Speed(rudder_output_pwm);  //-400 <-> +400
  static uint32_t rudder_log_timer = millis();
  if (millis() - rudder_log_timer < 3000) return;
  rudder_log_timer = millis();
  Serial.print("Rudder error: "); Serial.println(rudder_error);
  Serial.print("Rudder PWM: "); Serial.println(rudder_output_pwm);
}










