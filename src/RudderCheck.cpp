#include <Arduino.h>
#include <DualVNH5019MotorShield.h>
#include "RudderCheck.h"
                                                                                                                                                                                                                                                                                                                                                                                                           #include "DualVNH5019MotorShield.h" //drive motor
DualVNH5019MotorShield md;
// Following constantes were taken using a multimeter
constexpr int rudderOffsetADC= -13;
constexpr int rudderMinThreshold=346;
constexpr int rudderMaxThreshold=740;
constexpr int rudderMaxAngle=45;

void setup() {
  md.init();
  Serial.begin(9600);
}

void loop() {

  ReadRudder();
  MoveRudder();
}

void ReadRudder(){
  int pot_rudder_ADC = analogRead(A2)-rudderOffsetADC;
  int pot_rudder_angle=map(pot_rudder_ADC,rudderMinThreshold,rudderMaxThreshold,-rudderMaxAngle,rudderMaxAngle);
  Serial.print("Rudder ADC: ");  Serial.println(pot_rudder_ADC);
  Serial.print("Rudder angle: ");Serial.println(pot_rudder_angle);
}

void MoveRudder(){
  md.setM2Speed(400);
  delay(1500);
  md.setM2Speed(0);
  delay(1500);

  md.setM2Speed(-400);
  delay(1500);
  md.setM2Speed(0);
  delay(1500);

}