#include <Arduino.h>
#include "UN178Driver.hpp"
#include "RudderCheck.hpp"
                                                                                                                                                                                                                                                                                                                                                 #include "DualVNH5019MotorShield.h" //drive motor
UN178Driver sternBridge = UN178Driver(2, 3, 4, 5, 6, 7); // Stern motor and rudder respectively - Digital/Digital/PWM order of pins
UN178Driver winchBridge = UN178Driver(8, 9, 10); // Winch motor - Digital/Digital/PWM order of pins
constexpr uint8_t sailPinPotentiometer = A0;
constexpr uint8_t rudderPinPotentiometer = A1;
// Following constants were taken using a multimeter after setting the potentiometer to mid (5k) resistance and aligning the rudder to the center of the boat
constexpr int16_t rudderAngleOffset = -2;  // To align the rudder to the bow of the boat
constexpr int16_t sailAngleOffset = 0;  // To align the rudder to the bow of the boat
constexpr int16_t rudderADCMinThreshold = 365;  // Rudder -37 degrees (Starboard)
constexpr int16_t rudderADCMaxThreshold = 796;  // Rudder 57 degrees (Port)
constexpr int16_t sailADCMinThreshold = 204;
constexpr int16_t sailADCMaxThreshold = 818; 

constexpr uint16_t pixhawkMinimalPWM = 993;
constexpr uint16_t pixhawkMaximumPWM = 1986;
constexpr int16_t rudderMinAngle = -37; // Rudder -37 degrees (Starboard) 
constexpr int16_t rudderMaxAngle = 57;  // Rudder 57 degrees (Port)
constexpr int16_t sailMinAngle = 10;
constexpr int16_t sailMaxAngle = 75;

constexpr float rudder_proportional_constant = 1.5f;
constexpr float rudder_integral_constant = 3.0f;
constexpr float sail_proportional_constant = 0.5f;
constexpr float sail_integral_constant = 0.5f;

constexpr uint8_t pixHawkReadingPins[] = {sailInputPWMPin, rudderInputPWMPin, throttleInputPWMPin}; // Arduino pins that receive pixhawk output PWM to read with pulseIn() function
constexpr uint8_t numberPixhawkPins = sizeof pixHawkReadingPins / sizeof pixHawkReadingPins[0]; // Get the number of elements in the array
int16_t pixHawkReadingsPWM[numberPixhawkPins] = {1500}; // Array that stores the PWM output in microseconds coming from Pixhawk. Default initialized to middle servo trim of 1500us.
enum PrintSelection {
  PrintRudder = 0,
  PrintSail = 1,
  PrintThrottle = 2
};
uint8_t printSelectionIndex = PrintRudder;

#define PRINT_RUDDER
#define PRINT_SAIL
#define PRINT_THROTTLE
//#define PRINT_RUDDER_READINGS
//#define PRINT_SAIL_READINGS

void setup() {
  Serial.begin(115200);  // For PC communications
  Serial3.begin(115200);  // For Pixhawk communications  
  Serial.print("Boat Rudder Test\n");
  sternBridge.init(); //
  winchBridge.init_channel_A();
  pinMode(rudderPinPotentiometer, INPUT); 
  pinMode(sailPinPotentiometer, INPUT);
  for (auto& pixHawkReadingPin : pixHawkReadingPins) pinMode(pixHawkReadingPin, INPUT); // Pixhawk output PWM input to read with pulseIn() function

}

void TestSelectionIndex() {
  static timer test_selection_timer = millis();
  if (millis() - test_selection_timer < 1000) return;
  test_selection_timer = millis();
  switch (printSelectionIndex) {
    case PrintRudder:
      Serial.println("RudderSelection");
      break;
    case PrintSail:
      Serial.println("SailSelection");
      break;
    case PrintThrottle:
      Serial.println("ThrottleSelection");
      break;
    default:
      Serial.println("InvalidSelection");
      break;
  }
}

void printPixhawkArray() {
  Serial.print("Input: ");
  for (auto& pixHawkReading : pixHawkReadingsPWM) {
    Serial.print(pixHawkReading); Serial.print("\t");
  }
  Serial.println();
}
void loop() {
  GetAllPixhawkReadings();
  RudderControl(GetPixhawkReadingToAngle(rudderIndex));
  SailControl(GetPixhawkReadingToAngle(sailIndex));
  ThrottleControl(GetPixhawkReading(throttleIndex));
  GetSerialInput();
}

float PID_Proportional(float present_error, float proportional_gain) {
  return proportional_gain*present_error;
}

float PID_Integral(float present_error, float integral_gain) {
  static int32_t integral_sum = 0;
  static timer end_time = millis(); 
  static timer begin_time = millis();
  end_time = millis();
  float cycle_time = static_cast<float>(end_time - begin_time) / 1000.f;
  begin_time = millis();
  integral_sum = integral_sum + integral_gain*present_error*cycle_time;
  integral_sum = constrain(integral_sum, -UN178Driver::maxPWM, UN178Driver::maxPWM); 
  return integral_sum;
}


void RudderControl(int rudder_angle_desired) {  
  int rudder_angle_present = ReadRudder();
  int rudder_error = rudder_angle_desired - rudder_angle_present;
  int rudder_output_pwm = PID_Proportional(rudder_error, rudder_proportional_constant) + PID_Integral(rudder_error, rudder_integral_constant);
  rudder_output_pwm = constrain(rudder_output_pwm, -UN178Driver::maxPWM, UN178Driver::maxPWM);// Constrains output given H-bridge PWM output range,
  rudder_output_pwm = -rudder_output_pwm; // Flips the sign of the output to match the rudder's direction
  if (rudder_error > -2 && rudder_error < 2) rudder_output_pwm = 0; //Prevents excessive microadjustments from the actuator
  sternBridge.setPWM(rudder_output_pwm, UN178Driver::M2);
  MAVLinkToPixhawk(MAVLink_options::rudder_pwm, rudder_output_pwm);
  

  #ifdef PRINT_RUDDER
  if (printSelectionIndex == PrintRudder) {
    static uint32_t rudder_log_timer = millis(); // This section logs information periodically to the serial port
    if (millis() - rudder_log_timer < 1500) return;
    rudder_log_timer = millis();
    Serial.print("\nRudder desired: "); Serial.println(rudder_angle_desired);
    Serial.print("Rudder present: "); Serial.println(rudder_angle_present);
    Serial.print("Rudder error: "); Serial.println(rudder_error);
    Serial.print("Rudder PWM: "); Serial.println(rudder_output_pwm); 
  }
  #endif
}

void SailControl(int sail_angle_desired) {
  int sail_adc_reading = ReadSailRaw();
  if (sail_adc_reading < 100 || sail_adc_reading > 900) {
    winchBridge.setPWM(0, UN178Driver::M1);
    Serial.println("SAIL CRITICAL");
    return;
  }
  int sail_angle_present = map(sail_adc_reading, sailADCMinThreshold, sailADCMaxThreshold, sailMaxAngle, sailMinAngle);
  int sail_error = sail_angle_desired - sail_angle_present;
  int sail_output_pwm = PID_Proportional(sail_error, sail_proportional_constant) + PID_Integral(sail_error, sail_integral_constant);
  sail_output_pwm = constrain(sail_output_pwm, -UN178Driver::maxPWM, UN178Driver::maxPWM);  // Constrains output given H-bridge PWM output range,
  sail_output_pwm = -sail_output_pwm; // Flips the sign of the output to match the sail's direction
  if (sail_error > -2 && sail_error < 2) sail_output_pwm = 0; //Prevents excessive microadjustments from the actuator
  winchBridge.setPWM(sail_output_pwm, UN178Driver::M1); 
  MAVLinkToPixhawk(MAVLink_options::sail_pwm, sail_output_pwm);
  
  #ifdef PRINT_SAIL
  if (printSelectionIndex == PrintSail) {
    static uint32_t sail_log_timer = millis(); // This section logs information periodically to the serial port
    if (millis() - sail_log_timer < 1500) return;
    sail_log_timer = millis();
    Serial.print("Sail ADC: "); Serial.println(sail_adc_reading);
    Serial.print("\nSail desired: "); Serial.println(sail_angle_desired);
    Serial.print("Sail present: "); Serial.println(sail_angle_present);
    Serial.print("Sail error: "); Serial.println(sail_error);
    Serial.print("Sail PWM: "); Serial.println(sail_output_pwm);
  }
  #endif
}

void ThrottleControl(int16_t throttle_signal) {
  static constexpr int16_t throttle_buffer = 30;
  static constexpr int16_t throttle_trim = 1500;
  throttle_signal = constrain(throttle_signal, 1000, 2000);
  if (throttle_signal > throttle_trim + throttle_buffer) {
    map(throttle_signal, throttle_trim + throttle_buffer, 2000, 0, UN178Driver::maxPWM);
    sternBridge.setPWM(throttle_signal, UN178Driver::M1);
  }
  else if (throttle_signal < throttle_trim - throttle_buffer) {
    map(throttle_signal, 1000, throttle_trim - throttle_buffer, UN178Driver::maxPWM, 0);
    throttle_signal = -throttle_signal;
    sternBridge.setPWM(throttle_signal, UN178Driver::M1);
  }
  else {
    sternBridge.setPWM(0, UN178Driver::M1);
  }

  #ifdef PRINT_THROTTLE
  if (printSelectionIndex == PrintThrottle) {
    static uint32_t throttle_read_timer = millis();
    if (millis() - throttle_read_timer < 1500) return;
    throttle_read_timer = millis();
    Serial.print("Throttle signal: "); Serial.println(throttle_signal);
  }
  #endif 
}

int ReadRudder() {
  int pot_rudder_ADC = analogRead(rudderPinPotentiometer);
  int pot_rudder_angle = map(pot_rudder_ADC, rudderADCMinThreshold, rudderADCMaxThreshold, rudderMinAngle, rudderMaxAngle);
  pot_rudder_angle += rudderAngleOffset;
  #ifdef PRINT_RUDDER_READINGS
  static uint32_t rudder_read_timer = millis();
  if (millis() - rudder_read_timer < 3000) return pot_rudder_angle;
  rudder_read_timer = millis();
  Serial.print("Rudder ADC: ");   Serial.println(pot_rudder_ADC);
  Serial.print("Rudder angle: "); Serial.println(pot_rudder_angle);
  #endif
  return pot_rudder_angle;
}

//2V correspondendo a vela em 90 graus/ 4V correspondendo a vela em 0 graus
int ReadSail() {
  int pot_sail_ADC = analogRead(sailPinPotentiometer); //2V aqui        //4V Aqui            2V-->90 graus  4V-->10 graus
  int pot_sail_angle = map(pot_sail_ADC, sailADCMinThreshold, sailADCMaxThreshold, sailMaxAngle, sailMinAngle);
  pot_sail_angle += sailAngleOffset;
  #ifdef PRINT_SAIL_READINGS
  static uint32_t sail_read_timer = millis();
  if (millis() - sail_read_timer < 3000) return pot_sail_angle;
  sail_read_timer = millis();
  Serial.print("Rudder ADC: ");   Serial.println(pot_sail_ADC);
  Serial.print("Rudder angle: "); Serial.println(pot_sail_angle);
  #endif
  return pot_sail_angle;
}

int ReadSailRaw() {
  int pot_sail_ADC = analogRead(sailPinPotentiometer); //2V aqui        //4V Aqui            2V-->90 graus  4V-->10 graus
  return pot_sail_ADC;
}

void GetAllPixhawkReadings() {
  for (int i = 0; i < numberPixhawkPins; i++) {
    pixHawkReadingsPWM[i] = pulseIn(pixHawkReadingPins[i], HIGH);
  }
}

int16_t GetPixhawkReading(pixHawkChannels pixhawk_channel) {
  for (int i = 0; i < numberPixhawkPins; i++) {
    pixHawkReadingsPWM[i] = pulseIn(pixHawkReadingPins[i], HIGH);
  }
  return pixHawkReadingsPWM[pixhawk_channel];
}

int16_t GetPixhawkReadingToAngle(pixHawkChannels pixhawk_channel) {
  constexpr uint8_t rudder_buffer = 10;
  constexpr uint8_t sail_buffer = 10;
  switch(pixhawk_channel) { 
    case pixHawkChannels::rudderIndex:
      pixHawkReadingsPWM[pixhawk_channel] = map(pixHawkReadingsPWM[pixhawk_channel], pixhawkMinimalPWM, pixhawkMaximumPWM, rudderMinAngle + rudder_buffer, rudderMaxAngle - rudder_buffer);
      break;
    case pixHawkChannels::sailIndex: // Mapping is inverted for the sail eg 1000 = 0 degrees, 2000 = 90 degrees
      pixHawkReadingsPWM[pixhawk_channel] = map(pixHawkReadingsPWM[pixhawk_channel], pixhawkMinimalPWM, pixhawkMaximumPWM, sailMaxAngle - sail_buffer, sailMinAngle + sail_buffer);
      break;
    default: // Other options are not PID controlled by angle, so they do not join here.
      break;
  }
  return pixHawkReadingsPWM[pixhawk_channel];
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
    case MAVLink_options::rudder_angle:
    {
      mavlink_msg_param_set_pack(arduino_sys_id, arduino_comp_id, &msg, pixhawk_sys_id, pixhawk_comp_id, "RUDDER_ANGLE", data, MAV_VAR_FLOAT);
      //Serial.print("MAV: "); Serial.println(data);
    }
    break;

    case MAVLink_options::sail_angle:
    {
      mavlink_msg_param_set_pack(arduino_sys_id, arduino_comp_id, &msg, pixhawk_sys_id, pixhawk_comp_id,"SAIL_ANGLE", data, MAV_VAR_FLOAT);
      //Serial.print("MAV: "); Serial.println(data);
    }
    break;

    case MAVLink_options::rudder_pwm:
    {
      mavlink_msg_param_set_pack(arduino_sys_id, arduino_comp_id, &msg, pixhawk_sys_id, pixhawk_comp_id,"RUDDER_PWM", data, MAV_VAR_FLOAT);
      //Serial.print("MAV: "); Serial.println(data);
    }
    break;

    case MAVLink_options::sail_pwm:
    {
      mavlink_msg_param_set_pack(arduino_sys_id, arduino_comp_id, &msg, pixhawk_sys_id, pixhawk_comp_id,"SAIL_PWM", data, MAV_VAR_FLOAT);
      //Serial.print("MAV: "); Serial.println(data);
    }
    break;
  }  
 
  // Copy the message to the send buffer
  uint16_t length = mavlink_msg_to_send_buffer(buffer, &msg);
  // Send the message with the standard UART send function
  Serial3.write(buffer, length);
}

void GetSerialInput() {
  static char incomingByte = '\0';
  if(Serial.available() > 0) {
    incomingByte = Serial.read();
    switch(incomingByte) {
      case 'p':
        ++printSelectionIndex;
        if (printSelectionIndex > PrintSelection::PrintThrottle) printSelectionIndex = PrintSelection::PrintRudder;
        Serial.print("Print selection: "); Serial.println(printSelectionIndex);
        break;
    }
  }
}

