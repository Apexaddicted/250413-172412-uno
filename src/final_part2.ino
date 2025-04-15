// ECE485W Final Project
// Solar charger for lithium ion battery, controlled by Arduino Uno.
#include <Arduino.h>

// Pin declarations
#define INPUT_PIN_SOLAR_VOLTAGE 0   // A0 Analog input pin for reading voltage from solar panels.
#define INPUT_PIN_ACS_CURRENT 1     // A1 Analog input pin for reading current from ACS712.
#define INPUT_PIN_BATT_VOLTAGE 2    // A2 Analog input pin for reading voltage from battery.
#define OUTPUT_PIN_PWM 6            // D6 Digital output pin for writing PWM signal.

// Constants
const int NUM_READS = 100;                          // Average reads of pin value.
const float ACS_SENSITIVITY = 185.0;                // mV/A
const float BOARD_VOLTAGE = 4910.0;                 // Actual voltage measured on 5v pin.
const float MID_VOLTAGE = BOARD_VOLTAGE / 2.0;      // Half the board voltage measured.

const float BATTERY_LOW_LIMT = 2700.0;
const float BATTERY_CC_LIMIT = 4000.0;
const float BATTERY_CV_LIMIT = 4000.0;
const float BATTERY_HARD_LIMIT = 4200.0;

// Initial variable declarations
// PWM Control
int dutyCycle = 0;

// ACS712
float acsRawValue = 0.0;            // Averaged input value from Aduino ADC 0-1023.
float acsCalOffset = 0.0;           // Calibration value for +/- acsRawValue.
float acsVoltage = 0.0;             // Calculated voltage using acsRawValue.
float acsCurrent = 0.0;             // Calculated current using acsVoltage.

// Voltage Sensing
float solarRawValue = 0.0;
float solarVoltage = 0.0;
float battRawValue = 0.0;
float battVoltage = 0.0;

// Finite State Machine for battery
enum State{
  initialize = 0, 
  const_current = 1, 
  const_voltage = 2, 
  charge_complete = 3, 
  charge_error = 4
} ChargeState;


void setup() {
  Serial.begin(9600);
  pinMode(INPUT_PIN_SOLAR_VOLTAGE, INPUT);
  pinMode(INPUT_PIN_ACS_CURRENT, INPUT);
  pinMode(INPUT_PIN_BATT_VOLTAGE, INPUT);
  pinMode(OUTPUT_PIN_PWM, OUTPUT);
  
  //TCCR0B = TCCR0B & B11111000 | B00000011;      // for PWM frequency of 976.56 Hz (The DEFAULT)
  TCCR0B = TCCR0B & B11111000 | B00000101;        // for PWM frequency of 61.04 Hz

  ChargeState = initialize;
}


void loop() {
  RefreshChargeCurrent();
  RefreshChargeVoltage();
  RefreshBatteryVoltage();
  CheckState();
  PrintStatus();
}


// ***NEED TO FINISH***
// *****************************************************************
void CheckState() {

  switch(ChargeState) {

    case initialize:
      if (CheckNextState(ChargeState) == true) {
        ChargeState = const_current;
      }
      break;
    
    case const_current:
      if (CheckNextState(ChargeState) == true) {
        ChargeState = const_voltage;
      }
      break;
    
    case const_voltage:
      if (CheckNextState(ChargeState) == true) {
        ChargeState = charge_complete;
      }
      break;

    case charge_complete:
      // Stay here
      break;

    case charge_error:
      // Stay here
      break;

    default:
      ChargeState = charge_error;
  }

}
// *****************************************************************


// ***NEED TO FINISH***
// *****************************************************************
boolean CheckNextState(State CurrentState) {
  boolean nextStateReady;

  switch(CurrentState) {

    case initialize:
      // ***STUB***
      if (battVoltage < BATTERY_CC_LIMIT && battVoltage > BATTERY_LOW_LIMT) {
        // move on to const_current
        nextStateReady = true;
      }
      else {
        nextStateReady = false;
      }
      break;
    
    case const_current:
      // ***STUB***
      if (battVoltage == battVoltage) { 
        // move on to const_voltage
        nextStateReady = true;
      }
      else {
        nextStateReady = false;
      }
      break;
    
    case const_voltage:
      // ***STUB***
      if (battVoltage == battVoltage) { 
        // move on to charge_complete
        nextStateReady = true;
      }
      else {
        nextStateReady = false;
      }
      break;

    case charge_complete:
      // Stay here
      nextStateReady = false;
      break;

    case charge_error:
      // Stay here  
      nextStateReady = false;
      break;

    default:
      nextStateReady = false;

  }

}
// *****************************************************************


// ***NEED TO FINISH***
// *****************************************************************
float GetAcsOffset(void) {
  float tempValue;
  // READ ACS PIN VOLTAGE WHILE WE KNOW THAT NO CURRENT IS FLOWING
  SetDutyCycle(0);
  delay(10);

  // SET THE ACS OFFSET CALIBRATION VALUE
  tempValue = GetPinVoltage(INPUT_PIN_ACS_CURRENT);
  
}
// *****************************************************************


// Refresh current flow from charger.
void RefreshChargeCurrent() { 
  acsVoltage = GetPinVoltage(INPUT_PIN_ACS_CURRENT);      // Get acs sensor voltage.
  acsCurrent = ConvertVolt2Amp(acsVoltage);               // Convert acs sensor voltage to current.
}


// Refresh voltage from charger.
void RefreshChargeVoltage() {
  solarVoltage = GetPinVoltage(INPUT_PIN_SOLAR_VOLTAGE);  // Get solar panel voltage.
}


// Refresh voltage on battery.
void RefreshBatteryVoltage() {
  battVoltage = GetPinVoltage(INPUT_PIN_BATT_VOLTAGE);    // Get battery voltage.
}


// Set duty cycle to control charging during Constant Voltage 
void SetDutyCycle(int dutyCycle){
  int pwmValue;
  pwmValue = ConvertPWM2Analog(dutyCycle);                          // Convert duty cycle (0% - 100%) to 8-bit analog output (0 - 255)
  analogWrite(OUTPUT_PIN_PWM, pwmValue);                            // Write (0-255) analog output to output pin
}


// Get the 10-bit value on a pin and convert it to a voltage
float GetPinVoltage(int inputPin) {
  float analogValue = GetAvgInput(100, inputPin);                   // Get average analog value on input pin. (0.0 - 1023.0)
  float pinVoltage = ConvertAnalog2Volt(analogValue);               // Convert acsRawValue to voltage. (0.0V - 5000.0V)
  return pinVoltage;
}


// Convert 10-bit value to a calculated voltage (0.0V - 5000.0V)
float ConvertAnalog2Volt(float analogValue){
  return ((BOARD_VOLTAGE/1023.0) * analogValue);
}


// Convert ACS712 sensor voltage to a calculated current (-5000.0mA - 5000.0mA)
float ConvertVolt2Amp(float inputVoltage){
  float tempValue;
  tempValue = ((inputVoltage - MID_VOLTAGE) / ACS_SENSITIVITY);
  return tempValue * 1000.0;
}


// Scale a (0-100) value to an 8-bit (0 - 255) value
int ConvertPWM2Analog(int percentPWM) {
  float tempValue;
  tempValue = ((percentPWM/100.0) * 255.0);
  return tempValue;
}


// Return the average value on inputPin
float GetAvgInput(int numReads, int inputPin) {
  float combinedInputs = 0.0;
  float averagedInput = 0.0;
  for (int i = 0; i < numReads; i++) {
    combinedInputs += analogRead(inputPin);
  }
  averagedInput = combinedInputs / numReads;
  return averagedInput;
}


// Get the string value for the current state
String GetCurrentState() {
  String currState;
  switch(ChargeState) {
    case initialize:    
      currState = "initialize";
      break;
    case const_current:
      currState = "const_current";
      break;
    case const_voltage:
      currState = "const_voltage";
      break;
    case charge_complete:
      currState = "charge_complete";
      break;
    case charge_error:
      currState = "charge_error";
      break;
    default:
      currState = "unknown_state";
  }
  return currState;
}


// Print readings to Serial Monitor for debugging
void PrintStatus(){
  String currState = GetCurrentState();
  Serial.print("\n");
  Serial.print("Current State: ");
  Serial.print(currState);
  Serial.print("\n");
  Serial.print ("Duty Cycle: ");
  Serial.print(dutyCycle);
  Serial.print("\n");
  Serial.print("Solar Voltage: ");
  Serial.print(solarVoltage);
  Serial.print(" mV");
  Serial.print("\n");
  Serial.print("Battery Voltage: ");
  Serial.print(battVoltage);
  Serial.print(" mV");
  Serial.print("\n");
  Serial.print("Charge Current: ");
  Serial.print(acsCurrent);
  Serial.print(" mA");
  Serial.print("\n");
}