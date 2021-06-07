/*  Last updated: 22/03/2020 
 By Aoife Prendergast 
 
 Adapted from Arduino Mega to Teensy 3.5
 
 *********** Sensors *********** 
 Already there: 
 - Voltage sensing 
 - Current sensing 
 - Temperature 
 - LCD display 
 - Hall Effect sensor 
 Added: 
  - Accelerometer 
  - Sim7070G IoT Hat for GPS
  - Sim7070G IoT Hat for LTE
 Removed: 
 - Bluetooth module 
*/

#include <SD.h>
#include "DFRobot_LCD.h"
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#define ONE_WIRE_BUS 7
#define NUM_SAMPLES 1                             // set how many samples you want to poll from the sensors
unsigned char sample_count = 0;

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

double radius = 0.246;

//-----Pin Variables------//
const int motorVPin = A3;
const int motorIPin = A1;
const int batteryVPin = A2;
const int batteryIPin = A0;
const int acceleratorPin = A4;

float accelerator_sum = 0;
float motorVoltage_sum = 0;
float motor_I_voltage = 0;
float motorCurrent_sum = 0;
float batteryVoltage_sum = 0;
float battery_I_voltage = 0;
float batteryCurrent_sum = 0;

float tempOne = 0;
float tempTwo = 0;
float tempThree = 0;

boolean driverWarning = false;                      // whether to enable driver **STOP** warning

File myFile;

DFRobot_LCD lcd(16,2);

int minTotal = 0;
int secTotal = 0;
int lapCount = 0;
long lapTime = 0;
boolean lapFlag = false;
int minLap = 0;
int secLap = 0;

long startLapTime = 0;

long startTimeDisplay = 0;
long startTimePoll = 0;
long newTime = 0;
long timeDisplay = 0;

int updateScreenCount = 1;
long count = 0;

int accelerator = 0;

int frequency = 1;
long numInterrupts = 0;

double batteryVoltage = 0;
double batteryCurrent = 0;
double motorCurrent = 0;
double motorVoltage = 0;

//speed variables
int wRPMcount = 0;
long newWheelTime = 0;
long oldWheelTime = 0;
int numMagnets = 7;                   // Number of magnets on car
double rpm = 0;

int distanceCount = 0;                // Variable assignment RPM converted to distance
unsigned long distanceKM = 0;         // Variable assignment for cumulative distance travelled

double roadSpeed = 0;
double distanceTravelled = 0;

int hallEffectPin = 2;

String liveData;

// debounce
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 10;    //* the debounce time; increase if the output flickers

void setup() {
  Serial.begin(9600);
  Serial2.begin(9600);
  sensors.begin();  //Initialise 1-Wire library
  //Set-up LCD
    lcd.init();
    lcd.setPWM(REG_ONLY, 0);

  attachInterrupt(digitalPinToInterrupt(hallEffectPin), wRPMDetectorInterrupt, RISING);  // Speed Sensor Interrupt, Pin 2

  pinMode(motorVPin, INPUT);                                 // Motor Voltage
  pinMode(motorIPin, INPUT);                                 // Motor Current
  pinMode(batteryVPin, INPUT);                                 // Battery Voltage
  pinMode(batteryIPin, INPUT);                                 // Battery Current
  pinMode(accelerator, INPUT);                                 // Accelerator

  pinMode(4, OUTPUT);                                // SD card

  //lap counter
  attachInterrupt(1, lap, RISING);                   // pin 3
  pinMode(3, INPUT);                                  // set pin to input
  digitalWrite(3, HIGH);

  //set-up SD card
  if (!SD.begin(4)) {
    return;
  }
  myFile = SD.open("Data.csv", FILE_WRITE);

  // if file can be opened write header
  if (myFile) {
    myFile.println("time, lapCount, lapTime, wRPM, speed, distance, motorVoltage, motorCurrent, batteryVoltage, batteryCurrent, accelerator,tempOne, tempTwo, tempThree ");
    Serial.println("Wrote header to SD card");
    // close the file
    myFile.close();
  }

  lcd.setCursor(0, 0);
  startTimePoll = millis();
  startTimeDisplay = millis() / 1000;
}

void loop() {

  newTime = millis() / 1000;
  timeDisplay = (newTime - startTimeDisplay);

  if ( millis() - startTimePoll >=  count * (1000 / frequency) ) {
    pollSensors();
    count++;
  }
  
  if (lapFlag == true) {
    lapTime = timeDisplay - startLapTime;
    startLapTime = timeDisplay;
    lapCount++;
    lapFlag = false;
  }

  //driver warning
  //  if ((motorCurrent > 30 || batteryCurrent > 30 ) && driverWarning) {
  //    lcd.clear();
  //    lcd.setCursor(0, 0);
  //    lcd.print("******STOP******");
  //  } else if (count == updateScreenCount) {                //only update display every nth iteration
  //    send_display(roadSpeed, distanceTravelled, lapCount);
  //    //count = 0;
  //  }
}

void pollSensors() {
  sample_count = 0;
  accelerator_sum = 0;
  motorVoltage_sum = 0;
  motor_I_voltage = 0;
  motorCurrent_sum = 0;
  batteryVoltage_sum = 0;
  battery_I_voltage = 0;
  batteryCurrent_sum = 0;

  rpm = wRPMcalc();

  sensors.requestTemperatures(); // Send the command to get temperatures

  while (sample_count < NUM_SAMPLES) {
    // poll sensors

    accelerator_sum = analogRead(accelerator);

    motorVoltage = (((analogRead(motorVPin)) - 257.78)/(6.3955));
    motorVoltage_sum += motorVoltage;

    motor_I_voltage = (analogRead(motorIPin) * (5.0 / 1024.0));
    //motorCurrent = (((analogRead(motorIPin)) - 511.22)/(10.699));
    motorCurrent = (((analogRead(motorIPin)) - 518.6)/(10.556));
    motorCurrent_sum += motorCurrent;

    batteryVoltage = (((analogRead(batteryVPin)) - 257.78)/(6.3955));
    batteryVoltage_sum += batteryVoltage;

    battery_I_voltage = (analogRead(batteryIPin) * (5.0 / 1024.0));
    batteryCurrent = (((analogRead(batteryIPin)) - 518.6)/(10.556));
    batteryCurrent_sum += batteryCurrent;
    sample_count++;
  }

  sample_count = 0;

  accelerator = ((float)accelerator_sum / (float)NUM_SAMPLES);

  motorVoltage = ((float)motorVoltage_sum / (float)NUM_SAMPLES);
  motorCurrent = ((float)motorCurrent_sum / (float)NUM_SAMPLES);

  batteryVoltage = ((float)batteryVoltage_sum / (float)NUM_SAMPLES);
  batteryCurrent = ((float)batteryCurrent_sum / (float)NUM_SAMPLES);

  tempOne = sensors.getTempCByIndex(0);
  tempTwo = sensors.getTempCByIndex(1);
  tempThree = sensors.getTempCByIndex(2);


  roadSpeed = convertKMHR(rpm);
  distanceTravelled = distance();

  send_display(roadSpeed, distanceTravelled, lapCount);
  liveData = ((String)(millis() / 1000) + ", "
              + (String)lapCount + ", "
              + (String)lapTime + ", "
              + (String)rpm + ", "
              + (String)roadSpeed + ", "
              + (String)distanceTravelled + ", "
              + (String)motorVoltage + ", "
              + (String)motorCurrent + ", "
              + (String)batteryVoltage + ", "
              + (String)batteryCurrent + ", "
              + (String)accelerator + ", "
              + (String)tempOne + ", "
              + (String)tempTwo + ", "
              + (String)tempThree + "\n");

  myFile = SD.open("Data.csv", FILE_WRITE);

  if (myFile) {
    myFile.print(liveData);
    myFile.close();
  }

  Serial.print(liveData);
  Serial2.print(liveData);
}

//lap counter interrupt
void lap() {
  if ((millis() - lastDebounceTime) > debounceDelay) {
    lastDebounceTime = millis();
    lapFlag = true;
  }
}

// speed functions
void wRPMDetectorInterrupt () {
  wRPMcount++;
  distanceCount++;
}

double wRPMcalc() {
  detachInterrupt(digitalPinToInterrupt(hallEffectPin));

  newWheelTime = (millis() / 1000.0);                                                           // convert to seconds
  rpm = (60.0 * frequency * (wRPMcount / (newWheelTime - oldWheelTime))) / numMagnets;          // revolutions per minute
  wRPMcount = 0;
  oldWheelTime = (millis() / 1000.0);                                                           // convert to seconds
  attachInterrupt(digitalPinToInterrupt(hallEffectPin), wRPMDetectorInterrupt, RISING);
  return rpm;
}


double convertKMHR(long RPM) {
  double kmHrValue;                                            // Wheel diameter: 508mm
  kmHrValue = (RPM * ((2 * 3.14159) / 60.0) * radius * 3600.0 / 1000.0);
  if (kmHrValue > 0) {
    return kmHrValue;
  }
  return 0;
}

double convertMPS(long RPM) {
  double mpsValue;                                            // Wheel diameter: 508mm
  mpsValue = (RPM * ((2 * 3.14159) / 60.0) * radius);
  if (mpsValue > 0) {
    return mpsValue;
  }
  return 0;
}

double distance() {
  distanceKM = (2 * 3.14159 * radius * distanceCount / numMagnets);
  return distanceKM;
}

float getCurrent(float currentRead) {
  float I_voltage = (currentRead * (4.99 / 1024.0));
  float current = abs((I_voltage - 2.49) / 0.05);
  return current;
}

void send_display(double roadSpeed, double distanceTravelled, int lapCount) {
  minTotal =   timeDisplay / 60;
  secTotal =  timeDisplay % 60;
  minLap = lapTime / 60;
  secLap = lapTime % 60;

  // format strings
  String timestrTotal = String() + (minTotal < 10 ? "0" : "") + minTotal
                        + ':' + (secTotal < 10 ? "0" : "") + secTotal;

  String timestrLap = String() + (minLap < 10 ? "0" : "") + minLap
                      + ':' + (secLap < 10 ? "0" : "") + secLap;

  if (roadSpeed < 10) {
    lcd.setCursor(0, 1);
    lcd.print("0" + (String)roadSpeed + "m/s  " + timestrTotal);
  } else {
    lcd.setCursor(0, 1);
    lcd.print((String)roadSpeed + "m/s  " + timestrTotal);
  }

  lcd.setCursor(0, 0);
  lcd.print((String)distanceTravelled + "m  " + lapCount + "  " + timestrLap);
}
