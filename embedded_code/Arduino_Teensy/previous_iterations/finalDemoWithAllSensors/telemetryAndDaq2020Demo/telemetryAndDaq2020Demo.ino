/*
  Teensy GEEC 2021 DAQ code

  Note:
  - With DAQ 2020 sensors
  - Accelerometer data not configured or made use of yet

  Acceleromoter:
  - sda -> pin 18
  - scl -> pin 19

  last updated 6th of May 2021
  by Aoife Prendergast

*/

#include <SoftwareSerial.h>
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "DFRobot_LCD.h"
#include <SD.h>
#include <SPI.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Bounce2.h>
#include <Arduino.h>
#include "TeensyThreads.h"

#define ONE_WIRE_BUS 15
#define NUM_SAMPLES 1                            // set how many samples you want to poll from the sensors

#define COMMANDMAX 512

#define SIM7070G_LTE_PWR 2
#define SIM7070G_GPS_PWR 9

#define SCL_PIN 19
#define SDA_PIN 18

#define LED_PIN 13

/*
    Start of defition of global variables
*/

SoftwareSerial sim7070G_lte(0, 1); // RX, TX
SoftwareSerial sim7070G_gps(7, 8); // RX, TX

File dataFile;

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature tempSensors(&oneWire);

unsigned char sample_count = 0;

char sim_response[COMMANDMAX];  // AT command result buffer

bool blinkState = false;

elapsedMillis everySecond;

MPU6050 accelgyro;
int16_t ax, ay, az = 0;  // define accel as ax,ay,az
int16_t gx, gy, gz = 0;  // define gyro as gx,gy,gz

// DAQ 2020
double radius = 0.246;

//-----Pin Variables------//
const int motorVPin   = A6;
const int motorIPin   = A8;
const int batteryVPin = A7;
const int batteryIPin = A9;

double batteryVoltage = 0;
double batteryCurrent = 0;
double motorCurrent = 0;
double motorVoltage = 0;

float accelerator_sum = 0;
float motorVoltage_sum = 0;
//float motor_I_voltage = 0;
float motorCurrent_sum = 0;
float batteryVoltage_sum = 0;
//float battery_I_voltage = 0;
float batteryCurrent_sum = 0;

float tempOne = 0;
float tempTwo = 0;
float tempThree = 0;

boolean driverWarning = false;                      // whether to enable driver **STOP** warning

DFRobot_LCD lcd(16, 2);

int minTotal = 0;
int secTotal = 0;
int lapCount = 0;
long lapTime = 0;
boolean lapFlag = false;
int minLap = 0;
int secLap = 0;
String lastLiveSDData = 0;

long startLapTime = 0;

long startTimeDisplay = 0;
long startTimePoll = 0;
long newTime = 0;
long timeDisplay = 0;

int updateScreenCount = 1;
//long count = 0;

int accelerator = 0;

int frequency = 1;
long numInterrupts = 0;

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

// debounce
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 10;    //* the debounce time; increase if the output flickers

const int hallEffectPin = 12;
//Bounce hallEffect = Bounce(hallEffectPin, 10);  // 10 ms debounce

/*
     Saving data GLOBAL VARIABLES

*/

String liveDataHeaders = "runStatus,fixStatus,dateAndTime,latitude,longitude,mslAltitude,speedOverGround,courseOverGround,fixMode,reserved1,HDOP,PDOP,VDOP,reserved2,gnssInView,reserevd3,HPA,VPA,connectionStatus,timeSinceStart,lapCount,lapTime,rpm,roadSpeed,distanceTravelled,motorVoltage,motorCurrent,batteryVoltage,batteryCurrent,accelerator,tempOne,tempTwo,tempThree,ax,ay,az,gx,gy,gz,rssi,ber,systemMode,opMode,MCC-MNC,LAC,CellID,ARFCN,RxLev,TrackLOAdjust,C1-C2";

// individual strings to store daq data w/ default values
String gps_info;
String connection;
String ueInfo;
String daq2020data;
String accelerometerData;
String signalQuality;

// Combination the 6 above
String liveSDData;
String liveMQTTData;

String sdFilePath;

/*
    End of defition of global variables
*/


// *************************************************************************************************//
// Threads
/*
void hallEffectThread() {
  while (1) {
    hallEffect.update();
    if (hallEffect.fell()) {
      wRPMcount++;
      distanceCount++;
      Serial.println("\t Hall Effect Trigger");
    }

    threads.yield();
  }
}
*/

void tempThread() {
  while (1) {

    tempSensors.requestTemperatures(); // Send the command to get temperatures
    tempOne = tempSensors.getTempCByIndex(0);
    tempTwo = tempSensors.getTempCByIndex(1);
    tempThree = tempSensors.getTempCByIndex(2);
    
    threads.yield();
  }
}

void lcdThread() {
  while (1) {
    // only executes after sensors have been polled
    if ( liveSDData != lastLiveSDData) {
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

      lastLiveSDData = liveSDData;
    }
    threads.yield();
  }
}

// *************************************************************************************************//

void setup() {
  pinMode(SIM7070G_GPS_PWR, OUTPUT);
  pinMode(SIM7070G_LTE_PWR, OUTPUT);
  pinMode(LED_PIN, OUTPUT);           // configure LED pin
  //pinMode(hallEffectPin, INPUT_PULLUP);

  // Open serial communications and wait for port to open:
  Serial.begin(57600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  Serial.println("\n\t  --- START OF PROGRAM -- \n");

  // ------------------ DAQ 2020 Sensors ------------------ //
  Serial.println("Setting up DAQ 2020 Sensors...\n");
  tempSensors.begin();  //Initialise 1-Wire library
  attachInterrupt(digitalPinToInterrupt(hallEffectPin), wRPMDetectorInterrupt, RISING);  // Speed Sensor Interrupt, Pin 2
  lcd.init();
  lcd.setPWM(REG_ONLY, 0);

  pinMode(motorVPin, INPUT);                                 // Motor Voltage
  pinMode(motorIPin, INPUT);                                 // Motor Current
  pinMode(batteryVPin, INPUT);                                 // Battery Voltage
  pinMode(batteryIPin, INPUT);                                 // Battery Current
  pinMode(accelerator, INPUT);                                 // Accelerator

  //analogReadAveraging(200);

  // ------------------ Accelerometer Setup ------------------ //

  // Setting up accelerometer connection
  Wire.setSDA(SDA_PIN);
  Wire.setSCL(SCL_PIN);
  Wire.begin();      // join I2C bus
  Serial.println("Initializing I2C devices...");

  accelgyro.initialize();

  // verify connection
  Serial.println("\tTesting device connections...");
  Serial.println(accelgyro.testConnection() ? "\tMPU6050 connection successful" : "\tMPU6050 connection failed");

  while (!accelgyro.testConnection()) {
    delay(1000); // wait for serial port to connect. Needed for native USB port only
    Serial.println("\tWaiting for device connections...");
  }

  // ------------------ GPS IoT Hat Setup ------------------- //
  Serial.println("\nStarting IoT Boards setup... ");
  Serial.print("\n\n \t *** GPS SET UP *** \n");

  // set the data rate for the SoftwareSerial port
  sim7070G_gps.begin(115200);

  sim7070G_gps.flush();
  Serial.flush();
  delay(50);

  send_at(&sim7070G_gps, "AT+IPR=115200"); // Set baud rate

  get_ok_response(&sim7070G_gps, "AT", SIM7070G_GPS_PWR);

  send_at(&sim7070G_gps, "AT+CGNSPWR=1");

  sim7070G_gps.flush();
  Serial.flush();
  delay(50);

  // ------------------ LTE IoT Hat Setup ------------------- //

  Serial.print("\n\n \t *** LTE SET UP *** \n");

  // set the data rate for the SoftwareSerial port
  sim7070G_lte.begin(115200);

  sim7070G_lte.flush();
  Serial.flush();
  delay(50);

  send_at(&sim7070G_lte, "AT+IPR=115200"); // Set baud rate

  get_ok_response(&sim7070G_lte, "AT", SIM7070G_LTE_PWR);

  connect_to_broker();

  // get -> max length of topic
  //     -> range of supported content lengths
  //     -> list of supported qos
  //     -> list of supported reatins
  send_at(&sim7070G_lte, "AT+SMPUB=?");

  sim7070G_lte.flush();
  Serial.flush();
  delay(50);

  Serial.println("\n\nFinishing IoT Boards setup");

  Serial.println("\n\nSD Card Setup setup");

  // ------------------ SD Card File Name  ------------------- //

  // Get local timestamp to organise saving the data on the SD card
  Serial.println(F("Get local datestamp for CSV file name"));
  sdFilePath = "";
  int x = -1;
  while (x == -1) {
    sdFilePath = send_at(&sim7070G_gps, "AT+CGNSINF");
    x = sdFilePath.indexOf("2021");
  }

  x = sdFilePath.indexOf("2021");
  sdFilePath = sdFilePath.substring(x, x + 14);
  sdFilePath = sdFilePath.substring(2, 4) + "-" + sdFilePath.substring(4, 6) + "-" + sdFilePath.substring(6, 8) + ".CSV";
  // Length (with one extra character for the null terminator)
  char path[sdFilePath.length() + 1];
  sdFilePath.toCharArray(path, sdFilePath.length() + 1);

  Serial.println(path);

  // ------------------ SD Card Setup  ------------------- //
  Serial.println("Initializing SD card...");

  // see if the card is present and can be initialized:
  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("Card failed, or not present");
    // oops
  } else {
    Serial.println("Card initialized.\n");
  }

  if (!SD.exists(path)) {
    Serial.print("Creating - ");
    Serial.println(path);
    dataFile = SD.open(path, FILE_WRITE);
    dataFile.close();
  }

  // SD
  store_to_SD(liveDataHeaders);

  // Initialize variables values
  gps_info = "";
  everySecond = 0;

  lcd.setCursor(0, 0);
  lcd.print("Initialising System");
  startTimePoll = millis();
  startTimeDisplay = millis() / 1000;

  //threads.addThread(hallEffectThread);
  threads.addThread(tempThread);
  threads.addThread(lcdThread);
}


void loop() { // run over and over

  if ( everySecond >= 1000 ) {
    everySecond = 0;
    newTime = millis() / 1000;
    timeDisplay = (newTime - startTimeDisplay);


    Serial.print("\n\t************** ");
    Serial.print((String)(millis() / 1000));
    Serial.println(" **************");

    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);

    connection = send_at(&sim7070G_lte, "AT+SMSTATE?");
    connection.remove(0, 24);
    int x = connection.indexOf("\r\n\r\nOK\r\n");
    if ( x != -1) {
      connection.remove(x, 14);
    }

    if ((connection.indexOf("1") == -1) && (connection.indexOf("2") == -1)) {
      connect_to_broker();
    }

    ueInfo = send_at(&sim7070G_lte, "AT+CPSI?");
    ueInfo.remove(0, 18);
    x = ueInfo.indexOf("\r\n\r\nOK\r\n");
    if ( x != -1) {
      ueInfo.remove(x, 14);
    }

    signalQuality = send_at(&sim7070G_lte, "AT+CSQ");         // Signal Quality Report
    signalQuality.remove(0, 16);
    x = signalQuality.indexOf("\r\n\r\nOK\r\n");
    if ( x != -1) {
      signalQuality.remove(x, 14);
    }
    poll_accelerometer();

    poll_DAQ_2020_sensors();

    gps_info = send_at(&sim7070G_gps, "AT+CGNSINF");
    gps_info.remove(0, 23);
    x = gps_info.indexOf("\r\n\r\nOK\r\n");
    if ( x != -1) {
      gps_info.remove(x, 14);
    }

    liveSDData =  gps_info + ", " + connection + ", " + daq2020data  + ", " + accelerometerData + ", " + signalQuality + ", " + ueInfo;
    // SD
    store_to_SD(liveSDData);

    liveMQTTData =  gps_info + "#" + connection + "#" + daq2020data + "#" + accelerometerData + "#" + ueInfo ;

    //send mqtt message
    send_message(liveMQTTData);

  }
}

// *************************************************************************************************//
//                                          FUNCTIONS                                               //
// *************************************************************************************************//

void power(int power_pin) {
  digitalWrite(power_pin, HIGH);
  delay(5000);
  digitalWrite(power_pin, LOW);
  delay(15000);
}

void store_to_SD(String dataToStore) {
  // open the file.
  char path[sdFilePath.length() + 1];
  sdFilePath.toCharArray(path, sdFilePath.length() + 1);
  dataFile = SD.open(path, FILE_WRITE);
  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println(dataToStore);
    dataFile.close();
    // print to the serial port too:
    Serial.print("Write to SD file : ");
    Serial.println(path);
    //Serial.print(" : ");
    //Serial.println(dataToStore);

  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("Error opening log file on SD card");
  }
}

String send_at(SoftwareSerial * sim7070G, String at_command) {
  // By default, the last intialized port is listening.
  // when you want to listen on a port, explicitly select it
  sim7070G->listen();

  Serial.flush();
  sim7070G->flush();
  delay(5);

  int t = 0;
  char c = 0;

  //Serial.print("Sending AT Command: ");
  //Serial.println(at_command);

  sim7070G->println(at_command);
  delay(150);

  if ( at_command == "AT+SMCONN") {
    //Serial.println("Waiting longer");
    delay(10000);
  }

  sim_response[0] = 0;

  while ( sim7070G->available() ) {
    c = sim7070G->read();
    sim_response[t] = c;
    sim_response[++t] = 0;

  }
  //Serial.print("Sim7070 Response: ");
  //Serial.println(sim_response);

  return sim_response;
}

void get_ok_response(SoftwareSerial * sim7070G, String comm, int pin) {
  // By default, the last intialized port is listening.
  // when you want to listen on a port, explicitly select it
  sim7070G->listen();

  bool ok = false;
  int count = 0;

  while (!ok) {

    String response = send_at(sim7070G, comm);

    int x = response.indexOf("OK");
    if (x != -1) {
      ok = true;
    }
    count++;
    if ( (ok == false) && (count % 3 == 0) ) {
      Serial.println("Attempting hitting power button!");
      power(pin);
    }
  }
}


String send_message(String message) {
  sim7070G_lte.listen();

  char c = 0;
  int t = 0;

  //Serial.print("Attempting to send message: ");
  //Serial.println(message);
  //Serial.print("Message length: ");
  //Serial.println(String(message.length()));

  String pub_AT_cmd = "AT+SMPUB=\"testingGeec\"," + String(message.length()) + ",0,0";
  sim7070G_lte.println(pub_AT_cmd);
  delay(50);
  sim7070G_lte.println(message);
  delay(200);

  sim_response[0] = 0;

  while ( sim7070G_lte.available() ) {
    c = sim7070G_lte.read();
    sim_response[t] = c;
    sim_response[++t] = 0;
  }
  //Serial.print("Sim7070 Response: ");
  //Serial.println(sim_response);

  return sim_response;
}

void reconnect_to_broker() {

  // By default, the last intialized port is listening.
  // when you want to listen on a port, explicitly select it
  sim7070G_lte.listen();

  bool ok = false;
  int count = 0;

  while (!ok) {
    Serial.print("\n\n \t\t * Reconnecting to broker * \n");

    get_ok_response(&sim7070G_lte, "AT", SIM7070G_LTE_PWR);

    sim7070G_lte.flush();
    Serial.flush();
    delay(10);

    // Set MQTT Parameters & connect to broker
    send_at(&sim7070G_lte, "AT+SMCONF=\"URL\",broker.emqx.io,1883");
    send_at(&sim7070G_lte, "AT+SMCONF=\"CLIENTID\",mqttx_546aaee8");
    // Is 60 the right value to have here?
    send_at(&sim7070G_lte, "AT+SMCONF=\"KEEPTIME\",60");
    send_at(&sim7070G_lte, "AT+SMCONF=\"TOPIC\",testingGeec");
    send_at(&sim7070G_lte, "AT+SMCONF?");
    String response = send_at(&sim7070G_lte, "AT+SMCONN");    // MQTT Connection

    int x = response.indexOf("OK");
    if (x != -1) {
      ok = true;
    }
    count++;
  }
  everySecond = 0;
}

void connect_to_broker() {

  // By default, the last intialized port is listening.
  // when you want to listen on a port, explicitly select it
  sim7070G_lte.listen();

  bool ok = false;
  int count = 0;

  while (!ok) {

    Serial.print("\n\n \t\t * Connecting to broker * \n");

    get_ok_response(&sim7070G_lte, "AT", SIM7070G_LTE_PWR);

    sim7070G_lte.flush();
    Serial.flush();
    delay(10);

    send_at(&sim7070G_lte, "AT+SMDISC");
    send_at(&sim7070G_lte, "AT+CNACT?");      // Get local IP
    send_at(&sim7070G_lte, "AT+CNACT=0,0");   // Disconnect wireless
    send_at(&sim7070G_lte, "AT+CREG=0");      // disable network registration

    send_at(&sim7070G_lte, "AT+CSQ");         // Signal Quality Report
    send_at(&sim7070G_lte, "AT+CNMP=2");      // Set mode to automatically LTE or GSM
    send_at(&sim7070G_lte, "AT+CMNB=3");      // Set mode to CAT-M & NB-IOT
    send_at(&sim7070G_lte, "AT+CPSI?");       // Enquiring UE system information
    send_at(&sim7070G_lte, "AT+CGREG?");      // Network registration status
    send_at(&sim7070G_lte, "AT+CREG=1");      // anable network registration
    send_at(&sim7070G_lte, "AT+CREG?");       // Network registration status

    send_at(&sim7070G_lte, "AT+CNACT?");      // Get local IP
    send_at(&sim7070G_lte, "AT+CNACT=0,1");   // APP Network Active
    send_at(&sim7070G_lte, "AT+CNACT?");      // Get local IP
    send_at(&sim7070G_lte, "AT+CACID=0");     // Set TCP/UDP Identifier

    // Set MQTT Parameters & connect to broker
    send_at(&sim7070G_lte, "AT+SMCONF=\"URL\",test.mosquitto.org,1883");
    send_at(&sim7070G_lte, "AT+SMCONF=\"CLIENTID\",mqttx_546aaee8");
    // Is 60 the right value to have here?
    send_at(&sim7070G_lte, "AT+SMCONF=\"KEEPTIME\",60");
    send_at(&sim7070G_lte, "AT+SMCONF=\"TOPIC\",testingGeec");
    send_at(&sim7070G_lte, "AT+SMCONF?");
    String response = send_at(&sim7070G_lte, "AT+SMCONN");    // MQTT Connection

    int x = response.indexOf("OK");
    if (x != -1) {
      ok = true;
    }
    count++;
  }
  everySecond = 0;
}

void poll_accelerometer() {

  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);  // read measurements from device

  // display tab-separated accel/gyro x/y/z values
  /*
    Serial.print("a/g:\t");
    Serial.print(ax);
    Serial.print("\t");
    Serial.print(ay);
    Serial.print("\t");
    Serial.print(az);
    Serial.print("\t");
    Serial.print(gx);
    Serial.print("\t");
    Serial.print(gy);
    Serial.print("\t");
    Serial.println(gz);*/

  accelerometerData = ((String)ax + ", "
                       + (String)ay + ", "
                       + (String)az + ", "
                       + (String)gx + ", "
                       + (String)gy + ", "
                       + (String)gz );
}

void poll_DAQ_2020_sensors() {

  // increment one value by 1 for now :)
  // Just to show that the method has been entered

  sample_count = 0;
  accelerator_sum = 0;
  motorVoltage_sum = 0;
  //motor_I_voltage = 0;
  motorCurrent_sum = 0;
  batteryVoltage_sum = 0;
  //battery_I_voltage = 0;
  batteryCurrent_sum = 0;

  lapTime = 0;    // Seconds
  rpm = 0;
  roadSpeed = 0;
  distanceTravelled = 0;

  rpm = wRPMcalc();

  driverWarning = false;                      // whether to enable driver **STOP** warning


  // poll

  while (sample_count < NUM_SAMPLES) {
    // poll sensors

    //accelerator_sum = analogRead(accelerator);

    motorVoltage = (((analogRead(motorVPin)) - 390.22) / (9.629));
    //motorVoltage = analogRead(motorVPin);
    motorVoltage_sum += motorVoltage;

    //motor_I_voltage = (analogRead(motorIPin) * (5.0 / 1024.0));
    //motorCurrent = (((analogRead(motorIPin)) - 511.22)/(10.699));
    motorCurrent = (((analogRead(motorIPin)) - 771.05) / (16.078));
    //motorCurrent = analogRead(motorIPin);
    motorCurrent_sum += motorCurrent;

    batteryVoltage = (((analogRead(batteryVPin))  - 390.22) / (9.629));
    //batteryVoltage = analogRead(batteryVPin) ;
    batteryVoltage_sum += batteryVoltage;

    //battery_I_voltage = (analogRead(batteryIPin) * (5.0 / 1024.0));
    batteryCurrent = (((analogRead(batteryIPin)) - 771.05) / (16.078));
    //batteryCurrent = analogRead(batteryIPin);
    batteryCurrent_sum += batteryCurrent;
    sample_count++;
  }

  sample_count = 0;

  //accelerator = ((float)accelerator_sum / (float)NUM_SAMPLES);

  motorVoltage = ((float)motorVoltage_sum / (float)NUM_SAMPLES);
  motorCurrent = ((float)motorCurrent_sum / (float)NUM_SAMPLES);

  batteryVoltage = ((float)batteryVoltage_sum / (float)NUM_SAMPLES);
  batteryCurrent = ((float)batteryCurrent_sum / (float)NUM_SAMPLES);

  roadSpeed = convertKMHR(rpm);
  distanceTravelled = distance();

  // sending bench demo data
  daq2020data = ((String)(millis() / 1000) + ", "
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
                 + (String)tempThree );
  String temps = ( "1: " + (String)tempOne + ", "
                   + "2: " + (String)tempTwo + ", "
                   + "3: " + (String)tempThree );
  Serial.print("Battery Voltage: ");
  Serial.println(batteryVoltage);
  Serial.print("Battery Current: ");
  Serial.println(batteryCurrent);
  Serial.print("Temperatures: ");
  Serial.println(temps);
  Serial.print("Distance Travelled: ");
  Serial.println(distanceTravelled);

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
  Serial.println("\tHall Effect Interrupt Detected");
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

/*
  float getCurrent(float currentRead) {
  float I_voltage = (currentRead * (4.99 / 1024.0));
  float current = abs((I_voltage - 2.49) / 0.05);
  return current;
  }
*/
