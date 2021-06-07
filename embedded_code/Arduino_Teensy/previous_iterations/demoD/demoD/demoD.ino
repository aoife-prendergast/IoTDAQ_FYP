/*
  Teensy GEEC 2021 DAQ code

  Note:
  - Without DAQ 2020 sensors
  - Accelerometer data not configured or made use of yet

  Acceleromoter:
  - sda -> pin 4
  - scl -> pin 3

  created 2nd of May 2021
  by Aoife Prendergast

*/

#include <SoftwareSerial.h>
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include <SD.h>
#include <SPI.h>

#define COMMANDMAX 512

#define SIM7070G_LTE_PWR 2
#define SIM7070G_GPS_PWR 9

#define SCL_PIN 19
#define SDA_PIN 18

char sim_response[COMMANDMAX];  // AT command result buffer

SoftwareSerial sim7070G_lte(0, 1); // RX, TX
SoftwareSerial sim7070G_gps(7, 8); // RX, TX

File dataFile;

#define LED_PIN 13
bool blinkState = false;

elapsedMillis everySecond;

/*
     TELEMETRY DATA GLOBAL VARIABLES

*/
String liveDataHeaders = "runStatus,fixStatus,dateAndTime,latitude,longitude,mslAltitude,speedOverGround,courseOverGround,fixMode,reserved1,HDOP,PDOP,VDOP,reserved2,gnssInView,reserevd3,HPA,VPA,connectionStatus,timeSinceStart,lapCount,lapTime,rpm,roadSpeed,distanceTravelled,motorVoltage,motorCurrent,batteryVoltage,batteryCurrent,accelerator,tempOne,tempTwo,tempThree,ax,ay,az,gx,gy,gz,rssi,ber,systemMode,opMode,MCC-MNC,LAC,CellID,ARFCN,RxLev,TrackLOAdjust,C1-C2";

MPU6050 accelgyro;
int16_t ax, ay, az = 0;  // define accel as ax,ay,az
int16_t gx, gy, gz = 0;  // define gyro as gx,gy,gz


// DAQ 2020
float accelerator = 0;
double batteryVoltage = 0;
double batteryCurrent = 0;
double motorCurrent = 0;
double motorVoltage = 0;

float tempOne = 0;
float tempTwo = 0;
float tempThree = 0;

boolean driverWarning = false;                      // whether to enable driver **STOP** warning

int lapCount = 0;
int lapTime = 0;    // Seconds
double rpm = 0;
double roadSpeed = 0;
double distanceTravelled = 0;

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

void setup() {
  pinMode(SIM7070G_GPS_PWR, OUTPUT);
  pinMode(SIM7070G_LTE_PWR, OUTPUT);
  pinMode(LED_PIN, OUTPUT);           // configure LED pin

  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  // ------------------ Accelerometer Setup ------------------ //

  // Setting up accelerometer connection
  Wire.setSDA(SDA_PIN);
  Wire.setSCL(SCL_PIN);
  Wire.begin();      // join I2C bus
  Serial.println("Initializing I2C devices...");

  accelgyro.initialize();

  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  while (!accelgyro.testConnection()) {
    delay(1000); // wait for serial port to connect. Needed for native USB port only
    Serial.println("Waiting for device connections...");
  }

  Serial.println("\n\nStarting IoT Boards setup");

  // ------------------ GPS IoT Hat Setup ------------------- //
  Serial.print("\n\n \t *** GPS SET UP *** \n\n");

  // set the data rate for the SoftwareSerial port
  sim7070G_gps.begin(115200);

  sim7070G_gps.flush();
  Serial.flush();
  delay(50);

  Serial.println(F("Configuring to 115200 baud"));
  send_at(&sim7070G_gps, "AT+IPR=115200"); // Set baud rate

  get_ok_response(&sim7070G_gps, "AT", SIM7070G_GPS_PWR);

  send_at(&sim7070G_gps, "AT+CGNSPWR=1");

  sim7070G_gps.flush();
  Serial.flush();
  delay(50);

  // ------------------ LTE IoT Hat Setup ------------------- //

  Serial.print("\n\n \t *** LTE SET UP *** \n\n");

  // set the data rate for the SoftwareSerial port
  sim7070G_lte.begin(115200);

  sim7070G_lte.flush();
  Serial.flush();
  delay(50);

  Serial.println(F("Configuring to 9600 baud"));
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
  sdFilePath = sdFilePath.substring(x, x+14);
  sdFilePath = sdFilePath.substring(2, 4) + "-" + sdFilePath.substring(4, 6) + "-" + sdFilePath.substring(6, 8) + ".CSV";
  // Length (with one extra character for the null terminator)
  char path[sdFilePath.length() + 1];
  sdFilePath.toCharArray(path, sdFilePath.length()+1);

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
}

void loop() { // run over and over

  if ( everySecond >= 1000 ) {

    everySecond = everySecond - 1000;

    Serial.println("\t********* NEW LOOP **************\n\n");

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

    liveSDData =  gps_info + ", " + connection + ", "+ daq2020data  + ", "+ accelerometerData + ", "+ signalQuality + ", "+ ueInfo;
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
  sdFilePath.toCharArray(path, sdFilePath.length()+1);
  dataFile = SD.open(path, FILE_WRITE);
  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println(dataToStore);
    dataFile.close();
    // print to the serial port too:
    Serial.print(path);
    Serial.print(" : ");
    Serial.println(dataToStore);
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
  delay(10);

  int t = 0;
  char c = 0;

  //Serial.print("Sending AT Command: ");
  //Serial.println(at_command);

  sim7070G->println(at_command);
  delay(150);

  if ( at_command == "AT+SMCONN") {
    Serial.println("Waiting longer");
    delay(5000);
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

    Serial.print("\n\n \t *** Reconnecting to broker *** \n\n");

    get_ok_response(&sim7070G_lte, "AT", SIM7070G_LTE_PWR);

    sim7070G_lte.flush();
    Serial.flush();
    delay(10);

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

void connect_to_broker() {

  // By default, the last intialized port is listening.
  // when you want to listen on a port, explicitly select it
  sim7070G_lte.listen();

  bool ok = false;
  int count = 0;

  while (!ok) {

    Serial.print("\n\n \t *** Connecting to broker *** \n\n");

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
  Serial.println(gz);
  */

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

  accelerator = 0;
  batteryVoltage = 0;
  batteryCurrent = 0;
  motorCurrent = 0;
  motorVoltage = 0;

  tempOne = 0;
  tempTwo = 0;
  tempThree = 0;

  driverWarning = false;                      // whether to enable driver **STOP** warning

  lapCount++;
  lapTime = 0;    // Seconds
  rpm = 0;
  roadSpeed = 0;
  distanceTravelled = 0;

  // sending dummy data atm - just being included to conforim the string transmits at the right length
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
}
