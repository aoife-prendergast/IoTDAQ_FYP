
/*
  Teensy GEEC 2021 DAQ code 
    GPS teensy trial 

 "  Receives from the hardware serial, sends to software serial.
    Receives from software serial, sends to hardware serial. "

 The circuit:
 * RX is digital pin X (connect to TX of other device)
 * TX is digital pin X (connect to RX of other device)

 Note:
  ... things

 Adding Acceleromoter: 
 - sda -> pin 4 
 - scl -> pin 3

 modified 12st of April 2021
 by Aoife Prendergast

 */
 
#include <SoftwareSerial.h>
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"

#define COMMANDMAX 512

#define SIM7070G_LTE_PWR 2
#define SIM7070G_GPS_PWR 9

#define SCL_PIN 19
#define SDA_PIN 18

char sim_response[COMMANDMAX];  // AT command result buffer
String gps_info;
String connection;

SoftwareSerial sim7070G_lte(0, 1); // RX, TX
SoftwareSerial sim7070G_gps(7, 8); // RX, TX

MPU6050 accelgyro;
int16_t ax, ay, az;  // define accel as ax,ay,az
int16_t gx, gy, gz;  // define gyro as gx,gy,gz

#define LED_PIN 13
bool blinkState = false;

elapsedMillis everySecond;

void power(int power_pin){
  digitalWrite(power_pin,HIGH);
  delay(5000);
  digitalWrite(power_pin,LOW);
  delay(20000);
}

String send_at(SoftwareSerial *sim7070G, String at_command){
  // By default, the last intialized port is listening.
  // when you want to listen on a port, explicitly select it
  sim7070G->listen();
  
  Serial.flush();
  sim7070G->flush();
  delay(10);
  
  int t=0;
  char c=0;

  Serial.print("Sending AT Command: ");
  Serial.println(at_command);
 
  sim7070G->println(at_command);
  delay(200);

  if( at_command == "AT+SMCONN"){ 
    Serial.println("Waiting longer");
    delay(5000);
  }

  sim_response[0]=0;
  
  while ( sim7070G->available() ){
      c = sim7070G->read();
      sim_response[t] = c;
      sim_response[++t] = 0;

  }
  Serial.print("Sim7070 Response: ");
  Serial.println(sim_response);

  return sim_response;
}

void get_ok_response(SoftwareSerial *sim7070G, String comm, int pin){
  // By default, the last intialized port is listening.
  // when you want to listen on a port, explicitly select it
  sim7070G->listen();
  
  bool ok = false;
  int count = 0;

  while(!ok){

    String response = send_at(sim7070G, comm);

    int x = response.indexOf("OK");
    if(x != -1){
      ok = true;
    }
    count++;
    if( (ok == false) && (count%3 == 0) ){
      Serial.println("Attempting hitting power button!");
      power(pin);
    }
  }
}


String send_message(SoftwareSerial *sim7070G, String message){
  sim7070G->listen();
  
  char c=0;
  int t = 0;
  
  Serial.print("Attempting to send message: ");
  Serial.println(message);
  Serial.print("Message length: ");
  Serial.println(String(message.length()));

  String pub_AT_cmd = "AT+SMPUB=\"testingGeec\"," + String(message.length()) + ",0,0";
  sim7070G->println(pub_AT_cmd);
  delay(50);
  sim7070G->println(message);
  delay(200);

  sim_response[0]=0;
  
  while ( sim7070G->available() ){
      c = sim7070G->read();
      sim_response[t] = c;
      sim_response[++t] = 0;
  }
  Serial.print("Sim7070 Response: ");
  Serial.println(sim_response);

  return sim_response;
}

void connect_to_broker(){ 

  // By default, the last intialized port is listening.
  // when you want to listen on a port, explicitly select it
  sim7070G_lte.listen();
  
  bool ok = false;
  int count = 0;

  while(!ok){

    Serial.print("\n\n \t *** Connecting to broker *** \n\n");
    //power(SIM7070G_LTE_PWR);

    sim7070G_lte.flush();
    Serial.flush();
    delay(50);

    get_ok_response(&sim7070G_lte, "AT", SIM7070G_LTE_PWR);
    send_at(&sim7070G_lte, "AT+SMDISC");
    send_at(&sim7070G_lte, "AT+CSQ");         // Signal Quality Report 
    send_at(&sim7070G_lte, "AT+CNMP=2");      // Set mode to automatically LTE or GSM 
    send_at(&sim7070G_lte, "AT+CMNB=3");      // Set mode to CAT-M & NB-IOT
    send_at(&sim7070G_lte, "AT+CPSI?");       // Enquiring UE system information 
    send_at(&sim7070G_lte, "AT+CGREG?");      // Network registration status
    send_at(&sim7070G_lte, "AT+CNACT?");      // Get local IP 
    send_at(&sim7070G_lte, "AT+CNACT=0,0");   // Disconnect wireless
    send_at(&sim7070G_lte, "AT+CNACT?");      // Get local IP 
    send_at(&sim7070G_lte, "AT+CNACT=0,1");   // APP Network Active 
    send_at(&sim7070G_lte, "AT+CNACT?");      // Get local IP 
    send_at(&sim7070G_lte, "AT+CACID=0");     // Set TCP/UDP Identifier 
  
    // Set MQTT Parameters & connect to broker 
    send_at(&sim7070G_lte, "AT+SMCONF=\"URL\",broker.emqx.io,1883");
    send_at(&sim7070G_lte, "AT+SMCONF=\"CLIENTID\",mqttx_546aaee8");
    // Is 60 the right value to have here?
    send_at(&sim7070G_lte, "AT+SMCONF=\"KEEPTIME\",60");          
    send_at(&sim7070G_lte, "AT+SMCONF=\"TOPIC\",testingGeec");
    send_at(&sim7070G_lte, "AT+SMCONF?");   
    String response = send_at(&sim7070G_lte, "AT+SMCONN");    // MQTT Connection 

    int x = response.indexOf("OK");
    if(x != -1){
      ok = true;
    }
    count++;
  }
  everySecond = 0;
}

void poll_accelerometer(){

  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);  // read measurements from device

  // display tab-separated accel/gyro x/y/z values
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
  
}

void setup() {
  pinMode(SIM7070G_GPS_PWR, OUTPUT);
  pinMode(SIM7070G_LTE_PWR, OUTPUT);
  pinMode(LED_PIN, OUTPUT);           // configure LED pin
  
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

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

  // GPS Setup 
  Serial.print("\n\n \t *** GPS SET UP *** \n\n");
  
  // set the data rate for the SoftwareSerial port
  sim7070G_gps.begin(115200);
  
  //power(SIM7070G_GPS_PWR);

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

  // LTE Setup 
  
  Serial.print("\n\n \t *** LTE SET UP *** \n\n");
    
  // set the data rate for the SoftwareSerial port
  sim7070G_lte.begin(115200);

  power(SIM7070G_LTE_PWR);

  sim7070G_lte.flush();
  Serial.flush();
  delay(50);
  
  Serial.println(F("Configuring to 9600 baud"));
  send_at(&sim7070G_lte, "AT+IPR=115200"); // Set baud rate

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

  // Initialize values 
  gps_info = "";
  connection = "";
  everySecond = 0;
} 

void loop() { // run over and over
  
  if( everySecond >= 1000 ) {
    everySecond = everySecond - 1000;
    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);

    connection = send_at(&sim7070G_lte, "AT+SMSTATE?");
    /*
    if((connection.indexOf("1") == -1) && (connection.indexOf("2") == -1)){
      connect_to_broker();
    }
    */
    gps_info = send_at(&sim7070G_gps, "AT+SGNSCMD=1,0");
    //send_message(&sim7070G_lte, gps_info);
    Serial.println(gps_info);

    poll_accelerometer();
  } 
}
