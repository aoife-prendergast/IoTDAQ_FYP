
/*
  Software serial multple serial test

 Receives from the hardware serial, sends to software serial.
 Receives from software serial, sends to hardware serial.

 The circuit:
 * RX is digital pin 10 (connect to TX of other device)
 * TX is digital pin 11 (connect to RX of other device)

 Note:
 Not all pins on the Mega and Mega 2560 support change interrupts,
 so only the following can be used for RX:
 10, 11, 12, 13, 50, 51, 52, 53, 62, 63, 64, 65, 66, 67, 68, 69

 Not all pins on the Leonardo and Micro support change interrupts,
 so only the following can be used for RX:
 8, 9, 10, 11, 14 (MISO), 15 (SCK), 16 (MOSI).

 created back in the mists of time
 modified 25 May 2012
 by Tom Igoe
 based on Mikal Hart's example

 This example code is in the public domain.

 */
 
#include <SoftwareSerial.h>

#define COMMANDMAX 256

#define SIM7070G_GPS_PWR 6
#define SIM7070G_LTE_PWR 5

char sim_response[COMMANDMAX];  // AT command result buffer
String gps_info;
String connection;

SoftwareSerial sim7070G_gps(8, 9); // RX, TX
SoftwareSerial sim7070G_lte(10, 11); // RX, TX
int num = 0;

void power(int power_pin){
  digitalWrite(power_pin,HIGH);
  delay(2000);
  digitalWrite(power_pin,LOW);
  delay(2000);
}

String send_at(SoftwareSerial *sim7070G, String command){
  // By default, the last intialized port is listening.
  // when you want to listen on a port, explicitly select it
  sim7070G->listen();
  
  Serial.flush();
  sim7070G->flush();
  delay(100);
  
  int i=0,j=0;
  
  int t=0,n=0;
  char c=0;

  Serial.print("Sending AT Command: ");
  Serial.println(command);
 
  sim7070G->println(command);
  delay(1000);

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

void get_at_ok(SoftwareSerial *sim7070G, int pin){
  // By default, the last intialized port is listening.
  // when you want to listen on a port, explicitly select it
  sim7070G->listen();
  
  bool ok = false;
  int count = 0;

  while(!ok){
    String command = "AT";

    int i=0,j=0;
    
    int t=0,n=0;
    char c=0;
  
    Serial.print("Sending AT Command: ");
    Serial.println(command);
   
    sim7070G->println(command);
    delay(500);
  
    sim_response[0]=0;
    
    while ( sim7070G->available() ){
        c = sim7070G->read();
        sim_response[t] = c;
        sim_response[++t] = 0;
  
    }
    Serial.print("Sim7070 Response: ");
    Serial.println(sim_response);

    String response(sim_response);

    int x = response.indexOf("OK");
    if(x != -1){
      ok = true;
    }
    count++;
    if( (ok == false) && (count%3 == 0) ){
      Serial.println("Attempting hitting power button!");
      power(pin);
      delay(15000);
    }
  }
}


String send_message(SoftwareSerial *sim7070G, String message){
  sim7070G->listen();
  
  char c=0;
  int t=0,n=0;
  
  Serial.print("Attempting to send message: ");
  Serial.println(message);
  Serial.print("Message length: ");
  Serial.println(String(message.length()));

  String pub_AT_cmd = "AT+SMPUB=\"testingGeec\"," + String(message.length()) + ",0,0";
  sim7070G->println(pub_AT_cmd);
  delay(100);
  sim7070G->println(message);
  delay(500);

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

void reconnect_to_broker(){ 

  Serial.print("\n\n \t *** Reconnecting to broker *** \n\n");

  // By default, the last intialized port is listening.
  // when you want to listen on a port, explicitly select it
  sim7070G_lte.listen();
  
  bool ok = false;
  int count = 0;

  while(!ok){
    power(SIM7070G_LTE_PWR);
    delay(15000);

    delay(500);
    sim7070G_lte.flush();
    Serial.flush();
    delay(500);
  
    Serial.println(F("Configuring to 9600 baud"));
    send_at(&sim7070G_lte, "AT+IPR=9600"); // Set baud rate

    get_at_ok(&sim7070G_lte, SIM7070G_LTE_PWR);
    
    //Signal Quality Report 
    send_at(&sim7070G_lte, "AT+CSQ");
  
    //Enquiring UE system information 
    send_at(&sim7070G_lte, "AT+CPSI?");
  
    //Network registration status
    send_at(&sim7070G_lte, "AT+CGREG?");
  
    send_at(&sim7070G_lte, "AT+CNACT=0,1");
    send_at(&sim7070G_lte, "AT+CACID=0");
    send_at(&sim7070G_lte, "AT+SMCONF=\"URL\",broker.emqx.io,1883");
    send_at(&sim7070G_lte, "AT+SMCONF=\"CLIENTID\",mqttx_546aaee8");
    send_at(&sim7070G_lte, "AT+SMCONF=\"KEEPTIME\",60");
    send_at(&sim7070G_lte, "AT+SMCONF?");

    String response = send_at(&sim7070G_lte, "AT+SMCONN");

    int x = response.indexOf("OK");
    if(x != -1){
      ok = true;
    }
    count++;
  }
  
}

void setup() {

  // Dont think this code does anything :) 
  pinMode(9,OUTPUT);
  digitalWrite(9,LOW);
  delay(1000);

  // Dont think this code does anything :) 
  pinMode(11,OUTPUT);
  digitalWrite(11,LOW);
  delay(1000);
  
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  Serial.println("\n\nStarting program");

  // GPS Setup 

  Serial.print("\n\n \t *** GPS SET UP *** \n\n");
  
  // set the data rate for the SoftwareSerial port
  sim7070G_gps.begin(9600);
  
  power(SIM7070G_GPS_PWR);
  delay(15000);

  delay(500);
  sim7070G_gps.flush();
  Serial.flush();
  delay(500);
  
  Serial.println(F("Configuring to 9600 baud"));
  send_at(&sim7070G_gps, "AT+IPR=9600"); // Set baud rate

  get_at_ok(&sim7070G_gps, SIM7070G_GPS_PWR);
  
  send_at(&sim7070G_gps, "AT+CGNSPWR=1");

  delay(500);
  sim7070G_gps.flush();
  Serial.flush();
  delay(500);
  

  // LTE Setup 
  
  Serial.print("\n\n \t *** LTE SET UP *** \n\n");
    
  // set the data rate for the SoftwareSerial port
  sim7070G_lte.begin(9600);

  power(SIM7070G_LTE_PWR);
  delay(15000);

  delay(500);
  sim7070G_lte.flush();
  Serial.flush();
  delay(500);
  
  Serial.println(F("Configuring to 9600 baud"));
  send_at(&sim7070G_lte, "AT+IPR=9600"); // Set baud rate

  get_at_ok(&sim7070G_lte, SIM7070G_LTE_PWR);

  //Signal Quality Report 
  send_at(&sim7070G_lte, "AT+CSQ");
  
  //Enquiring UE system information 
  send_at(&sim7070G_lte, "AT+CPSI?");
  
  //Network registration status
  send_at(&sim7070G_lte, "AT+CGREG?");
  
  send_at(&sim7070G_lte, "AT+CNACT=0,1");
  send_at(&sim7070G_lte, "AT+CACID=0");
  send_at(&sim7070G_lte, "AT+SMCONF=\"URL\",broker.emqx.io,1883");
  send_at(&sim7070G_lte, "AT+SMCONF=\"CLIENTID\",mqttx_546aaee8");
  send_at(&sim7070G_lte, "AT+SMCONF=\"KEEPTIME\",60");
  send_at(&sim7070G_lte, "AT+SMCONF?");
  send_at(&sim7070G_lte, "AT+SMCONN");
  send_at(&sim7070G_lte, "AT+SMPUB=?");

  delay(500);
  sim7070G_lte.flush();
  Serial.flush();
  delay(500);

  gps_info = "";
  connection = "";
  
} 

void loop() { // run over and over
  Serial.print("\n\n \t *** IN THE LOOP *** \n\n");
  
  connection = send_at(&sim7070G_lte, "AT+SMSTATE?");
  if((connection.indexOf("1") == -1) && (connection.indexOf("2") == -1)){
    reconnect_to_broker();
  }

  gps_info = send_at(&sim7070G_gps, "AT+CGNSINF");
  send_message(&sim7070G_lte, gps_info );
  
  delay(1000);
}
