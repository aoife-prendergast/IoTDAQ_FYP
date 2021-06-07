
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

#define TIMEOUT 300        // AT-command timeout in 100ms  -> 200=20sec
#define COMMANDMAX 1024

int TimeOut=TIMEOUT;
char sim_response[COMMANDMAX];  // AT command result buffer
int num;

SoftwareSerial sim7070G(8, 9); // RX, TX

void power(){
  digitalWrite(6,HIGH);
  delay(2000);
  digitalWrite(6,LOW);
  delay(2000);
}

String send_at(String command){
  int i=0,j=0;
  
  int t=0,n=0;
  char c=0;

  Serial.print("Sending AT Command: ");
  Serial.println(command);
 
  sim7070G.println(command);
  delay(10000);
  
  sim_response[0]=0;

  Serial.println(sim7070G.available());
  
  while ( sim7070G.available() ){
      c = sim7070G.read();
      sim_response[t] = c;
      sim_response[++t] = 0;
  }
  Serial.print("Sim7070 Response: ");
  Serial.println(sim_response);

  return sim_response;

}

void get_at_ok(){
  String response = "";
  bool ok = false;
  int count = 0;
 
  while(!ok){
    response = send_at("AT");

    int i = response.indexOf("OK");
    if(i != -1){
      ok = true;
    }
    count++;
    if( (ok == false) && (count%3 == 0) ){
      Serial.println("Attempting hitting power button!");
      power();
      delay(15000);
    }
  }
}

void send_message(String message){
  char c=0;
  char r_buf[100];
  int t=0,n=0;
  
  Serial.print("Attempting to send message: ");
  Serial.println(message);

  String pub_AT_cmd = "AT+SMPUB=\"testingGeec\"," + String(message.length()) + ",0,0";
  sim7070G.println(pub_AT_cmd);
  delay(50);
  sim7070G.println(message);
  delay(500);

  sim_response[0]=0;
  
  while ( sim7070G.available() ){
      c = sim7070G.read();
      sim_response[t] = c;
      sim_response[++t] = 0;
  }
  Serial.print("Sim7070 Response: ");
  Serial.println(sim_response);
  
  delay(2000);
}

void setup() {
  pinMode(9,OUTPUT);
  digitalWrite(9,LOW);
  delay(1000);
  
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  Serial.println("\n\nStarting program");

  // set the data rate for the SoftwareSerial port
  sim7070G.begin(9600);

  power();
  delay(15000);

  delay(1000);
  sim7070G.flush();
  Serial.flush();
  delay(2000);

  Serial.println(F("Configuring to 9600 baud"));
  send_at("AT+IPR=9600"); // Set baud rate
  delay(3000);

  get_at_ok();

  /*
  //Signal Quality Report 
  send_at("AT+CSQ");
  
  //Enquiring UE system information 
  send_at("AT+CPSI?");
  
  //Network registration status
  send_at("AT+CGREG?");
  
  send_at("AT+CNACT=0,1");
  send_at("AT+CACID=0");
  send_at("AT+SMCONF=\"URL\",broker.emqx.io,1883");
  send_at("AT+SMCONF=\"CLIENTID\",mqttx_546aaee8");
  send_at("AT+SMCONF=\"KEEPTIME\",60");
  send_at("AT+SMCONF?");
  send_at("AT+SMCONN");
  

  send_at("AT+CGNSMOD=1,1,0,0,0");
  send_at("AT+SGNSCFG=\"NMEAOUTPORT\",1");
  send_at("AT+CGNSPWR=1");

  */

  delay(1000);
  sim7070G.flush();
  Serial.flush();
  delay(2000);

  send_at("AT+CGNSPWR=1");
} 

void loop() { // run over and over
  
  //String gps = send_at("AT+SGNSCMD=1,0");
  //delay(2000);
  //send_message(String(num++));
  
  send_at("AT+CGNSINF");
  delay(10000);
}
