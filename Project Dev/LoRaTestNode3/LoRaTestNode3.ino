/*-------------------
  Lora Node3
  The IoT Projects
---------------------*/
#include <SPI.h>              // include libraries
#include <LoRa.h>
#include <NewPing.h>          // Sensor Library
//--------------------------------------------------------------------------
#define ss 10
#define rst 9
#define dio0 2
// Sensor 1 Pin --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#define TRIGGER1  5                                      //| Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO1     6                                       //| Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE1 100                                 //| Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
NewPing sonar1(TRIGGER1, ECHO1, MAX_DISTANCE1);           //| NewPing setup of pins and maximum distance.
// Sensor 2 Pin --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#define TRIGGER2  7                                      //| Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO2     8                                      //| Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE2 100                                 //| Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
NewPing sonar2(TRIGGER2, ECHO2, MAX_DISTANCE2);           //| NewPing setup of pins and maximum distance.
//--------------------------------------------------------------------------
String outgoing;              // outgoing message
//--------------------------------------------------------------------------
byte msgCount = 0;            // count of outgoing messages
byte MasterNode = 0xFF;
byte Node3 = 0xDD;
// Sensor------------------------------------------------------------------
  int s_one  = 0;                    
  int s_two  = 0;
//--------------------------------------------------------------------------
String Mymessage = "";
//--------------------------------------------------------------------------
void setup() 
{
  Serial.begin(115200);                   // initialize serial
  while (!Serial);
  Serial.println("LoRa Node3");
  LoRa.setPins(ss, rst, dio0);
  if (!LoRa.begin(433E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
}
//--------------------------------------------------------------------------
void loop() 
{
  s_one  = sonar1.ping_cm();                    
  s_two  = sonar2.ping_cm();
  Serial.print(s_one); 
  Serial.println(s_two); 
  onReceive(LoRa.parsePacket());
}
//--------------------------------------------------------------------------
void onReceive(int packetSize) {
  if (packetSize == 0) return;          // if there's no packet, return
  // read packet header bytes:
  int recipient = LoRa.read();          // recipient address
  byte sender = LoRa.read();            // sender address
  byte incomingMsgId = LoRa.read();     // incoming msg ID
  byte incomingLength = LoRa.read();    // incoming msg length

  String incoming = "";

  while (LoRa.available()) {
    incoming += (char)LoRa.read();
  }

  if (incomingLength != incoming.length()) {   // check length for error
     Serial.println("error: message length does not match length");
    ;
    return;                             // skip rest of function
  }

  // if the recipient isn't this device or broadcast,
  if (recipient != Node3 && recipient != MasterNode) {
     Serial.println("This message is not for me.");
    ;
    return;                             // skip rest of function
  }
  Serial.println(incoming);
  int Val = incoming.toInt();
  if (Val == 20)
  {
    Mymessage = Mymessage + s_one + "," + s_two;
    sendMessage(Mymessage, MasterNode, Node3);
    delay(100);
    Mymessage = "";
  }

}
//--------------------------------------------------------------------------
void sendMessage(String outgoing, byte MasterNode, byte Node3) {
  LoRa.beginPacket();                   // start packet
  LoRa.write(MasterNode);               // add destination address
  LoRa.write(Node3);                    // add sender address
  LoRa.write(msgCount);                 // add message ID
  LoRa.write(outgoing.length());        // add payload length
  LoRa.print(outgoing);                 // add payload
  LoRa.endPacket();                     // finish packet and send it
  msgCount++;                           // increment message ID
}
