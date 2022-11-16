/*-----------------
  Master Lora Node
  The IoT Projects
-------------------*/
#include <PubSubClient.h>
#include <WiFi.h>
#include <SPI.h>           // include libraries
#include <LoRa.h>
// ESP 32 LoRa Pin-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#define ss   5            //GPIO 5
#define rst  14           //GPIO 14
#define dio0 2            //GPIO 2
// WiFi Set-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
const char* ssid ="Nhahee family 2.4G";// your ssid
const char* password = "69696969"; // your password
// MQTT Set-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
const char* mqtt_server = "192.168.1.19";   // your mqtt server IP 
const char* mqtt_username = "";             // your mqtt username
const char* mqtt_password = "";             // your mqtt password
const uint16_t mqtt_port = 1883;
WiFiClient tcpClient;                       // create tcp client object
PubSubClient mqttClient(tcpClient);         // pass tcp client to mqtt client
// My Node Byte----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
byte MasterNode = 0xFF;
byte Node1 = 0xBB;
byte Node2 = 0xCC;
byte Node3 = 0xDD;
byte Node4 = 0xAA;
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
String outgoing;              // outgoing message
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
byte msgCount = 0;            // count of outgoing messages
String incoming = "";
uint32_t lastTimestamp = millis();
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Tracks the time since last event fired
unsigned long previousMillis   = 0;
unsigned long int previoussecs = 0;
unsigned long int currentsecs  = 0;
unsigned long currentMillis    = 0;
int interval = 1 ; // updated every 1 second
int Secs = 0;
//------------------------------------------------------------------------------
void setup_wifi(const char* ssid, const char* password){
  Serial.println("Connecting to: ");
  Serial.println(ssid);
  WiFi.begin(ssid,password);
  while(WiFi.status() != WL_CONNECTED){
    delay(500);
    Serial.print(".");
  }
  Serial.println("\r\nWiFi Connected");
  Serial.println("IP Address: ");
  Serial.println(WiFi.localIP());
}
//------------------------------------------------------------------------------
void reconnect(){
  while(!mqttClient.connected()){
    Serial.println("Attemping MQTT Connection !");
    String clientID = "ESP32Client-" + String(WiFi.macAddress());
    Serial.print("ClientID : ");
    Serial.println(clientID);
    if (mqttClient.connect(clientID.c_str(),mqtt_username,mqtt_password)){
      Serial.println("MQTT Broker Connected");
      String msg = "Hello from " + clientID;
      mqttClient.publish("ESP32/heartbeat",msg.c_str());
    }
    else{
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println("try again in 1 seconds");
      delay(1000);
    }
  }
}
//------------------------------------------------------------------------------
void callBack(char* topic, byte* payload, uint16_t payload_length){
  String msg;
  String topic_str(topic);
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.println("] :");
  for (uint16_t i = 0; i < payload_length; i++ ){
    char c = (char)payload[i];
    msg += c;
  }
  Serial.println(msg);
}
//------------------------------------------------------------------------------
void setup() 
{
  Serial.begin(115200); // initialize serial
  setup_wifi(ssid,password);
  mqttClient.setServer(mqtt_server,mqtt_port);
  mqttClient.setCallback(callBack);
  mqttClient.setCallback(callBack);
  Serial.println("LoRa Master Node");
  LoRa.setPins(ss, rst, dio0);
  if (!LoRa.begin(433E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void loop() 
{
  if(!mqttClient.connected()){
    reconnect();
  }
  if(millis() - lastTimestamp > 1000){
    String msg("Timestamp from ESP32 [");
    msg += String(millis()) += String("]");
    Serial.print("Publish message: ");
    Serial.println(msg);
    mqttClient.publish("ESP32/heartbeat",msg.c_str());
  mqttClient.loop();
  
  currentMillis = millis();
  currentsecs = currentMillis / 1000;
  if ((unsigned long)(currentsecs - previoussecs) >= interval) {
    Secs = Secs + 1;
    //Serial.println(Secs);
    if ( Secs >= 21 )
    {
      Secs = 0;
      Serial.println("Master Node on Rest........");
    }
    // Time for Node 1 --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    if ( (Secs >= 1) && (Secs <= 5) )
    {
      String message = "10";
      sendMessage(message, MasterNode, Node1);
      Serial.println("Send Request to Node 1");
    }
    // Time for Node 2 --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    if ( (Secs >= 6 ) && (Secs <= 10))
    {
      String message = "20";
      sendMessage(message, MasterNode, Node2);
      Serial.println("Send Request to Node 2");
    }
    // Time for Node 3 --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    if ( (Secs >=11 ) && (Secs <= 15))
    {
      String message = "30";
      sendMessage(message, MasterNode, Node3);
      Serial.println("Send Request to Node 3");
    }
    // Time for Node 4 --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    if ( (Secs >= 16 ) && (Secs <= 20))
    {
      String message = "40";
      sendMessage(message, MasterNode, Node4);
      Serial.println("Send Request to Node 4");
    }
    //-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    previoussecs = currentsecs;
  }
  // parse for a packet, and call onReceive with the result:
  onReceive(LoRa.parsePacket());
  lastTimestamp = millis();
  }
}
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Send 10,20,30,40 to Request Node to Send to This Node ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void sendMessage(String outgoing, byte MasterNode, byte otherNode) {
  LoRa.beginPacket();                   // start packet
  LoRa.write(otherNode);                // add destination address
  LoRa.write(MasterNode);               // add sender address
  LoRa.write(msgCount);                 // add message ID
  LoRa.write(outgoing.length());        // add payload length
  LoRa.print(outgoing);                 // add payload
  LoRa.endPacket();                     // finish packet and send it
  msgCount++;                           // increment message ID
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void onReceive(int packetSize) {
  if (packetSize == 0) return;          // if there's no packet, return
  // read packet header bytes:
  int recipient = LoRa.read();          // recipient address
  byte sender = LoRa.read();            // sender address
  byte incomingMsgId = LoRa.read();     // incoming msg ID
  byte incomingLength = LoRa.read();    // incoming msg length
  
  while (LoRa.available()) {
    incoming += (char)LoRa.read();
  }
  if (incomingLength != incoming.length()) {   // check length for error
    Serial.println("error: message length does not match length");
    return;                             // skip rest of function
  }
  // if the recipient isn't this device or broadcast,
  if (recipient != Node1 && recipient != MasterNode) {
    Serial.println("This message is not for me.");
    return;                             // skip rest of function
  }
  //Bin 4---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  if ( sender == 0XAA )
  {
    String sb4 = getValue(incoming, ',', 0); // B4-Sensor1
    String valB4 = getValue(incoming, ',', 1); // B4-Sensor2

    incoming = "";
    //clear display
    Serial.println("Node4:");
    // show Bin4 sensor 1
    Serial.print("Bin4_S1: ");
    Serial.print(sb4);
    Serial.print(" ");
    Serial.println("cm");
    // show Bin4 sensor 2
    Serial.print("Value From Node : ");
    Serial.println(valB4);
    Serial.println(" ");
    mqttClient.publish("FromESP/LevelBin4", sb4.c_str());
  }
  //------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  //Bin 3---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  if ( sender == 0XDD )
  {
    String sb3 = getValue(incoming, ',', 0); // B3-Sensor1
    String valB3 = getValue(incoming, ',', 1); // B3-Sensor2

    incoming = "";
    //clear display
    Serial.println("Node3:");
    // show Bin3 sensor 1
    Serial.print("Bin3_S1: ");
    Serial.print(sb3);
    Serial.print(" ");
    Serial.println("cm");
    // show Value
    Serial.print("Value From Node : ");
    Serial.println(valB3);
    Serial.println(" ");
    mqttClient.publish("FromESP/LevelBin3", sb3.c_str());
  }
  //------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  //------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  if ( sender == 0XCC )
  {
    String sb2 = getValue(incoming, ',', 0); // B2-Sensor1
    String valB2 = getValue(incoming, ',', 1); // B2-Sensor2

    incoming = "";
    //clear display
    Serial.println("Node2:");
    // show Bin2 sensor 1
    Serial.print("Bin2_S1: ");
    Serial.print(sb2);
    Serial.print(" ");
    Serial.println("cm");
    // show Value
    Serial.print("Value From Node : ");
    Serial.println(valB2);
    Serial.println(" ");
    mqttClient.publish("FromESP/LevelBin2", sb2.c_str());
  }
  //---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  if ( sender == 0XBB )
  {
    String sb1 = getValue(incoming, ',', 0); // B1-Sensor1
    String valB1 = getValue(incoming, ',', 1); // B1-Sensor2

    incoming = "";
    //clear display
    Serial.println("Node1:");
    // show Bin1 sensor 1
    Serial.print("Bin1_S1: ");
    Serial.print(sb1);
    Serial.print(" ");
    Serial.println("cm");
    // show Value
    Serial.print("Value From Node : ");
    Serial.println(valB1);
    Serial.println(" ");
    mqttClient.publish("FromESP/LevelBin1", sb1.c_str());
  }
  //------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
}
String getValue(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = { 0, -1 };
  int maxIndex = data.length() - 1;

  for (int i = 0; i <= maxIndex && found <= index; i++) {
    if (data.charAt(i) == separator || i == maxIndex) {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i + 1 : i;
    }
  }
  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}
