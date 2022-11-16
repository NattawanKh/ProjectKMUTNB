/*****************************************************************************************************************************************************************************************
           < LoRa Sender BIN 1 >
  < BIN 1 WASTE MESUREMENT SENSOR CONTROL >
*************************************************************************************************************************************************************************************************************************************/
#include <NewPing.h>       //| Ultrasonic Library
#include <SPI.h>           //| Serial Peripheral Interface Library
#include <LoRa.h>          //| LoRa Library  
//===================================================================================================================================================================================================================== 
// Sensor 1 Pin define--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#define TRIGGER1  5                                      //| Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO1     6                                       //| Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE1 100                                 //| Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
NewPing sonar1(TRIGGER1, ECHO1, MAX_DISTANCE1);           //| NewPing setup of pins and maximum distance.
// Sensor 2 Pin define --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#define TRIGGER2  7                                      //| Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO2     8                                      //| Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE2 100                                 //| Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
NewPing sonar2(TRIGGER2, ECHO2, MAX_DISTANCE2);           //| NewPing setup of pins and maximum distance.
//=================================================================================================================================================================================================================================================
//define the pins used by the transceiver module
#define ss 10
#define rst 9
#define dio0 2
//====================================================================================================================================================================================================================================
// Setup----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void setup() {
//initialize Serial Monitor ====================================================================================================================================================================================================================
  Serial.begin(115200);
// LoRa Setup ================================================================================================================================================================================================================================================
  while (!Serial);
  Serial.println("LoRa Sender");
  //setup LoRa transceiver module
  LoRa.setPins(ss, rst, dio0);  
  //replace the LoRa.begin(---E-) argument with your location's frequency 
  //433E6 for Asia
  //866E6 for Europe
  //915E6 for North America
  while (!LoRa.begin(433E6)) {Serial.println(".");delay(500);}  
  // Change sync word (0xF3) to match the receiver
  // The sync word assures you don't get LoRa messages from other LoRa transceivers
  // ranges from 0-0xFF
  LoRa.setSyncWord(0xBB);
  Serial.println(" "); 
  Serial.println("Bin1 Initializing OK!");
  }
//======================================================================================================================================================================================================================================  
void loop() {
//Set Sensor to Float--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  float s_one  = sonar1.ping_cm();                        //| set float for distance 1 in cm
  float s_two  = sonar2.ping_cm();                        //| set float for distance 2 in cm
  float s_avg  = (s_one+s_two)/2;                         //| set float for average distance
  float s_real = 0;                                       //| set float for real distance
  int  level   = 0;                                          //| set to show the level of distance in String
//Sensor 1 Program----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  //delay(1000);                                          //| Wait 1 Sec between pings (about 4000 pings/sec). 29ms should be the shortest delay between pings.
  Serial.print("Sensor 1 : ");                            //|
  Serial.print(s_one);                                    //| Send ping, get distance in cm and print result (0 = outside set distance range)
  Serial.println(" cm");                                  //|
//Sensor 2 Program--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  //delay(1000);                                          //| Wait 1 Sec between pings (about 4000 pings/sec). 29ms should be the shortest delay between pings.
  Serial.print("Sensor 2 : ");                            //|
  Serial.print(s_two);                                    //| Send ping, get distance in cm and print result (0 = outside set distance range)
  Serial.println(" cm");                                  //|
//Show Average--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  Serial.print("Sensor Average : ");                      //| 
  Serial.print(s_avg);                                    //| Send ping, get distance in cm and print result (0 = outside set distance range)
  Serial.println(" cm");                                  //|
  //Serial.println(" ");                                  //|
//Find Real Distance----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
       if  (s_one<s_two){s_real=s_one;}                   //| if Sensor 1 is real distance show Sensor 1 distance
  else if  (s_one>s_two){s_real=s_two;}                   //| if Sensor 2 is real distance show Sensor 2 distance
  else                  {s_real=s_avg;};                  //| if both show average distance                                     
//Define Level of BIN =====================================================================================================================================================================================================================================                                                                                                                                   //|
//       if  (s_real>=80){level=3 ;}                    //| if Real Distance is Low  Level
//  else if  (s_real>=40){level=2 ;}                    //| if Real Distance is Mid  Level
//  else                 {level=1 ;};                   //| if Real Distance is High Level
//==============================================================================================================================================================================================================
// LoRa Send ================================================================================================================================================================================================================================================================================================
  Serial.print("Sending Level: ");
  Serial.print(s_real);
  Serial.println(" cm");                                 
  //Send LoRa packet to receiver
  LoRa.beginPacket();
  //LoRa.print("level 1:");
  LoRa.print(s_real);                                      //| LoRa Send Level to Reciver
  LoRa.endPacket();
//================================================================================================================================================================================================================================================================================================
  delay(1000);  //| Wait for Send again
}
//========================================================================================================================================================================================================================================================================================================================================================================================