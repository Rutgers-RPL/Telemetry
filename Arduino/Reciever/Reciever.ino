

// Arduino9x_RX
// -*- mode: C++ -*-
// Example sketch showing how to create a simple messaging client (receiver)
// with the RH_RF95 class. RH_RF95 class does not provide for addressing or
// reliability, so you should only use RH_RF95 if you do not need the higher
// level messaging abilities.
// It is designed to work with the other example Arduino9x_TX
 
#include <SPI.h>
#include <RH_RF95.h>
 
#define RFM95_CS 10
#define RFM95_RST 9
#define RFM95_INT 2
 
// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 915.0
 
// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);
 
// Blinky on receipt
//define LED D5
char com[1];
 
void setup() 
{
  pinMode(5, OUTPUT);
  pinMode(7, OUTPUT);     
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
 
  while (!Serial);
  Serial.begin(9600);
  delay(100);
 
  Serial.println("Arduino LoRa RX Test!");
  
  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);
 
  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    while (1);
  }
  digitalWrite(7, HIGH);
  digitalWrite(7, LOW);
  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);
 
  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on
 
  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then 
  // you can set transmitter powers from 5 to 23 dBm:
  //rf95.setTxPower(13, true);
  rf95.setTxPower(23); 
}
 
void loop()
{
  while (Serial.available()>0)com[0] = Serial.read();
  if (com[0] == '1' || com[0] == '0'){
    for(int i = 0; i < 20; i++){
     rf95.send((uint8_t *)com, 1);
     rf95.waitPacketSent();
    }
  }
  if (rf95.available())
  {
    // Should be a message for us now   
    char buf[71];
    uint8_t len = sizeof(buf);
    
    if (rf95.recv(buf, &len))
    {
      int counter =0;
      if (counter%8==0)
     // RH_RF95::printBuffer("Received: ", buf, len);
   //   Serial.print("Got: ");
      Serial.println(buf);
      Serial.print("RSSI: ");
      Serial.println(rf95.lastRssi(), DEC);
      //digitalWrite(LED, LOW);
      
      // Send a reply
//      uint8_t data[] = "And hello back to you";
//      rf95.send(data, sizeof(data));
//      rf95.waitPacketSent();
//      Serial.println("Sent a reply");
      counter++;
    }
    else
    {
//      digitalWrite(7, HIGH);
      Serial.println("Receive failed");
//      digitalWrite(7, LOW);
    }
  }
}
