#include <SPI.h>
#include <SD.h>
#include "HX711.h"
#include <RH_RF95.h>

//Assigning all necessary digital pins
#define RFM95_CS 9
#define RFM95_RST 8
#define RFM95_INT 3
#define SD_CS 10
#define DOUT 4
#define CLK 2

//Setting up necessary values and objects
#define RF95_FREQ 915.0 //Setting the frequency to 915 Mhz

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);
HX711 scale;
float calibration_factor = 8710; // A factor of 8710 works for units of N and a factor of 86 works for units of g

void setup() {
 // Open serial communications and wait for port to open:
 Serial.begin(9600);
 pinMode(RFM95_RST, OUTPUT);
 digitalWrite(RFM95_RST, HIGH);
 
 // manual reset
 digitalWrite(RFM95_RST, LOW);
 delay(10);
 digitalWrite(RFM95_RST, HIGH);
 delay(10);
 
  while (!rf95.init()) {
   Serial.println("LoRa radio init failed");
   while (1);
 }
 Serial.println("LoRa radio init OK!");
 
 // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
 if (!rf95.setFrequency(RF95_FREQ)) {
   Serial.println("setFrequency failed");
   while (1);
 }
 rf95.setTxPower(23, false);
 scale.begin(DOUT, CLK);
 scale.set_scale(calibration_factor);
 scale.tare();
 long zero_factor = scale.read_average(); //Get a baseline reading
 
 Serial.print("Zero factor: "); //This can be used to remove the need to tare the scale. Useful in permanent scale projects.
 Serial.println(zero_factor);
 
 while (!Serial) {
   ; // wait for serial port to connect. Needed for native USB port only
 }
 Serial.print("Initializing SD card...");
 
 // see if the card is present and can be initialized:
 if (!SD.begin(SD_CS)) {
   Serial.println("Card failed, or not present");
   // don’t do anything more:
   while (1);
 }
 Serial.println("card initialized.");
 
 // Now we will get initialize the load cell
 int counter = 0;
 if(counter < 6)
 {
   counter += 1;
   scale.get_units();
 }
 scale.tare();
}

bool STANDBY = true;
bool ARMED = false;
bool PRE_IGNITION = false;
bool DATA_ACQUISITION = false;

#define PREPIN 5 // Number may change later

pinMode(PREPIN, INPUT_PULLUP);

void loop() 
{
uint8_t buf[55]; //Content of Message
uint8_t len = sizeof(buf); //Size of Message

  // Once on, we immediately go to the 'standby' while loop 
  while (STANDBY)
  {    
      if (rf95.available())
      {
        if(rf95.recv(buf, &len)
      }

      if (buf == "Go1")
      {
        STANDBY = false;
        PRE_IGNITION = true;
      }
        
  }

  while (PREPHASE)
  {
    
  }

  

}
