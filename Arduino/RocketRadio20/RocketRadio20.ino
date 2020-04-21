#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <RH_RF95.h>
 
#define RFM95_CS 10
#define RFM95_RST 9
#define RFM95_INT 3

 #define GPSECHO  false
 
// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 915.0
 
// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);
 
// Blinky on receipt
#define LED 13

//IMU Library
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

//GPS Library
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

//Barometric Alt Library
#include <Adafruit_BMP280.h>

//IMU
#define BNO055_SAMPLERATE_DELAY_MS (100)

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);

//GPS Wiring
// Connect the GPS Power pin to 5V
// Connect the GPS Ground pin to ground
// Connect the GPS TX (transmit) pin to Digital 8
// Connect the GPS RX (receive) pin to Digital 7

// you can change the pin numbers to match your wiring:
SoftwareSerial mySerial(8, 7);
Adafruit_GPS GPS(&mySerial);

//Baro Alt
Adafruit_BMP280 bmp; // I2C
//Adafruit_BMP280 bmp(BMP_CS); // hardware SPI
//Adafruit_BMP280 bmp(BMP_CS, BMP_MOSI, BMP_MISO,  BMP_SCK);
double baseline;
int sendCounter = 0;
char cameraOn[1];

void setup() {
  Wire.begin();
     // Radio SetUP
       pinMode(RFM95_RST, OUTPUT);
       digitalWrite(RFM95_RST, HIGH);
 
       while (!Serial);
       Serial.begin(9600);
       delay(1000);
 
      // manual reset
      digitalWrite(RFM95_RST, LOW);
      delay(10);
      digitalWrite(RFM95_RST, HIGH);
      delay(10);
 
      while (!rf95.init()) {
      Serial.println("LoRa failed");
        while (1);
      }
 
      // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
      if (!rf95.setFrequency(915)) {
        Serial.println("setFrequency failed");
        while (1);
      }
  
      // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on
      // The default transmitter power is 13dBm, using PA_BOOST.
      // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then 
      // you can set transmitter powers from 5 to 23 dBm:
      rf95.setTxPower(23, false);

      GPS.begin(9600);
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time

  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);

     delay(1000);
     // Ask for firmware version
     // mySerial.println(PMTK_Q_RELEASE);

      //IMU Setup
  /* Initialise the sensor */
  if(!bno.begin()){
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  
    int8_t temp = bno.getTemp();
//  Serial.print("Current Temperature: ");
//  Serial.print(temp);
//  Serial.println(" C");
//  Serial.println("");

  bno.setExtCrystalUse(true);

  if (!bmp.begin()) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1);
  }
  baseline = (double)bmp.readPressure()/100;

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */ 
   
  cameraOn[0] = '0';
}//setup()

uint32_t timer = millis;
void loop() {
  if(rf95.available()){
    uint8_t cameraOnLen = sizeof(cameraOn);
    rf95.recv(cameraOn, &cameraOnLen);
  }
  // put your main code here, to run repeatedly:
    char c = GPS.read();
    // if you want to debug, this is a good time to do it!
    //if ((c) && (GPSECHO))
    //Serial.write(c);
    // if you want to debug, this is a good time to do it!
    // if a sentence is received, we can check the checksum, parse it...
    if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false

    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
    }
//Wire.beginTransmission(ACCEL_OFFSET_Z_MSB_ADDR);
//Wire.write(xxxxxx11b);
//Wire.endTransmission();

   // if millis() or timer wraps around, we'll just reset it
  if (timer > millis())  timer = millis();

  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 150) {
   char Telem[69];
   memset(Telem,'\0',69);
   timer = millis(); // reset the timer 
   imu::Vector<3> Gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
   imu::Vector<3> Accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
   imu::Vector<3> Mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);

   //int8_t Temp = bno.getTemp();
   //double pressure = bmp.readPressure(); //currently unused
   double altitude = bmp.readAltitude(baseline);
    
   dtostrf(altitude,7,1,Telem);
   Telem[7] = ' ';
   dtostrf(Gyro.x(),5,1,Telem+8);
   Telem[13] = ' ';
   dtostrf(Gyro.y(),5,1,Telem+14);
   Telem[19] = ' ';
   dtostrf(Gyro.z(),5,1,Telem+20);
   Telem[25] = ' ';
   dtostrf(Accel.x(),5,1,Telem+26);
   Telem[31] = ' ';
   dtostrf(Accel.y(),5,1,Telem+32);
   Telem[37] = ' ';
   dtostrf(Accel.z(),5,1,Telem+38);
//   dtostrf(Mag.x(),5,1,Telem+58);
//   Telem[63] = ' ';
//   dtostrf(Mag.y(),5,1,Telem+64);
//   Telem[69] = ' ';
//   dtostrf(Mag.z(),5,1,Telem+70);
//   Telem[75] = ' ';
   dtostrf(millis()/1000.0,8,3,Telem+43); //millis() will overflow back to zero after about 50 days
   Telem[51] = ' ';
   Telem[52] = cameraOn[0];
   
   if(GPS.latitudeDegrees != 0 && sendCounter >= 10){
     Telem[53] = ' '; 
     dtostrf(GPS.latitudeDegrees,7,4,Telem+54);
     Telem[61] = ' ';
     dtostrf(GPS.longitudeDegrees,7,4,Telem+62);
     rf95.send((uint8_t *)Telem, 71);
     Serial.println(Telem);
     sendCounter = 0;
   }else{
     rf95.send((uint8_t *)Telem, 53);
     Serial.println(Telem);
     sendCounter++;
   }
   rf95.waitPacketSent();
  }
}
