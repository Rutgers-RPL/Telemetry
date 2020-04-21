 #include <Arduino.h>


 
#include <SPI.h>
#include <RH_RF95.h>
 
#define RFM95_CS 10
#define RFM95_RST 9
#define RFM95_INT 2
 
// Change to 434.0 or other frequency, must match RX's freq!
//#define RF95_FREQ 915.0
 
// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);
 
// Blinky on receipt
#define LED 13



//IMU Libraries
#include <Wire.h>
//#include <Adafruit_Sensor.h>
#include <SparkFunLSM9DS1.h>

//
//GPS Library
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

//Barometric Alt Library
#include <SFE_BMP180.h>



//IMU
LSM9DS1 imu;
#define LSM9DS1_M  0x1E // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG  0x6B // Would be 0x6A if SDO_AG is LOW
#define PRINT_CALCULATED
//#define PRINT_RAW
#define PRINT_SPEED 250 // 250 ms between prints
static unsigned long lastPrint = 0; // Keep track of print time

// Earth's magnetic field varies by location. Add or subtract 
// a declination to get a more accurate heading. Calculate 
// your's here:
// http://www.ngdc.noaa.gov/geomag-web/#declination
#define DECLINATION -8.58 // Declination (degrees) in Boulder, CO.

//GPS
//GPS Wiring
// Connect the GPS Power pin to 5V
// Connect the GPS Ground pin to ground
// Connect the GPS TX (transmit) pin to Digital 8
// Connect the GPS RX (receive) pin to Digital 7

// you can change the pin numbers to match your wiring:
SoftwareSerial mySerial(8, 7);
Adafruit_GPS GPS(&mySerial);
#define GPSECHO  false



//Baro Alt
SFE_BMP180 pressure;
double baseline; // baseline pressure








double getPressure();
void setup() {

     // Radio SetUP
       pinMode(RFM95_RST, OUTPUT);
       digitalWrite(RFM95_RST, HIGH);
 
       while (!Serial);
       Serial.begin(9600);
       delay(100);
 
 
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
    //  Serial.println(915);
        Serial.println("wat");
  
      // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on
 
      // The default transmitter power is 13dBm, using PA_BOOST.
      // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then 
      // you can set transmitter powers from 5 to 23 dBm:
      rf95.setTxPower(23, false);




    //GPS Setup
      GPS.begin(9600);
//
//      // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
      GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
//      // uncomment this line to turn on only the "minimum recommended" data
//      //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
//      // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
//      // the parser doesn't care about other sentences at this time
//
//      // Set the update rate
      GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
//      // For the parsing code to work nicely and have time to sort thru the data, and
//      // print it out we don't suggest using anything higher than 1 Hz
//
//      // Request updates on antenna status, comment out to keep quiet
      GPS.sendCommand(PGCMD_ANTENNA);
//
     delay(1000);
//      // Ask for firmware version
//      mySerial.println(PMTK_Q_RELEASE);

      //IMU Setup
  // Before initializing the IMU, there are a few settings
  // we may need to adjust. Use the settings struct to set
  // the device's communication mode and addresses:
  imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.mAddress = LSM9DS1_M;
  imu.settings.device.agAddress = LSM9DS1_AG;
  // The above lines will only take effect AFTER calling
  // imu.begin(), which verifies communication with the IMU
  // and turns it on.
  if (!imu.begin())
  {
    Serial.println("Failed to communicate with LSM9DS1.");
    Serial.println("Double-check wiring.");
    Serial.println("Default settings in this sketch will " \
                  "work for an out of the box LSM9DS1 " \
                  "Breakout, but may need to be modified " \
                  "if the board jumpers are.");
    while (1)
      ;
  }



    //Baro Alt Setup

      if (!pressure.begin())
      {
        Serial.println("Baro Alt failed");
        while(1);
      }

      // Get the baseline pressure:
  
    baseline = getPressure();
 

}




uint32_t timer = millis;
void loop() {

  double a,P;
  char Telem[81];
  Telem[0] = 0;

  
  // put your main code here, to run repeatedly:
    char c = GPS.read();
  // if you want to debug, this is a good time to do it!
//  if ((c) && (GPSECHO))
//    Serial.write(c);
//
//  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
//    // a tricky thing here is if we print the NMEA sentence, or data
//    // we end up not listening and catching other sentences!
//    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
//    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
//
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
    }

  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis())  timer = millis();

  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 2000) {
    timer = millis(); // reset the timer
    
  // Update the sensor values whenever new data is available
  if ( imu.gyroAvailable() )
  {
    // To read from the gyroscope,  first call the
    // readGyro() function. When it exits, it'll update the
    // gx, gy, and gz variables with the most current data.
    imu.readGyro();
  }
  if ( imu.accelAvailable() )
  {
    // To read from the accelerometer, first call the
    // readAccel() function. When it exits, it'll update the
    // ax, ay, and az variables with the most current data.
    imu.readAccel();
  }
  if ( imu.magAvailable() )
  {
    // To read from the magnetometer, first call the
    // readMag() function. When it exits, it'll update the
    // mx, my, and mz variables with the most current data.
    imu.readMag();
  }
    //int8_t Temp = bno.getTemp();
    P = getPressure();
    a = pressure.altitude(P,baseline);

    
    
//    Serial.print(P);
//    Serial.print(" ");
//    Serial.print(Accel.x());
//    Serial.print(" ");
//    Serial.print(Accel.y());
//    Serial.print(" ");
//    Serial.print(Accel.z());
//    Serial.print(" ");
//    Serial.print(a);
//    Serial.print(" ");
//    Serial.print(timer);
//    Serial.print(" ");
//    Serial.println(Temp);


   
//   if (GPS.fix)
//   {
//      dtostrf(GPS.latitude,4,2,Telem);
//      Telem[7] = ' ';
//      dtostrf(GPS.longitude,5,2,Telem+8);
//      else
      {
//        for (int i = 1; i <= 13; i++)
//        {
//          Telem[i] = '0';
//        }
      }
//   }


//  for(int i=0;i<35;i++)
//  {
//    Telem[i]='a';
//    Serial.print(Telem[i]);
//  }
  
  dtostrf(a,7,1,Telem);
  Telem[7] = ' ';
//  dtostrf(P,6,1,Telem+8);
//  Telem[14] = ' ';
  dtostrf(2345 ,4,2,Telem+8);
  Telem[12] = ' ';
  dtostrf(0000 ,4,2,Telem+13);
  Telem[17] = ' ';
//  dtostrf(imu.calcAccel(imu.ax),5,1,Telem+36);
//  Telem[17] = ' ';
//  dtostrf(imu.calcAccel(imu.ay),5,1,Telem+18);
//  Telem[23] = ' ';
//  dtostrf(millis()/1000.0,8,3,Telem+24);

  
  dtostrf(imu.calcGyro(imu.gx),5,1,Telem+18);
  Telem[23] = ' ';
  dtostrf(imu.calcGyro(imu.gy),5,1,Telem+24);
  Telem[29] = ' ';
  dtostrf(imu.calcGyro(imu.gz),5,1,Telem+30);
  Telem[35] = ' ';
  dtostrf(imu.calcAccel(imu.ax),5,1,Telem+36);
  Telem[41] = ' ';
  dtostrf(imu.calcAccel(imu.ay),5,1,Telem+42);
  Telem[47] = ' ';
  dtostrf(imu.calcAccel(imu.az),5,1,Telem+48);
  Telem[53] = ' ';
  dtostrf(imu.calcMag(imu.mx),5,1,Telem+54);
  Telem[59] = ' ';
  dtostrf(imu.calcMag(imu.my),5,1,Telem+60);
  Telem[65] = ' ';
  dtostrf(imu.calcMag(imu.mz),5,1,Telem+66);
  Telem[71] = ' ';
  dtostrf(millis()/1000.0,8,3,Telem+72);

//Serial.println(Telem);
 
Serial.print(a);
Serial.print(" ");
Serial.print(P);
Serial.print(" ");
Serial.print(GPS.latitude);
Serial.print(" ");
Serial.print(GPS.longitude);
Serial.print(" ");
Serial.print(imu.calcGyro(imu.gx), 2);
Serial.print(" ");
Serial.print(imu.calcGyro(imu.gy), 2);
Serial.print(" ");
Serial.print(imu.calcGyro(imu.gz), 2);
Serial.print(" ");
Serial.print(imu.calcAccel(imu.ax), 2);
Serial.print(" ");
Serial.print(imu.calcAccel(imu.ay), 2);
Serial.print(" ");
Serial.print(imu.calcAccel(imu.az), 2);
Serial.print(" ");
Serial.print(imu.calcMag(imu.mx), 2);
Serial.print(" ");
Serial.print(imu.calcMag(imu.my), 2);
Serial.print(" ");
Serial.print(imu.calcMag(imu.mz), 2);
Serial.print(" ");
Serial.println(millis());
//Serial.println(Telem);




 
//  dtostrf(GPS.latitude,4,2,Telem);
//  Telem[7] = ' ';
//  dtostrf(GPS.longitude,5,2,Telem+8);
//  Telem[14] = ' ';
//  dtostrf(timer,7,0,Telem+15);
//  Telem[21] = ' ';
//  dtostrf(Accel.x(),2,2,Telem+22);
//  Telem[25] = ' ';
//  dtostrf(Accel.y(),2,2,Telem+26);
//  Telem[29] = ' ';
//  dtostrf(Accel.z(),2,2,Telem+30);
//  Telem[33] = ' ';
//  dtostrf(a,3,1,Telem+34);
//  Telem[37] = ' ';
//  dtostrf(P,2,2,Telem+38);
//  Telem[41] = ' ';
//  dtostrf(Temp,3,2,Telem+42);
//  Telem[46] = ' ';
//  dtostrf(GPS.altitude,4,2,Telem+47);
  
// Serial.println(Telem[5]);
 rf95.send((uint8_t *)Telem, 81);

 rf95.waitPacketSent();


  }
}

double getPressure()
{
  char status;
  double T,P,p0,a;

  // You must first get a temperature measurement to perform a pressure reading.
  
  // Start a temperature measurement:
  // If request is successful, the number of ms to wait is returned.
  // If request is unsuccessful, 0 is returned.

  status = pressure.startTemperature();
  if (status != 0)
  {
    // Wait for the measurement to complete:

    delay(status);

    // Retrieve the completed temperature measurement:
    // Note that the measurement is stored in the variable T.
    // Use '&T' to provide the address of T to the function.
    // Function returns 1 if successful, 0 if failure.

    status = pressure.getTemperature(T);
    if (status != 0)
    {
      // Start a pressure measurement:
      // The parameter is the oversampling setting, from 0 to 3 (highest res, longest wait).
      // If request is successful, the number of ms to wait is returned.
      // If request is unsuccessful, 0 is returned.

      status = pressure.startPressure(3);
      if (status != 0)
      {
        // Wait for the measurement to complete:
        delay(status);

        // Retrieve the completed pressure measurement:
        // Note that the measurement is stored in the variable P.
        // Use '&P' to provide the address of P.
        // Note also that the function requires the previous temperature measurement (T).
        // (If temperature is stable, you can do one temperature measurement for a number of pressure measurements.)
        // Function returns 1 if successful, 0 if failure.

        status = pressure.getPressure(P,T);
        if (status != 0)
        {
          return(P);
        }
       // else Serial.println("error retrieving pressure measurement\n");
      }
     // else Serial.println("error starting pressure measurement\n");
    }
  //  else Serial.println("error retrieving temperature measurement\n");
  }
  //else Serial.println("error starting temperature measurement\n");
}
