#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <RH_RF95.h>
#include <Quaternion.h>
 
#define RFM95_CS 5
#define RFM95_RST 15
#define RFM95_INT 4

 #define GPSECHO  false
 
// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 915.0
 
// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);
 
// Blinky on receipt
#define LED 2

//IMU Library
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

//GPS Library
#include <Adafruit_GPS.h>
//#include <SoftwareSerial.h>

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
//SoftwareSerial mySerial(8, 7);

//Baro Alt
Adafruit_BMP280 bmp; // I2C
//Adafruit_BMP280 bmp(BMP_CS); // hardware SPI
//Adafruit_BMP280 bmp(BMP_CS, BMP_MOSI, BMP_MISO,  BMP_SCK);

double ee,gx,gy,gz,ax,ay,az,mx,my,mz,asax,asay,asaz,x,y,z;
long double start,finish,dt;
//double start = 0.0;
double val[4];

double qa;
double qb;
double qc;
double qd;

double val0;
double val1;
double val2;
double val3;

double Gx;
double Gy;
double Gz;
double Ax;
double Ay;
double Az;

imu::Vector<3> Gyro;
imu::Vector<3> Accel;
imu::Vector<3> Mag;

void setup(){
    Wire.begin();
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
  qa = 1;
  qb = 0;
  qc = 0;
  qd = 0;

  val0 = 0;
  val1 = 0;
  val2 = 0;
  val3 = 0;
}
uint32_t timer = (uint32_t)millis;
void Start()
{
  start = (long double)micros();
}

void getdata(){
   imu::Vector<3> Gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
   imu::Vector<3> Accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
   imu::Vector<3> Mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
   Gx=(double)Gyro.x();
   Gy=(double)Gyro.y();
   Gz=(double)Gyro.z();
   Ax=(double)Accel.x();
   Ay=(double)Accel.y();
   Az=(double)Accel.z();
   //For python Computation
   //delay(10);
   Serial.print(Gx);
   Serial.print(" ");
   Serial.print(Gy);
   Serial.print(" ");
   Serial.print(Gz);
   Serial.print(" ");
   Serial.print(Ax);
   Serial.print(" ");
   Serial.print(Ay);
   Serial.print(" ");
   Serial.print(Az);
   Serial.print(" ");
}

/*void setup_MPU9250()
{
  Wire.beginTransmission(0x68); //MPU9250 adress
  Wire.write(0x6B);
  Wire.write(B00000000); //enables wakeup mode
  Wire.endTransmission();
}*/

void QCF(double gx, double gy, double gz, double ax, double ay, double az, double alpha)
{ 
  gx=gx*3.14/180;
  gy=gy*3.14/180;
  gz=gz*3.14/180;
  
  double q_acca, q_accb, q_accc, q_accd, q_maga, q_magd;
  
  double wa = 0.0; //angular velocity vector represented as a pure quaternion
  double wb = gx;
  double wc = gy;
  double wd = gz;
  
  double qdota = (wa*qa - wb*qb - wc*qc - wd*qd) * -0.5;//estimated state derivative quaternion
  double qdotb = (wa*qb + wb*qa + wc*qd - wd*qc) * -0.5;
  double qdotc = (wa*qc - wb*qd + wc*qa + wd*qb) * -0.5;
  double qdotd = (wa*qd + wb*qc - wc*qb + wd*qa) * -0.5;

  double qwa = qa + qdota * dt; //updated estimated state quaternion
  double qwb = qb + qdotb * dt;
  double qwc = qc + qdotc * dt;
  double qwd = qd + qdotd * dt;

  double norm = sqrt(qwa*qwa + qwb*qwb + qwc*qwc + qwd*qwd);

   if (norm==0){
    norm=0.01;} 
  qwa = qwa/norm;
  qwb = qwb/norm;
  qwc = qwc/norm;
  qwd = qwd/norm;

  double norma= sqrt(ax*ax + ay*ay + az*az);
   if (norma==0){
    norma=0.01;} 
  ax = ax/norma;
  ay = ay/norma;
  az = az/norma;
  
  double aa = 0; //accelerometer readings in pure quaternion form
  double ab = ax;
  double ac = ay;
  double ad = az;
  
  double normb = sqrt(ab*ab + ac*ac + ad*ad);
   if (normb==0){
    normb=0.01;} 
  ab = ab/normb;
  ac = ac/normb;
  ad = ad/normb;
  
  double gb = (2*qwa*qwa - 1 + 2*qwb*qwb)*ab + (2*qwc*qwb + 2*qwa*qwd)*ac + (2*qwd*qwb - 2*qwa*qwc)*ad;//pure quaternion g representing predicted gravity
  double gc = (2*qwb*qwc - 2*qwa*qwd)*ab + (2*qwa*qwa - 1 + 2*qwc*qwc)*ac + (2*qwd*qwc + 2*qwa*qwb)*ad;
  double gd = (2*qwb*qwd + 2*qwa*qwc)*ab + (2*qwc*qwd - 2*qwa*qwb)*ac + (2*qwa*qwa - 1 + 2*qwd*qwd)*ad;//sensor frame to earth frame
  double normc = sqrt(gb*gb + gc*gc + gd*gd); 
   if (normc==0){
    normc=0.01;} 
  gb = gb/normc;
  gc = gc/normc;
  gd = gd/normc;
  if(gd >= 0)
  {
    q_acca = sqrt((gd + 1)*0.5);
    q_accb = -gc/2*q_acca;
    q_accc = gb/2*q_acca;
    q_accd = 0.0;

    double normd = sqrt(q_acca*q_acca + q_accb*q_accb + q_accc*q_accc + q_accd*q_accd);
     if (normd==0){
    normd=0.01;} 
    q_acca = q_acca/normd;
    q_accb = q_accb/normd;
    q_accc = q_accc/normd;
    q_accd = q_accd/normd;
   }
  else
  {
    q_acca = -gc/sqrt(2 - 2*gd);
    q_accb = sqrt((1 - gd)/2);
    q_accc = 0.0;
    q_accd = gb/sqrt(2 - 2*gd);
    
    double norme = sqrt(q_acca*q_acca + q_accb*q_accb + q_accc*q_accc + q_accd*q_accd);
     if (norme==0){
    norme=0.01;} 
    q_acca = q_acca/norme;
    q_accb = q_accb/norme;
    q_accc = q_accc/norme;
    q_accd = q_accd/norme;
  }
  
  if(q_acca > 0.9)
  {
    q_acca = (1 - alpha) + alpha * q_acca; //using qI(1,0,0,0)... then simplifying to take up less memory
    q_accb = alpha * q_accb;
    q_accc = alpha * q_accc;
    q_accd = alpha * q_accd;

    double normf = sqrt(q_acca*q_acca + q_accb*q_accb + q_accc*q_accc + q_accd*q_accd);
     if (normf==0){
    normf=0.01;} 
    q_acca = q_acca/normf;
    q_accb = q_accb/normf;
    q_accc = q_accc/normf;
    q_accd = q_accd/normf;
  }
  else
  {
    double angle = acos(q_acca);
    q_acca = sin((1 - alpha) * angle)/sin(angle) + sin(alpha * angle) * q_acca/sin(angle);
    q_accb = sin(alpha * angle) * q_accb/sin(angle);
    q_accc = sin(alpha * angle) * q_accc/sin(angle);
    q_accd = sin(alpha * angle) * q_accd/sin(angle);
  }
  
  qa = qwa * q_acca - qwb * q_accb - qwc * q_accc - qwd * q_accd;
  qb = qwa * q_accb + qwb * q_acca + qwc * q_accd - qwd * q_accc;
  qc = qwa * q_accc - qwb * q_accd + qwc * q_acca + qwd * q_accb;
  qd = qwa * q_accd + qwb * q_accc - qwc * q_accb + qwd * q_acca;
  
  double normg = sqrt(qa*qa + qb*qb + qc*qc + qd*qd);
   if (normg==0){
    normg=0.01;} 
  qa = (double)qa/normg;
  qb = (double)qb/normg;
  qc = (double)qc/normg;
  qd = (double)qd/normg;

  val[0] = 2*acos(qa); //gets the angle of rotation
  if(val[0] <= 0.01) 
  {
    val[1] = qb; // gets x the axes of rotation
    val[2] = qc; // gets y the axes of rotation
    val[3] = qd; // gets z the axes of rotation
  }
  else
  {
    val[1] = qb/sin(val[0]/2); //send over serial port to java grahics program
    val[2] = qc/sin(val[0]/2);
    val[3] = qd/sin(val[0]/2);
    
  }
  Serial.print(val[0]);
  Serial.print(" ");
  Serial.print(val[1]);
  Serial.print(" ");
  Serial.print(val[2]);
  Serial.print(" ");
  Serial.println(val[3]);
    
}
void End(){
  finish = (long double)micros();
  dt = (long double)((finish - start)/(1000000.0));
  Serial.println((double)dt);
}
void loop(){
 Start();
 getdata();
 //QCF(Gx, Gy, Gz, Ax, Ay, Az,0.15);
 End();
}
