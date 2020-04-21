
#if (ARDUINO >= 100)
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include "Quaternion.h"

Quaternion::Quaternion(float w, float x, float y, float z) // Constructor 
{
  q1 = w;
  q2 = x;
  q3 = y;
  q4 = z;
}

Quaternion Quaternion::conjugate() // finds conjugate of Quaternion
{
  q2 = -q2;
  q3 = -q3;
  q4 = -q4;
  return Quaternion(q1, q2, q3, q4);
}


Quaternion Quaternion::normalize()
{
  float norm = sqrt(q1*q1 + q2*q2 + q3*q3 + q4*q4);
  q1 = q1/norm;
  q2 = q2/norm;
  q3 = q3/norm;
  q4 = q4/norm;
  return Quaternion(q1, q2, q3, q4);
}

Quaternion Quaternion::multipliedBy(Quaternion q) // multiply by quaternion q
{
   q1 = q1 * q.q1 - q2 * q.q2 - q3 * q.q3 - q4 * q.q4;
   q2 = q1 * q.q2 + q2 * q.q1 + q3 * q.q4 - q4 * q.q3;
   q3 = q1 * q.q3 - q2 * q.q4 + q3 * q.q1 + q4 * q.q2;
   q4 = q1 * q.q4 + q2 * q.q3 - q3 * q.q2 + q4 * q.q1;
   
   return Quaternion(q1, q2, q3, q4);  
}

float Quaternion::getAngle() // getter method
{
  float angle = 2*acos(q1); //gets the angle of rotation
  return angle;
}

float Quaternion::get_xAxis() // getter method
{
  float s = sqrt(1 - q1*q1);
  if(s < 0.001)
  {
    return q2;
  }
  else
  {
    q2 = q2/s;
    return q2;
  }
}

float Quaternion::get_yAxis() // getter method
{
  float s = sqrt(1 - q1*q1);
  if(s < 0.001)
  {
    return q3;
  }
  else
  {
    q3 = q3/s;
    return q3;
  }
}

float Quaternion::get_zAxis() // getter method
{
  float s = sqrt(1 - q1*q1);
  if(s < 0.001)
  {
    return q4;
  }
  else
  {
    q4 = q4/s;
    return q4;
  }
}

Quaternion Quaternion::rotateVectorByQuaternion(Quaternion q)
{ 
   q1 = 0.0;
   q2 = (q.q1*q.q1 + q.q2*q.q2 - q.q3*q.q3 - q.q4*q.q4)*q2 + 2*(q.q2*q.q3 - q.q1*q.q4)*q3 + 2*(q.q2*q.q4 + q.q1*q.q3)*q4;
   q3 = 2*(q.q2*q.q3 + q.q1*q.q4)*q2 + (q.q1*q.q1 - q.q2*q.q2 + q.q3*q.q3 - q.q4*q.q4)*q3 + 2*(q.q3*q.q4 - q.q1*q.q2)*q4;
   q4 = 2*(q.q2*q.q4 - q.q1*q.q3)*q2 + 2*(q.q3*q.q4 + q.q1*q.q2)*q3 + (q.q1*q.q1 - q.q2*q.q2 - q.q3*q.q3 + q.q4*q.q4)*q4;
   
   return Quaternion(q1,q2,q3,q4);
}
