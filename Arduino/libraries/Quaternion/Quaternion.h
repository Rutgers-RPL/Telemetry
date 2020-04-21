#ifndef ql
#define ql

#if (ARDUINO >= 100)
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif


class Quaternion
{
  public:
    // Variables
    float q1, q2, q3, q4;
    // Constructors
    Quaternion(float w, float x, float y, float z);

    // Methods
    Quaternion multipliedBy(Quaternion q);
    Quaternion conjugate();
    Quaternion normalize();
    Quaternion rotateVectorByQuaternion(Quaternion q);
    float getAngle();
    float get_xAxis();
    float get_yAxis();
    float get_zAxis();
};

#endif
