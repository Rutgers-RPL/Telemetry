/********** ANGULAR POSITION W/ SERVOS ***********

 1. Starting position is not the actual position of the servo arm. We must assume a starting position and calibrate to either end. 
 2. CCW is done by adding position from the starting position (<--)
 3. CW is done by subtracting position from the starting position (-->)
 4. Servo arm all the way to the right starts at position 0 
 5. Servo arm all the way to the left starts at position 180. 
 
*************************************************/

#include <Servo.h>

Servo myservo;  // https://www.arduino.cc/en/tutorial/sweep
// twelve servo objects can be created on most boards

int pos = 0; // Servo is initalized by assuming the servo arm position is all the way to RIGHT. 
void setup() 
{
  Serial.begin(9600); // Initializes Serial Monitor
  myservo.attach(9);  // attaches the servo on pin 2 to the servo object

  //Calibrating Servo by Turning Completely to the left. 
  Serial.println("Calibrating, please wait.");
  for (int i = 0; i < 181; i++) 
  { 
    // in steps of 1 degree
    myservo.write(pos); 
    pos = pos + 1; 
    //Serial.println(myservo.read());
    delay(50);                       // waits 100ms for the servo to reach the position
  }

  Serial.print(myservo.read()); // This is the position we start with, it is at 180 degrees referenced to the right. 
  Serial.println(" degrees reference to the right.");


  Serial.println("How to use: Typing commands in the serial monitor controls the servo.");
  Serial.println("Typing in 'A' rotates the servo 90 degrees CW, 'B'rotates 90 degrees CCW'.");
  Serial.println("Typing in 'C' resets the servo to be 180 degress from the right.");
  Serial.println("Typing in 'D' brings the motor to the right, 0 degrees from the right.");
  Serial.println("Ready to go!");
}


void loop() 
{



  //Command Servo to Move 90 degees CW
  while (Serial.available()>0)
  { 
    char com = Serial.read();
    //Serial.println(Serial.read());
    
    if (com == 'A') 
    {
      Serial.println("Now moving 90 degrees CW. Please wait.");
      for (int cwi =0; cwi <= 90; cwi++) // When the condition becomes false, the  loop ends
      { 
        pos = pos - 1;
        myservo.write(pos); 
        delay(50);                       // waits 200ms for the servo to reach the position
      }
      Serial.print(myservo.read());
      Serial.println(" degrees reference to the right.");
    }
    

    
    //Command Servo to Move 90 degrees CCW
    else if (com == 'B')
     {
        Serial.println("Now moving 90 degrees CCW. Pleast wait.");
        for (int ccwi =0; ccwi <= 89; ccwi++) // When the condition becomes false, the  loop ends
        { 
        pos = pos + 1;
        //Serial.println(pos);
        myservo.write(pos); 
        delay(50);                      
        }
        Serial.print(myservo.read());
        Serial.println(" degrees reference to the right.");
    }


    
    //Command Servo To Reset
    else if (com == 'C')
    {
      Serial.println("Now resetting position back to 180 degrees from the right. Please wait.");
      for (int li = 0; li < 181; li++) 
        { 
        myservo.write(pos); 
        pos = pos + 1; 
        delay(50);                       
        }
        Serial.print(myservo.read());
        Serial.println(" degrees reference to the right.");
     }




     //Command Servo To Go To The Right (0)
    else if (com == 'D')
    {
      Serial.println("Going to the right, 0 degrees from the right. Please wait.");
      for (int ri = 0; ri < 181; ri++) 
        { 
        myservo.write(pos); 
        pos = pos - 1; 
        delay(50);                       
        }
        Serial.print(myservo.read());
        Serial.println(" degrees reference to the right.");
     }


  }
}
