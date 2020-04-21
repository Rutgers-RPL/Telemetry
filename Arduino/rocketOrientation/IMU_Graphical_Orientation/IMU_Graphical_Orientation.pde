import processing.serial.*;
import java.util.*;
PShape s;
//Serial myPort;
String myString = null;
float x, y, z, angle, vertical_angle;
BufferedReader reader;
String line;

String[] lines=null;
int counter=-1;

void setup()
{
  lines=loadStrings("Quat.txt");
  //String portName = Serial.list()[2];
  //myPort = new Serial(this, portName, 9600);
  size(800,800,P3D);
  //myPort.bufferUntil(10);
  s=loadShape("L3_Assembly2.obj");
  reader = createReader("Quat.txt");    
}
void draw()
{
  counter++;
  if(counter < lines.length)
  {
      String line = lines[counter];
      String[] numbers = split(line, " ");
      angle=Float.parseFloat(numbers[0]);
      x=Float.parseFloat(numbers[1]);
      y=Float.parseFloat(numbers[2]);
      z=Float.parseFloat(numbers[3]);
      print(" ");
      print(z);
  } else {
      exit();
      //end
  }
  background(0);
  smooth();
  lights();
  fill(250);
  stroke(75);
  pushMatrix();
  translate(400,400,0);
  rotateX(PI/2);
  rotate(angle, x , y, z);
  shape(s,0, 0);
  scale(60,60,60);
  shape(s,0,0);
  popMatrix();
}

/*
void serialEvent(Serial myPort)
{
 while(myPort.available() > 0)
 {
  myString = myPort.readStringUntil(10); 
  if(myString != null)
  {
     myString = trim(myString);
     float[] val = float(split(myString, ","));
      if(val.length == 4)
      {
       angle = val[0];
       z = val[1];//-,+
       x = val[2];//-,+
       y = -val[3];//-,-
       //vertical_angle = val[4];
      }    
  }
 }
}
*/
