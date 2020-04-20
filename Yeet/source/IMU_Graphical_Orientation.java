import processing.core.*; 
import processing.data.*; 
import processing.event.*; 
import processing.opengl.*; 

import processing.serial.*; 
import java.util.*; 

import java.util.HashMap; 
import java.util.ArrayList; 
import java.io.File; 
import java.io.BufferedReader; 
import java.io.PrintWriter; 
import java.io.InputStream; 
import java.io.OutputStream; 
import java.io.IOException; 

public class IMU_Graphical_Orientation extends PApplet {

public static void main(String[] args){
	setup();
	while(true){
		draw();
	}
}

PShape s;
//Serial myPort;
String myString = null;
float x, y, z, angle, vertical_angle;

String[] lines=null;
int counter=-1;

public void setup()
{
  lines=loadStrings(System.in);
  //String portName = Serial.list()[2];
  //myPort = new Serial(this, portName, 9600);
  
  //myPort.bufferUntil(10);
  s=loadShape("shuttle.obj");
}
public void draw()
{
  delay(20);
  counter++;
  if(counter < lines.length)
  {
      String line = lines[counter];
      String[] numbers = line.split("\\W+");
      angle=Float.parseFloat(numbers[0]);
      x=Float.parseFloat(numbers[1]);
      y=Float.parseFloat(numbers[2]);
      z=Float.parseFloat(numbers[3]);
      
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
  rotate(angle, x , y, z);
  shape(s, 0, 0);
  scale(30,30,30);
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
  public void settings() {  size(800,800,P3D); }
  static public void main(String[] passedArgs) {
    String[] appletArgs = new String[] { "IMU_Graphical_Orientation" };
    if (passedArgs != null) {
      PApplet.main(concat(appletArgs, passedArgs));
    } else {
      PApplet.main(appletArgs);
    }
  }
}
