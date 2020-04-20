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

public class Serial_Vis extends PApplet {



PShape s;
Serial myPort;
String myString = null;
float x, y, z, angle, vertical_angle;
Scanner scan = new Scanner(System.in);
public void setup()
{
  String portName = Serial.list()[0];
  myPort = new Serial(this, portName, 9600);
  
  myPort.bufferUntil(10);
  s=loadShape("shuttle.obj");
}

public void draw()
{
  System.out.println(scan.nextLine());
  background(0);
  smooth();
  lights();
  fill(250);
  stroke(75);
  pushMatrix();
  translate(1000,500,0);
  rotate(angle, x , y, z);
  shape(s, 0, 0);
  scale(40,40,40);
  shape(s,0,0);
  popMatrix();
}

public void serialEvent(Serial myPort)
{
 while(myPort.available() > 0)
 {
  myString = myPort.readStringUntil(10); 
  if(myString != null)
  {
     myString = trim(myString);
     float[] val = PApplet.parseFloat(split(myString, " "));
      if(val.length == 4)
      {
       angle = val[0];
       z = val[2];//-,+
       x = -val[1];//-,+
       y = -val[3];//-,-
       //vertical_angle = val[4];
      }    
  }
 }
}
  public void settings() {  size(800,800,P3D); }
  static public void main(String[] passedArgs) {
    String[] appletArgs = new String[] { "Serial_Vis" };
    if (passedArgs != null) {
      PApplet.main(concat(appletArgs, passedArgs));
    } else {
      PApplet.main(appletArgs);
    }
  }
}
