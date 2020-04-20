import processing.serial.*;
import java.util.*;
import processing.net.*;
PShape s;
//Serial myPort;
String myString = null;
float x, y, z, angle, vertical_angle;
BufferedReader reader;
String line = "";
Scanner scan = new Scanner(System.in);
String[] numbers=null;
int counter=-1;
Client myClient;

void setup()
{
  myClient = new Client(this, "localhost", 12345);
  size(800,800,P3D);
  s=loadShape("shuttle.obj");
  reader = createReader("Quat.txt");    
}
void draw()
{
  if(myClient.available() >0){
    line = myClient.readStringUntil('\n');
    println(line);
    numbers = line.split(" ");
    //print(numbers[0]);
    angle=Float.parseFloat(numbers[0]);
    //print(numbers[1]);
    x=Float.parseFloat(numbers[1]);
    //print(numbers[2]);
    y=-Float.parseFloat(numbers[2]);
    //println(numbers[3]);
    z=Float.parseFloat(numbers[3]);
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
    scale(40,40,40);
    shape(s,0,0);
    popMatrix();
  }
}
