BufferedReader reader;
String line;
float x,y,z,angle;
 
void setup() {
  // Open the file from the createWriter() example
  reader = createReader("Quat.txt");    
}
 
void draw() {
  try {
    line = reader.readLine();
  } catch (IOException e) {
    e.printStackTrace();
    line = null;
  }
  if (line == null) {
    // Stop reading because of an error or file is empty
    noLoop();  
  } else {
    String[] pieces = split(line, " ");
    float angle = float(pieces[0]);
    float x = float(pieces[1]);
    float y = float(pieces[2]);
    float z = float(pieces[3]);
    print(angle);
  }
} 
