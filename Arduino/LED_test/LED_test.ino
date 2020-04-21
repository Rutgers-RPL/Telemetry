void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);
pinMode(5, OUTPUT);
pinMode(7, OUTPUT);
}
int i=0;
void loop() {
  if (i<=2)
    Serial.print("hello");
    
    i=i+1;
  if (i>=3)   
    Serial.print("complete");
    
    i=i+1;
}
