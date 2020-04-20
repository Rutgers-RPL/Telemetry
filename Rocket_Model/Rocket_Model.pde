PShape s;

void setup() {
  size(800, 800, P3D);
  // The file "bot.obj" must be in the data folder
  // of the current sketch to load successfully
  s = loadShape("shuttle.obj");
}

void draw() {
  background(204);
  smooth();
  lights();
  noFill();
  translate(width/2, height/2);
  shape(s, 0, 0);
  scale(30,30,30);
  shape(s,0,0);
}
