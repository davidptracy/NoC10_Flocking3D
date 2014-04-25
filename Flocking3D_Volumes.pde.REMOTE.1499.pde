import peasy.*;
import processing.opengl.*;

PeasyCam cam;
PMatrix3D currCameraMatrix;
PGraphics3D g3;

import controlP5.*;
ControlP5 sliders;
float sepVal, aliVal, cohVal;
float radius, maxForce, maxSpeed;
float desiredSeparation, alignThreshold, cohesionThreshold;

float camRot;
Flock flock;

void setup() {
  size(800, 800, OPENGL);
  smooth();

  sepVal = 8.5;
  aliVal = 4.5;
  cohVal = 8;

  maxForce = .08;
  maxSpeed = 3;
  desiredSeparation = 200;
  alignThreshold = 175;
  cohesionThreshold = 50;



  // New PeasyCam
  cam = new PeasyCam(this, 800);
  g3 = (PGraphics3D)g;

  // New ControlP5 element
  sliders = new ControlP5(this);
  //(label, min, max, startVal, xLoc, yLoc, sliderW, sliderH)
  sliders.addSlider("sepVal", 0, 10, sepVal, 20, 100, 100, 10);
  sliders.addSlider("aliVal", 0, 10, aliVal, 20, 120, 100, 10);
  sliders.addSlider("cohVal", 0, 10, cohVal, 20, 140, 100, 10);
  //  sliders.addSlider("radius", 0, 10, 5, 20, 160, 100, 10);
  sliders.addSlider("maxForce", 0, .2, maxForce, 20, 180, 100, 10);
  sliders.addSlider("maxSpeed", 0, 10, maxSpeed, 20, 200, 100, 10);
  sliders.addSlider("desiredSeparation", 0, 250, desiredSeparation, 20, 220, 100, 10);
  sliders.addSlider("alignThreshold", 0, 250, alignThreshold, 20, 240, 100, 10);
  sliders.addSlider("cohesionThreshold", 0, 125, cohesionThreshold, 20, 260, 100, 10);
  sliders.setAutoDraw(false);



  //  Initializes the flock with an initial set of boids
  int startCount = 25;
  flock = new Flock();
  for (int i=0; i<startCount; i++) {
    Boid b = new Boid(PVector.random3D(), radius);
    flock.addBoid(b);
  }
}

void draw() {

  background(0);

  //  camera(400*cos(camRot), 400*sin(camRot), 400, 0, 0, 0, 0, 1, 0);
  flock.run();


  colorMode(RGB);
  // Axis for world coordinate system
//  strokeWeight(3);
  stroke(255, 0, 0);
  line(0, 0, 0, 50, 0, 0);
  stroke(0, 255, 0);
  line(0, 0, 0, 0, 50, 0);
  stroke(0, 0, 255);
  line(0, 0, 0, 0, 0, 50);


  pushMatrix();
  translate(0, 0, 00);
  noFill();
  strokeWeight(1);
  stroke(0);
  box(500);
  popMatrix();

  gui();

  if (keyPressed) flock.addBoid(new Boid(PVector.random3D(), radius));
}

//void keyPressed() {
//  flock.addBoid(new Boid(0,0,0, radius));
//}

void gui() {
  currCameraMatrix = new PMatrix3D(g3.camera);
  camera();
  sliders.draw();
  g3.camera = currCameraMatrix;
}

