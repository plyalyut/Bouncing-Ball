//kappa, gamma, K, omega, acceleration, and initial time 
float kap = 1.03;
float gamma = 7.70309924565;
//float gamma = 10;
int scale = 22;
float omega = 0.0638619521;
//float omega = 0;
float K = kap/sq(scale);
float a = 0.1;
float t = 0;

//Setting the initial conditions x = (1,100), v = 0 , g = 90 units/second
PVector pos = new PVector(10,50);
PVector vel = new PVector(0,0);
PVector acc = new PVector(0,-a);
PVector bounceX = new PVector(pos.x,pos.y); //used in Newton's method for accurate bouncing
PVector bounceV = new PVector(vel.x,vel.y); //used in Newton's method for accurate bouncing
float t0 = t; //used in Newton's method
//Arraylist to store positions of the ball at each time step
//in order to create the trail
ArrayList<PVector> posList = new ArrayList<PVector>();

//coefficients of restitution
float a_T = 0.6;
float a_N = 0.975;

//bounce #, trail size, stepSz
float bounce = 0;
int trail = 1;
float stepSz = 1;

void setup(){
  
  size(600,600);
  background(255);
  //frameRate(30);
}

void draw(){
  background(255);
  noFill();
  
  //to get our coordinate system in the right plane
  translate(width/2, 3*height/4);
  scale(1,-1);
  
  for(float i=-300; i<300; i+=0.5){
    float y = K*sq(i)+gamma*sin(omega*t);
    fill(0,0,0,80); //coloring for our parabola
    ellipse(i,y,2,2);
  }
  //Implementation of eulers method
  //pos.add(vel.mult(stepSz));
  //vel.mult(1/stepSz);
  //vel.add(acc.mult(stepSz));
  //acc.mult(1/stepSz);
  //
  
  pos = new PVector(bounceX.x+bounceV.x*(t-t0),bounceX.y+bounceV.y*(t-t0)-0.5*a*sq(t-t0)); 
  vel = new PVector(bounceV.x,bounceV.y-a*(t-t0));
  posList.add(new PVector(pos.x, pos.y));
  
  //this is to get the trails to work
  for(int i = max(0, posList.size()-trail); i<posList.size(); i++){
      noStroke();
      fill(100,255,255,70);
      ellipse(posList.get(i).x,posList.get(i).y,4,4);
  }
  
  ellipse(pos.x, pos.y, 5,5);
  
  //checking for bounces and making sure no chattering
  if(pos.y<=K*sq(pos.x)+gamma*sin(omega*t) && t-bounce>stepSz*5){
    //Run newtons method and get new x,y,vx,vy,t
    println("Original Time: " + str(t));
    float[] pvt = NewtonMethod(t,t0, bounceX, bounceV);
    
    //Update the time of last bounce to be 
    //the new time given by newton's method
    //also gets the new bounce coordinates
    float t1 = pvt[4];
    t0 = t1;
    bounceX = new PVector(pvt[0],pvt[1]);
    
    vel = new PVector(pvt[2],pvt[3]);
    
    pos.x = bounceX.x; 
    pos.y = bounceX.y;
    
    PVector T_hat = new PVector(1, 2*K*pos.x).normalize();
    PVector N_hat = new PVector(-2*K*pos.x, 1).normalize();
    float[][] P = {{T_hat.x, N_hat.x},{T_hat.y, N_hat.y}};
    float parVel = gamma*omega*cos(omega*t1);
    vel.y = vel.y - parVel;
    vel = outVec(P,vel);
    //print(t1);
    println("New Time "+str(t1));
    //println("New Time "+str(xyt.z));
    //t0 = t1;
    //bounceX = new PVector(pos.x, pos.x);
    bounceV = new PVector(vel.x,vel.y);
    bounce = t1;
    t = t1;
  }
  
  t+=stepSz;
}

PVector outVec(float[][] P, PVector v){
  float det = P[1][1]*P[0][0]-P[1][0]*P[0][1];
  float[][] inverse = {{P[1][1]/det, -P[0][1]/det},{-P[1][0]/det, P[0][0]/det}};
  PVector v_prime = matvecmul(inverse, v);
  float[][] A = {{a_T, 0},{0, -a_N}};
  v_prime = matvecmul(A, v_prime);
  return matvecmul(P,v_prime);
}

PVector matvecmul(float[][] P, PVector v){
  //Matrix multiply a 2x2 matrix and vector
  return new PVector(P[0][0]*v.x+P[0][1]*v.y, P[1][0]*v.x+P[1][1]*v.y);
}

float[] NewtonMethod(float t, float t0, PVector x0, PVector v0){
  float t1 = t;
  float f = x0.y + v0.y*(t-t0)-0.5*a*sq(t-t0)-K*sq(x0.x+v0.x*(t-t0))-gamma*sin(omega*t);
  float fprime = v0.y-a*(t-t0)-K*v0.x*(x0.x+v0.x*(t-t0))-gamma*omega*cos(omega*t);
  //run Newton's Method
  for(int i=0; i<10; i++){
    t1 = t1-f/fprime;
    f = x0.y + v0.y*(t1-t0)-0.5*a*sq(t1-t0)-K*sq(x0.x+v0.x*(t1-t0))-gamma*sin(omega*t1);
    fprime = v0.y-a*(t1-t0)-K*v0.x*(x0.x+v0.x*(t1-t0))-gamma*omega*cos(omega*t1);
  }
  //println("New Time " + str(t1));
  float x = x0.x+v0.x*(t1-t0);
  float y = x0.y+v0.y*(t1-t0)-0.5*a*sq(t1-t0);
  float[] bounceParams = new float[5];
  bounceParams[0] = x; //new x position
  bounceParams[1] = y; //new y position
  bounceParams[2] = v0.x; //horizontal vel stays the same
  bounceParams[3] = v0.y-a*(t1-t0); //new vertical vel
  bounceParams[4] = t1;
  
  return bounceParams;
}