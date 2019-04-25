//This code will be used for calibrating the 
// accelerometer in all directions (gives the coefficients m and b)
//
// A1 = Z output
// A2 = Y output
// A3 = X output
// We will be using a moving average until we reach equilibrium

//C:\Users\Harris\AppData\Local\VirtualStore\Program Files (x86)\PuTTy

//How to use:
//There are arrows labeled on each of the legs of the calibrator
//First place the arrows with the the x direction pointing up.
//Wait around a few seconds and then hit the s key. 

double X[100];
double Y[100];
double Z[100];
int i = 0;
int  axis = 0;
bool compute = false;
long timeRead;
bool done; 

void setup() {
  Serial.begin(9600); 
  analogReadResolution(12);
  timeRead = millis();
  done = true;
}

//Calibrates each value for 3 seconds. After that is done
// we update the values for X 
void loop() {
  computeAvg();
  if(millis()-timeRead < 3000 && done == false){
    if(axis = 0){
      Serial.print("Calibrating X: ");
      X[i%100] = analogRead(A3);
    }
    else if(axis = 1){
      Serial.print("Calibrating Y: ");
      Y[i%100] = analogRead(A2);
    }
    else if(axis = 2){
      Serial.print("Calibrating Z: ");
      Z[i%100] = analogRead(A1);
    }
    i = i + 1;
  } else{
    //done = true;  
    computeAvg();
    i = 0;
    Serial.print(X[0]);
  } 
}

void computeAvg(){
  if(Serial.available() && done == true ){
    char user_input = Serial.read();
    if(user_input == 'c'){
      //compute = !compute;
      axis = axis + 1;
      done = false;
    }
  }
}

//double movingAvg(double[] valsArray){
//  double[] vals = valsArray;
//  double sum = 0;
//  for(unsigned int a = 0; a<100; a = a+1){
//    if( vals[a] != NULL ){
//      sum = sum + vals[a]; 
//    }
//    }
//  sum = sum/100;
//  return sum;
//       
//  }

