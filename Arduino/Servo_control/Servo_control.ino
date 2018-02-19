// Sweep
// by BARRAGAN <http://barraganstudio.com> 
// This example code is in the public domain.


#include <Servo.h> 

Servo servo1, servo2, servo3, servo4, servo5, servo6;  // create servo object to control a servo 
// a maximum of eight servo objects can be created 
float theta = 135.0, theta_m;

int pos = 0;    // variable to store the servo position 

void setup() 
{   
  Serial.begin(9600);
  Serial.setTimeout(15);

  servo1.attach(2,500,2500);  // attaches the servo on pin 9 to the servo object 
  servo2.attach(3,95,565);
  servo3.attach(4,95,565);
  servo4.attach(5,95,565);
  servo5.attach(6,95,565);
  servo6.attach(7,95,565);

} 


void loop() 
{ 
  if(Serial.available())
  {
    theta = Serial.parseFloat();
  }

    theta_m = map(theta,0,270,0,180);

  servo1.write(180-theta_m);              
  servo2.write(theta_m);
  servo3.write(180-theta_m);
  servo4.write(theta_m);
  servo5.write(180-theta_m);
  servo6.write(theta_m);

} 

