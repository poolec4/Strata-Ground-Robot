// Sweep
// by BARRAGAN <http://barraganstudio.com> 
// This example code is in the public domain.


#include <Servo.h> 

Servo servo1, servo2, servo3, servo4, servo5, servo6;  // create servo object to control a servo 
// a maximum of eight servo objects can be created 
float theta[6] = [135.0,135.0,135.0,135.0,135.0,135.0], theta_m[6];
float t;

int pos = 0;    // variable to store the servo position 

char incomming_byte;
int temp[10];
int i, val_num, dec_index;
float val;

void setup() 
{   
  Serial.begin(115200);
  Serial.setTimeout(15);

  servo1.attach(2,500,2500);  // attaches the servo on pin 9 to the servo object 
  servo2.attach(3,500,2500);
  servo3.attach(4,500,2500);
  servo4.attach(5,500,2500);
  servo5.attach(6,500,2500);
  servo6.attach(7,500,2500);

} 


void loop() 
{ 
  t = millis();
  Serial.flush();

  if(Serial.available())
  { 
    Serial.println("Beginning to process incomming data...");
    
    val_num = 0;
    incomming_byte = 0;
    i = 0;
    
    while(incomming_byte != '\n')
    {
      incomming_byte = Serial.read();
      Serial.println(incomming_byte);
      
      if((incomming_byte >= '0') && (incomming_byte <= '9'))
        temp(i) = (int)(incomming_byte - '0');
      
      if(incomming_byte == '.')
        dec_index = i;
      
      if(incomming_byte == ',')
      {
        for(int digit=0; digit<i; digit++)
          theta(val_num) = theta(val_num) + (float)temp(digit)*pow(10,i-digit)
        
        theta(val_num) = theta(val_num)/(10*(i-dec_index));
        Serial.print(theta(val_num));
        
        val_num++;
        i = 0;
      }
      else
      {
        i++;
      }      
     }
  }

    //theta = 135+45*sin(2*PI*0.1*t/1000);
  for(int servo_num = 0;l servo_num <6; servo_num++)
  {  
    theta_m(i) = map(theta,0,270,0,180);
  }
  
  servo1.write(180-theta_m);              
  servo2.write(theta_m);
  servo3.write(180-theta_m);
  servo4.write(theta_m);
  servo5.write(180-theta_m);
  servo6.write(theta_m);

} 



