/*
  Written by Chris Poole - Spring 2018
  
  
  Sets servo angles from serial input in the form:
       "th1,th2,th3,th4,th5,th6\n"
  where the angle is in degrees.
 
 thi = 0 corresponds to the ith servo in the down position
 
 for servos [1,3,5], positive rotation is in the CW direction
 for servos [2,4,6], positive rotation is in the CCW direction
*/

#include <Servo.h> 

Servo servo1, servo2, servo3, servo4, servo5, servo6;  // create servo object to control a servo 
float theta[6] = {0,0,0,0,0,0};
float theta_m[6];
float t;

int incoming_byte = 0;
int temp[20];
int x, l, j, i, comma;
int dec_index = -1;
float val;

void setup() 
{   
  Serial.begin(115200);

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

  if(Serial.available() > 0)
  { 
    //Serial.println("Beginning to process incoming data...");
    comma = 0;

    do {
      incoming_byte = Serial.read();
      //Serial.println(incoming_byte);

      while(incoming_byte != (int)',')
      {
        if(incoming_byte == (int)'\n')
          break;

        if(incoming_byte == (int)'.')
          dec_index = i;

        if((incoming_byte >= (int)'0') && (incoming_byte <= (int)'9'))
        {
          temp[i] = incoming_byte - (int)'0';
          i++;
        }

        incoming_byte = Serial.read();
      }

      j = i;
      i = 0;

      theta[comma] = 0;

      for(x=0, l=j; x<j; x++,l--)
      {
        theta[comma] = theta[comma] + temp[x]*pow(10,l-1);
      }

      if(dec_index != -1)
        theta[comma] = theta[comma]*pow(10,dec_index-j);
        
      comma++;
      dec_index = -1;
    }
    while(comma < 6); 
  }
  
  for(int servo_num = 0; servo_num <6; servo_num++)
  {  
    theta_m[servo_num] = map(theta[servo_num],0,270,0,180);
    //Serial.print(theta[i]);
    //Serial.print(", ");
  }
  Serial.print("\n");

  servo1.write(180-theta_m[0]);              
  servo2.write(theta_m[1]);
  servo3.write(180-theta_m[2]);
  servo4.write(theta_m[3]);
  servo5.write(180-theta_m[4]);
  servo6.write(theta_m[5]);
} 






