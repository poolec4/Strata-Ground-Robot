/*
  Written by Chris Poole - Spring 2018
 
 
 Sets servo angles from serial input in the form:
 "th1,th2,th3,th4,th5,th6\n"
 where the angle is in degrees.
 
 thi = 0 corresponds to the ith servo in the down position
 
 for servos 1, 3, 5 positive rotation is in the CW direction
 for servos 2, 4, 6 positive rotation is in the CCW direction
 
 In other words, the parser breaks if it gets negative values....
 */

#include <Servo.h> 
#include <string.h>

Servo servo1, servo2, servo3, servo4, servo5, servo6;  // create servo object to control a servo 
double theta[6] = {135,135,135,135,135,135};
double theta_m[6];
double t;

String buffer;
char buffer_array[150];

void setup() 
{   
  Serial.begin(19200);
  Serial.setTimeout(5);

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
    buffer = Serial.readStringUntil('\n');    
    buffer.toCharArray(buffer_array,buffer.length()+1);
    //Serial.println(buffer_array);
    
    theta[0] = parse_string_to_double(buffer_array, "A");
    theta[1] = parse_string_to_double(buffer_array, "B");
    theta[2] = parse_string_to_double(buffer_array, "C");
    theta[3] = parse_string_to_double(buffer_array, "D");
    theta[4] = parse_string_to_double(buffer_array, "E");
    theta[5] = parse_string_to_double(buffer_array, "F");
  }
  
  for(int servo_num = 0; servo_num <6; servo_num++)
  { 
    //Serial.print(theta[servo_num]);
    //Serial.print(",");
    theta_m[servo_num] = map(theta[servo_num],0,270,0,180);
  }
  //Serial.print("\n");
  
  if(abs(180-theta_m[0])<60 || abs(theta_m[1])<60 || abs(180-theta_m[2])<60 || abs(theta_m[3])<60 || abs(180-theta_m[4])<60 || abs(theta_m[5])<60)
  {
  }
  else
  {
    servo1.write(180-theta_m[0]);              
    servo2.write(theta_m[1]);
    servo3.write(180-theta_m[2]);
    servo4.write(theta_m[3]);
    servo5.write(180-theta_m[4]);
    servo6.write(theta_m[5]);
  }
} 

double parse_string_to_double(char *string, char const *tag)
{
  char* string_copy = (char*)malloc(strlen(string) + 1);
  char* char_result;
  char* token;
  double result = 0.0;

  strcpy(string_copy, string);

  token = strtok(string_copy, "&");

  while(token)
  {
    char* equals_sign = strchr(token, '=');

    if(equals_sign)
    {
      *equals_sign = 0;

      if( 0 == strcmp(token, tag))
      {
        equals_sign++;
        char_result = (char*)malloc(strlen(equals_sign) + 1);
        strcpy(char_result, equals_sign);
        result = atof(char_result);
        free(char_result);
      }
    }
    token = strtok(0, "&");
  }
  free(string_copy);

  return result;
}






