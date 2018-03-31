#define L1_F 1
#define L1_B 2
#define R1_F 3
#define R1_B 4

#define L2_F 5
#define L2_B 6
#define R2_F 7
#define R2_B 8

#define L3_F 9
#define L3_B 10
#define R3_F 11
#define R3_B 12

double motor_vals[6] = {0,0,0,0,0,0}; 

void setup() {
  pinMode(L1_F, OUTPUT);
  pinMode(L1_B, OUTPUT);
  pinMode(R1_F, OUTPUT);
  pinMode(R1_B, OUTPUT);
  pinMode(L2_F, OUTPUT);
  pinMode(L2_B, OUTPUT);
  pinMode(R2_F, OUTPUT);
  pinMode(R2_B, OUTPUT);
  pinMode(L3_F, OUTPUT);
  pinMode(L3_B, OUTPUT);
  pinMode(R3_F, OUTPUT);
  pinMode(R3_B, OUTPUT);

  digitalWrite(L1_F, LOW);
  digitalWrite(L1_B, LOW);
  digitalWrite(R1_F, LOW);
  digitalWrite(R1_B, LOW);

  digitalWrite(L2_F, LOW);
  digitalWrite(L2_B, LOW);
  digitalWrite(R2_F, LOW);
  digitalWrite(R2_B, LOW);

  digitalWrite(L3_F, LOW);
  digitalWrite(L3_B, LOW);
  digitalWrite(R3_F, LOW);
  digitalWrite(R3_B, LOW);

  Serial.begin(9600);
}

void loop() {
  Serial.flush();
  
  if(Serial.available() > 0)
  { 
    buffer = Serial.readStringUntil('\n');    
    buffer.toCharArray(buffer_array,buffer.length()+1);
    //Serial.println(buffer_array);
    
    motor_vals[0] = parse_string_to_double(buffer_array, "A");
    motor_vals[1] = parse_string_to_double(buffer_array, "B");
    motor_vals[2] = parse_string_to_double(buffer_array, "C");
    motor_vals[3] = parse_string_to_double(buffer_array, "D");
    motor_vals[4] = parse_string_to_double(buffer_array, "E");
    motor_vals[5] = parse_string_to_double(buffer_array, "F");
  
    if(motor_vals[0]<255){
      analogWrite(L1_R, 255-motor_vals[0]);
    }
    if(motor_vals[1]<255){
      analogWrite(L2_R, 255-motor_vals[1]);
    }
    if(motor_vals[2]<255){
      analogWrite(L3_R, 255-motor_vals[2]);
    }
    if(motor_vals[3]<255){
      analogWrite(R1_R, 255-motor_vals[3]);
    }
    if(motor_vals[4]<255){
      analogWrite(R2_R, 255-motor_vals[4]);
    }
    if(motor_vals[5]<255){
      analogWrite(R3_R, 255-motor_vals[5]);
    }

    if(motor_vals[0]>255){
      analogWrite(L1_F, motor_vals[0]-255);
    }
    if(motor_vals[1]>255){
      analogWrite(L2_F, motor_vals[1]-255);
    }
    if(motor_vals[2]>255){
      analogWrite(L3_F, motor_vals[2]-255);
    }
    if(motor_vals[3]>255){
      analogWrite(R1_F, motor_vals[3]-255);
    }
    if(motor_vals[4]>255){
      analogWrite(R2_F, motor_vals[4]-255);
    }
    if(motor_vals[5]>255){
      analogWrite(R3_F, motor_vals[5]-255);
    }
  
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
