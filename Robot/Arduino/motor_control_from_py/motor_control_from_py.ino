#define L1_F 2
#define L1_B 3
#define R1_F 4
#define R1_B 5

#define L2_F 6
#define L2_B 7
#define R2_F 8
#define R2_B 9

#define L3_F 10
#define L3_B 11
#define R3_F 12
#define R3_B 13

double motor_vals[6] = {0, 0, 0, 0, 0, 0};

String buffer;
char buffer_array[150];
unsigned long t_old = 0;

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

  Serial.begin(19200);
  Serial.setTimeout(5);
}

void loop() {
  Serial.flush();

  if (Serial.available() > 0)
  {
    t_old = millis();
    buffer = Serial.readStringUntil('\n');
    buffer.toCharArray(buffer_array, buffer.length() + 1);
    //Serial.println(buffer_array);

    motor_vals[0] = parse_string_to_double(buffer_array, "A");
    motor_vals[1] = parse_string_to_double(buffer_array, "B");
    motor_vals[2] = parse_string_to_double(buffer_array, "C");
    motor_vals[3] = parse_string_to_double(buffer_array, "D");
    motor_vals[4] = parse_string_to_double(buffer_array, "E");
    motor_vals[5] = parse_string_to_double(buffer_array, "F");

    for (int i = 0; i < 6; i++) {
      Serial.print(motor_vals[i]);
      Serial.print(",");
    }
    Serial.print("\n");

    if (motor_vals[0] < 255) {
      analogWrite(L1_B, 255 - motor_vals[0]);
      digitalWrite(L1_F, LOW);
    }
    if (motor_vals[1] < 255) {
      analogWrite(L2_B, 255 - motor_vals[1]);
      digitalWrite(L2_F, LOW);
    }
    if (motor_vals[2] < 255) {
      analogWrite(L3_B, 255 - motor_vals[2]);
      digitalWrite(L3_F, LOW);
    }
    if (motor_vals[3] < 255) {
      analogWrite(R1_B, 255 - motor_vals[3]);
      digitalWrite(R1_F, LOW);
    }
    if (motor_vals[4] < 255) {
      analogWrite(R2_B, 255 - motor_vals[4]);
      digitalWrite(R2_F, LOW);
    }
    if (motor_vals[5] < 255) {
      analogWrite(R3_B, 255 - motor_vals[5]);
      digitalWrite(R3_F, LOW);
    }

    if (motor_vals[0] >= 255) {
      analogWrite(L1_F, motor_vals[0] - 255);
      digitalWrite(L1_B, LOW);
    }
    if (motor_vals[1] >= 255) {
      analogWrite(L2_F, motor_vals[1] - 255);
      digitalWrite(L2_B, LOW);
    }
    if (motor_vals[2] >= 255) {
      analogWrite(L3_F, motor_vals[2] - 255);
      digitalWrite(L3_B, LOW);
    }
    if (motor_vals[3] >= 255) {
      analogWrite(R1_F, motor_vals[3] - 255);
      digitalWrite(R1_B, LOW);
    }
    if (motor_vals[4] >= 255) {
      analogWrite(R2_F, motor_vals[4] - 255);
      digitalWrite(R2_B, LOW);
    }
    if (motor_vals[5] >= 255) {
      analogWrite(R3_F, motor_vals[5] - 255);
      digitalWrite(R3_B, LOW);
    }

  }

  if ((millis() - t_old) > 500) {
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

  while (token)
  {
    char* equals_sign = strchr(token, '=');

    if (equals_sign)
    {
      *equals_sign = 0;

      if ( 0 == strcmp(token, tag))
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
