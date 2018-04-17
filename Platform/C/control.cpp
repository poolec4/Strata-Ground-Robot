/* Global */
// Define Controller Weights
int a = 800;
int b = 800;
int c = 0;
int d = 50;
int e = 50;
int f = 20;
int g = 0.0008;
// Create Weight Matricies
int A[6] = {1, 1, 1, -1, -1, 1};
int B[6] = {1, -1, -1, 1, -1, 1};
int C[6] = {-1, -1, -1, -1, -1, -1};
int D[6] = {1, 1, 0, 0, 0, 0};
int E[6] = {0, 0, 1, 1, -1, -1};
int F[6] = {-1, 1, -1, 1, -1, 1};
float G = g;
// Error Array
float error[6] = {0, 0, 0, 0, 0 ,0};
float error_old[6];
// Initial Angles
float theta[6] = {135, -135, 135, -135, 135, -135};
float dTheta[6] = {0, 0, 0, 0, 0, 0};
// Time
unsigned long t = millis();
// Platform Data
angles[3] = {0, 0, 0};
translation[3] = {0, 0, 0};

/* Setup */
void setup(){
  for (i=0; i<6; i++){
    A[i] = a*A[i];
    B[i] = b*B[i];
    C[i] = c*C[i];
    D[i] = d*D[i];
    E[i] = e*E[i];
    F[i] = f*F[i];
  }
}

/* Run */
void loop(){
  // Get Data from IMU
  // Step Controller
  PI_control();
}

void PI_control(){
  float dt = millis()-t;
  for (i=0; i<6; i++){
    // Proportional Control
    dTheta[i] = A[i]*sind(angles[0])+B[i]*sind(angles[1])+C[i]*sind(angles[2])+D[i]*translation[0]+E[i]*translation[1]+F[i]*translation[2];
    
  }
}

void bound(){

}

float sind(float theta){
  return sin(theta*PI/180.0)
}
