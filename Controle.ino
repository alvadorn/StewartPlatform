#include <PID_v1.h>
#include <Servo.h>

#define MID_X 250
#define MID_Y 250

#define S0 10 // left
#define S1 9 // bot
#define S2 6 // right
#define S3 5 // top

#define X1 A0
#define X2 A1
#define Y1 A2
#define Y2 A3

#define Xres 600
#define Yres 240

double setpointX[2] = { 350 };
double setpointY[2] = { 250 };

double oldPos[4] = {100, 100, 100, 100 };
double posServo[4] = { 100, 100, 100, 100 };

Servo servos[4];

int posX, posY, lastPosX = setpointX, lastPosY = setpointY;

double Kp[4] = { 50, 50, 50, 50 };
double Kd[4] = { 30, 30, 30, 30 };
double Ki[4] = { 60, 60, 60, 60 };

double input[4] = { 0 };

PID s1(&input[0], &posServo[0], &setpointX[0], Kp[0],Ki[0],Kd[0], DIRECT);
PID s2(&input[1], &posServo[1], &setpointY[0], Kp[1],Ki[1],Kd[1], DIRECT);
PID s3(&input[2], &posServo[2], &setpointX[1], Kp[2],Ki[2],Kd[2], DIRECT);
PID s4(&input[3], &posServo[3], &setpointY[1], Kp[3],Ki[3],Kd[3], DIRECT);

int readX() {
  pinMode(X1, OUTPUT);
  digitalWrite(X1, HIGH);
  pinMode(Y1, OUTPUT);
  digitalWrite(Y1, LOW);
  // Tri-state
  pinMode(Y2, INPUT);
  digitalWrite(Y2, LOW);
  // Read
  pinMode(X2, INPUT);
  return map(analogRead(X2), 0, 1023, 0, Xres);
}

int readY() {
   pinMode(X2, OUTPUT);
   pinMode(Y2, OUTPUT);
   pinMode(X1, INPUT);
   pinMode(Y1, INPUT);
   digitalWrite(X2,LOW);
   digitalWrite(Y2, HIGH);
   digitalWrite(Y1, LOW);
   return map(analogRead(X1),0,1023, 0, Yres);
}

void fixServoPos(Servo &servo, int pos) {
  for (int i = 180; i > pos; i-= 10) {
    servo.write(i);
  }
}

void sweep(Servo &servo, double &currentPos, double nextPos, int agg=10) {
  if (currentPos > nextPos) {
    for (int i = currentPos; i > nextPos; i -= agg) {
      servo.write(i);
    }
  } else {
    for (int i = currentPos; i < nextPos; i += agg) {
      servo.write(i);
    }
  }
  currentPos = nextPos;
}

void setup() {
  servos[0].attach(S0);
  servos[1].attach(S1);
  servos[2].attach(S2);
  servos[3].attach(S3);

  for (int i = 0; i < 4; i++) {
    fixServoPos(servos[i], posServo[i]);
  }

  s1.SetMode(AUTOMATIC);
  s2.SetMode(AUTOMATIC);
  s3.SetMode(AUTOMATIC);
  s4.SetMode(AUTOMATIC);

}

void loop() {
  posX = readX();
  posY = readY();

  input[0] = posX;
  input[1] = posY;
  input[2] = MID_X - posX;
  input[3] = MID_Y - posY;


  s1.Compute();
  s2.Compute();
  s3.Compute();
  s4.Compute();


  for (int i = 0; i < 4; i++) {
    sweep(servos[i], oldPos[i],posServo[i]);
  }
  
}

/*
 * 0: left servo
 * 1: bot servo
 * 2: right servo
 * 3: top servo
 */
