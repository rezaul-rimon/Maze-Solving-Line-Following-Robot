#include <EEPROM.h>
#define BlackSerface  1
#define MX motor(baseSpeed,baseSpeed);
#define TEST(x) MX delay(x); sensorRead();
#define ARSpeed 180

//==================Sensor Array===================
int sen[8], onBlack;
const int senArrPin[8] = {A0, A1, A2, A3, A4, A5, A6, A7};
long int Read = 0, lastRead = 0, bigRead = 0;

int leftWheel, rightWheel;
//==================Motor Driver Pin===================
const int inA = 10, inB = 3, inC = 11, inD = 9;
int lastSpeedL, lastSpeedR;
int leftSign, rightSign;

//==================Button=============================
const int buttonPin = 13;
//const int bazzer = 11;
const int led = 2;

//=================PID Variable========================
float Ki = 0, Kd = 442, baseSpeed = 130,  MaxSpeed = 255, setPoint = 4.5, ForMax = 4.5;
float Kp = (MaxSpeed + baseSpeed) / ForMax, position = 0, error = 0, lastError = 0, P = 0, I = 0, D = 0, PID = 0, leftPID = 0, rightPID = 0;
//=================Algorithum Rule==========================
const String leftRule[7][2] = {{"LBR", "B"}, { "LBS", "R"}, { "LBL", "S"}, {"SBL", "R"}, { "SBS", "B"}, { "RBL", "B"}};
const String rightRule[7][2] = {{"RBL", "B"}, { "RBS", "L"}, { "RBR", "S"}, {"SBR", "L"}, { "SBS", "B"}, { "LBR", "B"}};
const char dir = 'L';
String path = "";


//=================General Variable===================
int nintyDegDelay = 50 , turnSpeed = 150, mazeSolve = false, scan = true, pathCount = 0;
int s[8], value[8];
int maxs[8], mins[8], ref[8];

//===================Void Setup====================================
void setup() {
  Serial.begin(9600);
  for (int i = 0; i < 8; i++)
    pinMode(senArrPin[i], INPUT);
  pinMode(buttonPin, INPUT);
  //pinMode(bazzer, OUTPUT);
  pinMode(inA, OUTPUT);
  pinMode(inB, OUTPUT);
  pinMode(inC, OUTPUT);
  pinMode(inD, OUTPUT);
  pinMode(led, OUTPUT);

  //  beep(100);
  //  delay(200);
  //  beep(100);
  //  delay(200);
  //  beep(400);

  if (dir == 'R') {
    leftSign = 1;
    rightSign = -1;
  }
  else if (dir == 'L') {
    leftSign = -1;
    rightSign = 1;
  }

  while (digitalRead(buttonPin))
  {
    delay(100);  // Fingure releasing delay
    autoCal();
  }
  for (int i = 0; i < 8; i++)
  {
    EEPROM.get(i * 2, ref[i]);
  }
  while (!digitalRead(buttonPin)) {}
  delay(100);  // Fingure releasing delay
}

//===================Void Loop======================================
void loop() {
  sensorRead();

  if (Read == 0) {
    TEST(10);
    if (Read == 0) {
      TEST(10)
      if (Read == 0) {
        white();
      }
    }
  }
  else if (blackCondition()) {
    TEST(10);
    if (blackCondition()) {
      TEST(10);
      if (blackCondition()) {
        turnLeft();
      }
    }
  }
  else if (starCondtiton()) {
    TEST(10);
    if (starCondtiton()) {
      TEST(10);
      if (starCondtiton()) {
        starAction();
      }
    }
  }
  else if (left90()) {
    TEST(10);
    if (left90()) {
      TEST(5);
      if (left90()) {
        turnLeft();
      }
    }
  }
  else if (right90()) {
    TEST(10);
    if (right90()) {
      TEST(5);
      if (right90()) {
        turnRight();
      }
    }
  }
  else onLinePID();

}

//====================Motor Driver=====================================
void motor(int left, int right) {
  int a = 0, b = 0, c = 0, d = 0;
  lastSpeedL = left;
  lastSpeedR = right;

  if (left > 0)a = left;
  else b = left * -1;
  if (right > 0)c = right;
  else d = right * -1;
  analogWrite(inA, a);
  analogWrite(inB, b);
  analogWrite(inC, c);
  analogWrite(inD, d);
}
void botBreak(int a) {
  motor(-lastSpeedL, -lastSpeedR);
  delay(a);
  motor(0, 0);
}

//void beep(int x) {
//  digitalWrite(bazzer, 1);
//  delay(x);
//  digitalWrite(bazzer, 0);
//}


/////////////////////Sensor Read///////////////////////////
void sensorRead() {
  int i, j;
  if (Read)lastRead = Read;
  if (Read > bigRead)bigRead = Read;
  onBlack = 0;
  Read = 0;
  position = 0;
  /*
    for(i=0; i<8; i++){
      Serial.print(i+1);
      Serial.print(" ");
      Serial.println(analogRead(senArrPin[i]));
    } Serial.println();Serial.println();
    return;
  */

  for (i = 0; i < 8; i++)
    sen[i] = ((ref[i]) > analogRead(senArrPin[i])) ? BlackSerface : !BlackSerface;

  for (i = 0; i < 8; i++) {
    onBlack += sen[i];
    Read = (Read * 10 + (sen[i] * (i + 1)));
    position += (sen[i] * (i + 1));
  }
  if (onBlack)
    position /= onBlack;
  else position = setPoint;
}

//====================Auto Calibration============================
void autoCal() {
  sensorReading();
  for (int i = 0; i < 8; i++) // presetting max min value
  {
    maxs[i] = mins[i] = value[i];
  }
  int start_time = millis();
  while ((millis() - start_time) < 3000) // Robot scanning and rotating
  {
    sensorReading();
    motor(100, -100);
    for (int i = 0; i < 8; i++)
    {
      if (maxs[i] < value[i])
        maxs[i] = value[i];
    }
    for (int i = 0; i < 8; i++)
    {
      if (mins[i] > value[i])
        mins[i] = value[i];
    }
  }
  motor(0, 0);  //Robot stop and end of getting data

  for (int i = 0; i < 8; i++)
  {
    EEPROM.put(i * 2, ((maxs[i] + mins[i]) / 2));
  }
}

//========================================MazeSolver===============================================================================================
void mazeSoluation() {
  bool terminator = true;
  while (terminator) {
    terminator = false;
    for (int i = 0; i < 6; i++) {
      if (dir == 'L') {
        while (path.indexOf(leftRule[i][0]) != -1) {
          path = path.substring(0, path.indexOf(leftRule[i][0])) + leftRule[i][1] + path.substring( path.indexOf(leftRule[i][0]) + 3, path.length());
          terminator = true;
        }
      }
      else {
        while (path.indexOf(rightRule[i][0]) != -1) {
          path = path.substring(0, path.indexOf(rightRule[i][0])) + rightRule[i][1] + path.substring( path.indexOf(rightRule[i][0]) + 3, path.length());
          terminator = true;
        }
      }
    }
  }
}

//===========================Condition=========================================
bool blackCondition() {
  if (Read == 12345670 || Read == 12345678 || Read == 2345678)
    return true;
  else
    return false;
}

bool left90() {
  if (Read == 12345600 || Read == 12345000 || Read == 12340000 || Read == 12300000)
    return true;
  else
    return false;
}

bool right90() {
  if (Read == 345678 || Read == 45678 || Read == 5678 || Read == 678)
    return true;
  else
    return false;
}

bool Tcondition() {
  if ((
        bigRead == 12345670 ||
        bigRead == 2345678  ||
        bigRead == 12345678 /*||
        bigRead == 345678  ||
        bigRead == 12345600*/
      ) &&
      Read == 0
     )
    return true;
  else
    return false;
}

bool plusCondition() {
  if ((
        bigRead == 12345670 ||
        bigRead == 2345678  ||
        bigRead == 12345678 /*||
        bigRead == 345678  ||
        bigRead == 12345600*/
      ) &&
      Read
     )
    return true;
  else
    return false;
}

bool starCondtiton() {
  if (
    Read == 10000008 ||
    Read == 12000008 ||
    Read == 10000078 ||
    Read == 12000078 ||
    Read == 2300670 ||
    Read == 2000670 ||
    Read == 2300070 ||

    Read == 2000070 ||
    Read == 300600 ||
    Read == 2300670 ||
    Read == 12005670 ||
    Read == 2300078
  )
    return true;
  else
    return false;
}

//===========================Action=========================================================
bool turnCondition(char c) {
  if (blackCondition()) {
    endPoint();
    return true;
  }
  else if (Tcondition() || plusCondition()) {
    if (dir == 'L')
      path += 'L';
    else if (dir == 'R')
      path += 'R';

    turn(leftSign * turnSpeed, rightSign * turnSpeed);
    return true;
  }

  else if ((lastRead == 678 || lastRead == 78 || lastRead == 8) && c == 'L') { //left 90
    if (dir == 'L')
      path += 'L';
    else
      path += 'R';////////////////////S

    turn(leftSign * turnSpeed, rightSign * turnSpeed);
    return true;
  }

  else if ((lastRead == 12300000 || lastRead == 12000000 || lastRead == 10000000) && c == 'R') { //Right 90
    if (dir == 'R')
      path += 'R';
    else
      path += 'L';/////////////////////S

    turn(leftSign * turnSpeed, rightSign * turnSpeed);
    return true;
  }

  //Read == 300000 || Read == 340000 || Read == 40000 || Read == 45000 || Read == 5000
  else if (Read) {
    if ((dir == 'R' && c == 'L') || (dir == 'L' && c == 'R')) {
      path += 'S';
      onLinePID();
    }
    else if (dir == 'L' && c == 'L') {
      path += 'L';
      turn(-turnSpeed, turnSpeed);
    }
    else if (dir == 'R' && c == 'R') {
      path += 'R';
      turn(turnSpeed, -turnSpeed);
    }
    return true;
  }

  else return false;
}



bool soluationTurn(char c) {
  if (blackCondition()) {
    bool i = 1;
    while (blackCondition()) {
      sensorRead();
      motor(0, 0);
      delay(500);
      digitalWrite(led, i);
      i = !i;
    }

    digitalWrite(led, 0);

    return true;
  }
  else  if (Tcondition() || plusCondition()) {
    if (path[pathCount] == 'S') {
      onLinePID();
    }
    else if (path[pathCount] == 'R') {
      turn(turnSpeed, -turnSpeed);
    }
    else if (path[pathCount] == 'L') {
      turn(-turnSpeed, turnSpeed);
    }
    pathCount++;
    return true;
  }


  else if ((lastRead == 678 || lastRead == 78 || lastRead == 8) && c == 'L') { //left 90
    if (c == 'L' && path[pathCount] == 'L')
      turn(-turnSpeed, turnSpeed);
    else
      turn(turnSpeed, -turnSpeed);
    pathCount++;
    return true;
  }
  else if ((lastRead == 12300000 || lastRead == 12000000 || lastRead == 10000000) && c == 'R') { //Right 90
    if (c == 'R' && path[pathCount] == 'R')
      turn(turnSpeed, -turnSpeed);
    else
      turn(-turnSpeed, turnSpeed);
    pathCount++;
    return true;
  }

  else if (Read) {
    if (c == 'L' && path[pathCount] == 'L')
      turn(-turnSpeed, turnSpeed);
    else if (c == 'R' && path[pathCount] == 'R')
      turn(turnSpeed, -turnSpeed);
    else onLinePID();
    pathCount++;
    return true;
  }

  else return false;
}

void endPoint() {
  botBreak(100);
  //beep(100);
  digitalWrite(led, 1);
  Serial.println(path);
  mazeSoluation();
  Serial.println(path);
  mazeSolve = true;
  scan = false;
  //beep(1000);
  digitalWrite(led, 0);
  bool i = 1;
  baseSpeed = ARSpeed;
  while (!digitalRead(buttonPin)) {
    delay(500);
    digitalWrite(led, i);
    i = !i;
  }
  digitalWrite(led, 0);
  //  beep(500);
}

void white() {
  if (scan == true && mazeSolve == false) {
    path += 'B';
    motor(baseSpeed, baseSpeed);
    delay(40);
    turn(leftSign * turnSpeed, rightSign * turnSpeed);
  }
  else {
    botBreak(60);
    while (Read == 0) {
      motor(0, 0);
      sensorRead();
    }
  }
}

void starAction() {
  if (scan == true && mazeSolve == false) {
    if (dir == 'R')
      path += 'R';
    else
      path += 'L';
    motor(baseSpeed, baseSpeed);
    delay(80);
    turn(leftSign * turnSpeed, rightSign * turnSpeed);
  }
  else if (scan == false && mazeSolve == true) {
    if (path[pathCount] == 'R') {
      turn(turnSpeed, -turnSpeed);
    }
    else if (path[pathCount] == 'L') {
      turn(-turnSpeed, turnSpeed);
    }
    else if (path[pathCount] == 'S' && dir == 'L') {
      turn(turnSpeed, -turnSpeed);
    }
    else if (path[pathCount] == 'S' && dir == 'R') {
      turn(-turnSpeed, turnSpeed);
    }
    else onLinePID();
    pathCount++;
  }
}

void turnLeft() {
  bigRead = 0;
  for (int i = 0; i < nintyDegDelay; i++) {
    motor(baseSpeed, baseSpeed);
    sensorRead();
  }
  if (mazeSolve == false && scan == true) {
    if (turnCondition('L') == false) {
      turn(-turnSpeed, turnSpeed);
      path += 'L';
    }
  }
  else if (mazeSolve == true && scan == false) {
    if (soluationTurn('L') == false) {
      turn(-turnSpeed, turnSpeed);
      pathCount++;
    }
  }
}

void turnRight() {
  bigRead = 0;
  for (int i = 0; i < nintyDegDelay; i++) {
    motor(baseSpeed, baseSpeed);
    sensorRead();
  }
  if (mazeSolve == false && scan == true) {
    if (turnCondition('R') == false) {
      turn(turnSpeed, -turnSpeed);
      path += 'R';
    }
  }
  else if (mazeSolve == true && scan == false) {
    if (soluationTurn('R') == false) {
      turn(turnSpeed, -turnSpeed);
      pathCount++;
    }
  }
}




void turn(int left, int right) {
  botBreak(40);
  motor(left, right);
  delay(200);
  sen[3] = 0;
  sen[4] = 0;
  while (!sen[3] || !sen[4]) {
    motor(left, right);
    sensorRead();
  }
  botBreak(40);
}

//===================PID=====================================================
void onLinePID() {
  error = position - setPoint;
  P = error;
  I += error;
  D = error - lastError;
  lastError = error;
  PID = Kp * P + Ki * I + Kd * D;

  leftPID = (PID >   (MaxSpeed - baseSpeed)) ?  (MaxSpeed - baseSpeed) : PID;
  leftPID = (PID <  -(MaxSpeed + baseSpeed)) ? -(MaxSpeed + baseSpeed) : leftPID;

  rightPID = (PID < -(MaxSpeed - baseSpeed)) ? -(MaxSpeed - baseSpeed) : PID;
  rightPID = (PID >  (MaxSpeed + baseSpeed)) ?  (MaxSpeed + baseSpeed) : rightPID;

  motor(baseSpeed + leftPID, baseSpeed - rightPID);
}



void resetPID() {

  error = 0;
  lastError = 0;
  P = 0;
  I = 0;
  D = 0;
  PID = 0;
  leftPID = 0;
  rightPID = 0;

  sensorRead();
}

void sensorReading()
{
  value[0] = analogRead(A0);
  value[1] = analogRead(A1);
  value[2] = analogRead(A2);
  value[3] = analogRead(A3);
  value[4] = analogRead(A4);
  value[5] = analogRead(A5);
  value[6] = analogRead(A6);
  value[7] = analogRead(A7);

  for (int i = 0; i < 8; i++)
  {
    if ((value[i]) < ref[i])
    {
      s[i] = 1;
    }
    else
    {
      s[i] = 0;
    }
  }
}
