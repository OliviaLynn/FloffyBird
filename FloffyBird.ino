#include <Pololu3pi.h>
#include <PololuQTRSensors.h>
#include <OrangutanMotors.h>
#include <OrangutanAnalog.h>
#include <OrangutanLCD.h>
#include <OrangutanPushbuttons.h>
#include <OrangutanBuzzer.h>

Pololu3pi robot;
OrangutanLCD lcd;
OrangutanPushbuttons buttons;
OrangutanAnalog analog;
OrangutanBuzzer buzzer;
OrangutanMotors motors;

int spd;
const int normalSpeed = 20;
unsigned int sensors [5];

bool isStable = true;
bool isDazed = false;
bool isMoving = true;

int dazeLeft = 0;
int strokes = 0;
int strokeCountdown = 0;
int tiltedCountdown = 20;
int tiltedCountdownMax = 20;

const int squarePin = 6;
const int yPin = 7;

int yMin;
int yMax;
int yStart;

const int squareMin = 200;
const int squareMax = 500;

void setup() {
  
  robot.init(2000); // For the IR sensors
  
  calibrateGyro();
  
  while (!OrangutanPushbuttons::isPressed(BUTTON_B))
  {
    delay(10);
  }

  OrangutanPushbuttons::waitForRelease(BUTTON_B);
  delay(200);

  unsigned int lineCalibrationCounter;
  for (lineCalibrationCounter=0; lineCalibrationCounter<80; lineCalibrationCounter++)
  {
    if (lineCalibrationCounter < 20 || lineCalibrationCounter >= 60)
      OrangutanMotors::setSpeeds(40, -40);
    else
      OrangutanMotors::setSpeeds(-40, 40);

    robot.calibrateLineSensors(IR_EMITTERS_ON);
    // Since our counter runs to 80, the total delay will be 80*20 = 1600 ms.
    delay(20);
  }
  OrangutanMotors::setSpeeds(0, 0);
  isMoving = true;
}

void calibrateGyro() {
yMin = 486;
yMax = 490;
  
  yStart = analogRead(yPin);
  
  yMin = yStart;
  yMax = yStart;

  lcd.clear();
  unsigned int sensorCalibration = 40;
  while(sensorCalibration > 0) {
    int y = analogRead(yPin);
    if (y < yMin)
      yMin = y;
    if (y > yMax)
      yMax = y;
    sensorCalibration--;
    lcd.clear();
    lcd.print("Time: ");
    lcd.print(sensorCalibration);
    lcd.gotoXY(0, 1);
    lcd.print("Y: ");
    lcd.print(y);
    delay(100);
  }
  lcd.clear();
  lcd.print("Lo: ");
  lcd.print(yMin);
  lcd.gotoXY(0, 1);
  lcd.print("Hi: ");
  lcd.print(yMax);
  
}

void loop() {
  
  int position = robot.readLine(sensors, IR_EMITTERS_ON);
  int terr = getTerrain();
  int roadPosition = getRoadPosition();
  
  int squarePressure = analogRead(squarePin);
  int yRead = analogRead(yPin);

  checkIsStable(yRead);
  detectSquarePressure(squarePressure);
  displayInfo(yRead, squarePressure, terr, roadPosition);
  /*
  if (millis() % 100 == 0) {
  lcd.clear();
  lcd.print(squarePressure);
  }
  */

  if (isDazed) 
    beDazed();
    /*
  if (isMoving && !isDazed) 
    go(roadPosition);
  else 
    motors.setSpeeds(0, 0);
*/
  if (isMoving) {
    go(roadPosition, terr);
  }  
  else {
    motors.setSpeeds(0, 0);    
  }
}


int getRoadPosition() {
  //returns average index of road
  int count = 0;
  int sum = 0;
  for (int i = 0; i < 5; i++) {
    if (sensorTerrain(sensors[i]) == 4) {
      count++;
      sum += (i + 1);
    }
  }
  return (sum/count);
}

int maxFast = 30;
int minFast = 0;

int shiver() {
  return maxFast * sin(millis()/10);
}

int drag() {
  int step_length = 500;
  return (maxFast/2) * cos((millis()*PI)/step_length);

}

void setMotorsLeft(int terr) {
  motors.setSpeeds(minFast, maxFast);
}

void setMotorsStraight(int terr) {
  if (terr == 1)
    motors.setSpeeds(maxFast + shiver(), maxFast - shiver());
  else if (terr == 3)
    motors.setSpeeds(maxFast/2 + drag(), maxFast/2 - drag());
  else
    motors.setSpeeds(maxFast, maxFast);
}

void setMotorsRight(int terr) {
  motors.setSpeeds(maxFast, minFast);
}

void go(int roadPosition, int terr) {
  if (roadPosition == 1)
    setMotorsLeft(terr);
  else if (2 <= roadPosition && roadPosition <= 4)
    setMotorsStraight(terr);
  else if (roadPosition == 5)
    setMotorsRight(terr);
  else
    setMotorsStraight(terr);
}

int sensorTerrain(int x) {
  if ( x < 300) { //snow
    return 1;
  }
  else if (300 <= x && x < 850) { //grass
    return 2;
  }
  else if (850 <= x && x < 875) { //mud
    return 3;
  }
  else if (875 <= x) { //road
    return 4;
  }
}

int getTerrain() {
  int counts [5] = {0,0,0,0,0};
  for (int i = 0; i < 5; i++) {
    int terrainType = sensorTerrain(sensors[i]);
    counts[terrainType] = counts[terrainType] + 1;
  }
  int index = 0;
  int count = 0;
  for (int j = 1; j < 4; j++) { //not 5, because we don't want to count roads
    if (counts[j] >= count) { //using <= so grass can override snow if they're tied
      index = j;
      count = counts[j];
    }
  }
  return index;
}


void detectSquarePressure(int squarePressure) {
  if (squareMin < squarePressure){ // && squarePressure < squareMax) {
    doStroke();
  }
  /*if (squarePressure > squareMax) {
    setDaze();
  }
  */
}

void doStroke() {
  buzzer.playNote( NOTE_G(3), 50, 10);
  isMoving = true;
  tiltedCountdown = tiltedCountdownMax;
}

void setDaze() {
  /*
  int seconds = 1;
  isDazed = true;
  dazeLeft = 10;
  */
}

void beDazed() {
  if (dazeLeft == 0)
    isDazed = false;
  else {
    dazeLeft = dazeLeft - 1;
    cry();
  }
}

void cry() {
  const int len = 50;
  const int vol = 15;
  buzzer.playNote( NOTE_E(6), len, vol);
}

void printTerrain(int terr) {
  if (terr == 1)
    lcd.print("SNOW");
  if (terr == 2)
    lcd.print("GRASS");
  if (terr == 3)
    lcd.print("MUD");
}

void displayInfo (int y, int squarePressure, int terr, int roadPosition) {
  lcd.clear();
  lcd.gotoXY(0,0);
  printStabilityText(y);
  lcd.gotoXY(0,1);
  printTerrain(terr);
  lcd.print(" ");
  lcd.print(roadPosition);
  /*
  if (isDazed) 
    lcd.print(dazeLeft);
  else 
    lcd.print(squarePressure);
    */
}

String printStabilityText(int y) {
  if (isDazed) {
    lcd.print("DAZED!!");
  }
  else if (isStable) {
    lcd.print("Stable");
  }
  else {
    if (y <= yMin)
      lcd.print("Down");
    if (y >= yMax)
      lcd.print("Up");
  }
}

void checkIsStable(int y) {
  if (isStableY(y)) {
    if (!isStable)
      buzzer.playNote( NOTE_G(5), 50, 0);
    isStable = true;
    tiltedCountdown = tiltedCountdownMax;
  }
  else {
    if (isMoving) {
      if (isStable && tiltedCountdown == tiltedCountdownMax) {
        buzzer.playNote( NOTE_G(4), 50, 0);
      }
      else if (tiltedCountdown > 0) {
        tiltedCountdown--;
      }
      else if (tiltedCountdown <= 0) {
        isMoving = false;
        buzzer.playNote( NOTE_D(4), 50, 10);
      }
    }
    isStable = false;
  }
}

bool isStableY(int y) {
  return yMin < y && y < yMax;
}

