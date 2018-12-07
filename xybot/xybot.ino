#include <MeOrion.h>
#include <EEPROM.h>
#include <SoftwareSerial.h>
#include <Wire.h>

// data stored in eeprom
/* For Arduino Uno the following applies:
 * int:                 16b (2 B)
 * char:                 8b (1 B)
 * unsigned char:        8b (1 B) (equivalent to byte)
 */
static union{
    struct{
      char name[8];               //  8 B
      unsigned char motoADir;     //  1 B
      unsigned char motoBDir;     //  1 B
      unsigned char motorSwitch;  //  1 B
      int height;                 //  2 B
      int width;                  //  2 B
      int speed;                  //  2 B
      int penUpPos;               //  2 B
      int penDownPos;             //  2 B
    }data;                        // This means a total of 8 + 3*1 + 5*2 = 21B used of the 64 reserved
    char buf[64];                 // 64 B
}roboSetup;

// arduino only handle A,B step mapping
float curSpd,tarSpd; // speed profile
float curX,curY,curZ;             // current xyz position [xy-coordinates]
float tarX,tarY,tarZ;             // target xyz position [xy-coordinates]
// step values
int tarA,tarB;                    // target stepper position [steps]
int posA,posB;                    // current stepper position [steps]
int8_t motorAfw,motorAbk;         // [Moto{A,B}Dir == 0] => Backward-Move => [motor{A,B}fw = 1, motor{A,B}bk = -1]
int8_t motorBfw,motorBbk;         // [Moto{A,B}Dir == 1] => Forward-Move  => [motor{A,B}fw = -1, motor{A,B}bk = 1]

MePort stpA(PORT_1);
MePort stpB(PORT_2);
MePort ylimit(PORT_3);
int ylimit_pin1 = ylimit.pin1();  //limit 2
int ylimit_pin2 = ylimit.pin2();  //limit 1

MePort xlimit(PORT_6);
int xlimit_pin1 = xlimit.pin1();  //limit 4
int xlimit_pin2 = xlimit.pin2();  //limit 3
long last_time;
long last_time2;
MeDCMotor laser(M2);
MePort servoPort(PORT_7);
int servopin =  servoPort.pin2();
Servo servoPen;

// Laser changes
boolean laserOn = false;          // Added by Bartosz (9.1.17). Represents laser state to prevent plotter from speeding up when laser is on
long laserDelay;                  // Delay of head when laser is on (constant speed required in opposite to pen drawing)
long mStoredDelay;                // Storage for mDelay from doMove when laser is on

/************** motor movements ******************/
void stepperMoveA(int dir)
{
  if(dir>0){
    stpA.dWrite1(LOW);
  }else{
    stpA.dWrite1(HIGH);
  }
  stpA.dWrite2(HIGH);
  stpA.dWrite2(LOW);
}

void stepperMoveB(int dir)
{
  if(dir>0){
    stpB.dWrite1(LOW);
  }else{
    stpB.dWrite1(HIGH);
  }
  stpB.dWrite2(HIGH);
  stpB.dWrite2(LOW);
}


/************** calculate movements ******************/
#define STEPDELAY_MIN 200     // micro second
#define STEPDELAY_MAX 1000
#define STEPDELAY_HEALTHY_MIN 300   // Added by Bartosz. 05.10.17 Adjusted min and max delay so it's healthier for motors
#define STEPDELAY_HEALTHY_MAX 3000  // HEALTHY_MIN is aprox the mDelay that doMove() uses at Speed 90. HEALTHY_MAX is the one 
                                    // it uses at Speed 40. Everything outside these borders is far too extreme. (too slow or too fast)
long stepAuxDelay=0;            // [micro seconds]
int stepdelay_min=200;          // [micro seconds]  is set in RoboSetup. Speed 60 --> stepdelay_min (100-60)*10
int stepdelay_max=1000;         // [micro seconds]  is set in RoboSetup. Speed 60 --> stepdelay_max (100-60)*100
// absolute min_delay = (100-99)*10
#define ACCELERATION 2          // mm/s^2 don't get inertia exceed motor could handle
#define SEGMENT_DISTANCE 10     // 1 mm for each segment
#define SPEED_STEP 1

// Limit pin checking expansion
    
/** Extesion by Bartosz, 9.1.17 Limit-pin checking before doing movements */
boolean limit_pins_ok() {
  // pin == 0 means that head touches that pin
if (limit_pins_1_ok() and limit_pins_2_ok())
  return true;
else 
  return false;
}
  
boolean limit_pins_1_ok() {
int x1 = digitalRead(xlimit_pin1);
int y1 = digitalRead(ylimit_pin1);
if (x1 and y1)
  return true;
else 
  return false;
}

boolean limit_pins_2_ok() {
int x2 = digitalRead(xlimit_pin2);
int y2 = digitalRead(ylimit_pin2);
if (x2 and y2)
  return true;
else 
  return false;
}

boolean limit_pin_x1_ok() {
int x1 = digitalRead(xlimit_pin1);
if (x1) return true;
else return false;
}

boolean limit_pin_x2_ok() {
int x2 = digitalRead(xlimit_pin2);
if (x2) return true;
else return false;
}

boolean limit_pin_y1_ok() {
int y1 = digitalRead(ylimit_pin1);
if (y1) return true;
else return false;
}

boolean limit_pin_y2_ok() {
int y2 = digitalRead(ylimit_pin2);
if (y2) return true;
else return false;
}


void doMove()
{
  long mDelay=stepdelay_max;          // [micro seconds] delay that doesn't get resetted every run of the loop. basically the current speed = stepSize/mDelay
  long temp_delay;                    // [micro seconds] temporary delay that takes stepAuxDelay into account (theoretically at least, see Bug suspicion found on 9.1.17)
  int speedDiff = -SPEED_STEP;        // responsible for accelerating and slowing down, speedDiff=1 means the mDelay gets bigger, so the moves get slower
  int dA,dB,maxD;                     // deltaA, deltaB, maxDelta (to target) [steps]
  float stepA,stepB;                  // motor step size, range:0-1 [steps] (so that motorA and B finish move at same time)
  float cntA=0,cntB=0;                // stepcounter range:pos{A,B}-tar{A,B} in steps of step{A,B} [steps]
  int d;                              // direction to move a motor to. range={-1,1}
  dA = tarA - posA;
  dB = tarB - posB;
  maxD = max(abs(dA),abs(dB));
  stepA = (float)abs(dA)/(float)maxD;
  stepB = (float)abs(dB)/(float)maxD;

  bool giveFeed = true;               // update feed extension, giving information about move status while moving
  int feedInterval = 300;

  for(int i=0;(posA!=tarA)||(posB!=tarB);i++){                         // Robbo1 2015/6/8 Changed - change loop terminate test to test for moving not finished rather than a preset amount of moves
    // move A
    if(posA!=tarA){
      cntA+=stepA;
      if(cntA>=1){       
        if (limit_pins_ok()) {  // If everything ok, normal move
          d = dA>0?motorAfw:motorAbk;
          posA+=(dA>0?1:-1);
          stepperMoveA(d);
          cntA-=1;
        } else {
          if ((limit_pin_x1_ok() && dA > 0) || (limit_pin_x2_ok() && dA < 0)) { // At least moves in 2 directions are allowed
            d = dA>0?motorAfw:motorAbk;
            posA+=(dA>0?1:-1);
            stepperMoveA(d);
            cntA-=1;
          } else {
          Serial.print("Move not allowed. PINS X1 X2 Y1 Y2: "); // Don't change the first 17 chars of the message. It gets preg-matched by the Control-Unit
          Serial.print(limit_pin_x1_ok());
          Serial.print(limit_pin_x2_ok());
          Serial.print(limit_pin_y1_ok());
          Serial.println(limit_pin_y2_ok());
          break;
          }
        }
      }
    }
    // move B
    if(posB!=tarB){
      cntB+=stepB;
      if(cntB>=1){
        if (limit_pins_ok()) { // If everythin ok, normal move
          d = dB>0?motorBfw:motorBbk;
          posB+=(dB>0?1:-1);
          stepperMoveB(d);
          cntB-=1;
        } else {
          if ((limit_pin_y1_ok() && dB > 0) || (limit_pin_y2_ok() && dB < 0)) { // At least moves in 2 directions are allowed
            d = dB>0?motorBfw:motorBbk;
            posB+=(dB>0?1:-1);
            stepperMoveB(d);
            cntB-=1;
          } else {
          Serial.print("Move not allowed. PINS X1 X2 Y1 Y2: ");
          Serial.print(limit_pin_x1_ok());
          Serial.print(limit_pin_x2_ok());
          Serial.print(limit_pin_y1_ok());
          Serial.println(limit_pin_y2_ok());
          break;
          }
        }
      }
    }
    if (laserOn) {    // Prevent acceleration caused delay changes when laser is on (Bartosz, 9.1.17)
      // BEWARE of limits. Speed is not to exceed [stepdelay_min, stepdelay_max] using the standard makeblock XY-Plotter servo motors  
      mDelay = stepdelay_min + (stepdelay_max - stepdelay_min)/2;                                     // Change this line if head-speed is not satisfactory 
      mDelay = constrain(mDelay, stepdelay_min, stepdelay_max);
    } else {
      //mDelay=constrain(mDelay+speedDiff,stepdelay_min,stepdelay_max); // original line for speed calculation
      mDelay = stepdelay_min + ((stepdelay_max - stepdelay_min)/5)*1;                                     // Change this line if head-speed is not satisfactory 
      mDelay = constrain(mDelay, stepdelay_min, stepdelay_max);
    }
    temp_delay = mDelay + stepAuxDelay;
    if(millis() - last_time > 400)
    {
      last_time = millis();
      if(true == process_serial())                              // Input of new Commands is (almost) always possible
      {
        return;  
      }
    }

    // Output of Robot-dependent data (Bartosz 22.08.17)
    // Format of Data: [UpdFeed posA posB tarA tarB mDelay temp_delay stepdelay_max stepdelay_min PIN_X1 PIN_X2 PIN_Y1 PIN_Y2 laserOn UpdFeed] 'UpdFeed' is a magic number
    // Explanations:
    // posA + posB:                   absolute position of Head
    // tarA + tarB:                   currently moving to Position XXX XXX
    //                                idea: Use abs(tarA-posA) for interpolation of actual robot head while moving
    // mDelay, temp_delay:            delay between steps [micro seconds]. speed = stepSize/mDelay. temp_delay is mDelay + auxDelay
    // stepdelay_max, stepdelay_min:  minimal and maximal delay of servo motors [micro seconds]
    
    if (giveFeed && abs(millis() - last_time2) > feedInterval) {
      last_time2 = millis();
      Serial.print("UpdFeed ");
      Serial.print(posA); Serial.print(" ");
      Serial.print(posB); Serial.print(" ");
      Serial.print(tarA); Serial.print(" ");
      Serial.print(tarB); Serial.print(" ");
      Serial.print(mDelay); Serial.print(" ");
      Serial.print(temp_delay); Serial.print(" ");
      Serial.print(stepdelay_max); Serial.print(" ");
      Serial.print(stepdelay_min); Serial.print(" ");
      Serial.print(limit_pin_x1_ok()); Serial.print(" ");
      Serial.print(limit_pin_x2_ok()); Serial.print(" ");
      Serial.print(limit_pin_y1_ok()); Serial.print(" ");
      Serial.print(limit_pin_y2_ok()); Serial.print(" ");
      Serial.print(laserOn); Serial.print(" ");
      Serial.println("UpdFeed");
    }

    if(temp_delay > stepdelay_max)
    {
      temp_delay = stepAuxDelay;                              // Bug: only makes sense if stepAuxDelay is at least bigger than 1000. delete this line? (Bartosz, 9.1.17)
      delay(temp_delay/1000);
      delayMicroseconds(temp_delay%1000);
    }
    else
    {
      delayMicroseconds(temp_delay);
    }
    if((maxD-i)<((stepdelay_max-stepdelay_min)/SPEED_STEP)){
      speedDiff=SPEED_STEP;
    }
  }
  posA = tarA;
  posB = tarB;

  if (giveFeed) {
    last_time2 = millis();
    Serial.print("UpdFeed ");
    Serial.print(posA); Serial.print(" ");
    Serial.print(posB); Serial.print(" ");
    Serial.print(tarA); Serial.print(" ");
    Serial.print(tarB); Serial.print(" ");
    Serial.print(mDelay); Serial.print(" ");
    Serial.print(temp_delay); Serial.print(" ");
    Serial.print(stepdelay_max); Serial.print(" ");
    Serial.print(stepdelay_min); Serial.print(" ");
    Serial.print(limit_pin_x1_ok()); Serial.print(" ");
    Serial.print(limit_pin_x2_ok()); Serial.print(" ");
    Serial.print(limit_pin_y1_ok()); Serial.print(" ");
    Serial.print(limit_pin_y2_ok()); Serial.print(" ");
    Serial.print(laserOn); Serial.print(" ");
    Serial.println("UpdFeed");
  }
}

/******** mapping xy position to steps ******/
#define STEPS_PER_CIRCLE 3200.0f
#define WIDTH 380
#define HEIGHT 310
#define DIAMETER 11 // the diameter of stepper wheel
//#define STEPS_PER_MM (STEPS_PER_CIRCLE/PI/DIAMETER) 
#define STEPS_PER_MM 87.58 // the same as 3d printer
void prepareMove()
{
  float dx = tarX - curX;
  float dy = tarY - curY;
  float distance = sqrt(dx*dx+dy*dy);
  Serial.print("distance=");Serial.println(distance);
  if (distance < 0.001)
    return;
  tarA = tarX*STEPS_PER_MM;
  tarB = tarY*STEPS_PER_MM;
  doMove();
  curX = tarX;
  curY = tarY;
}

void goHome()
{
  long mDelay = STEPDELAY_HEALTHY_MIN;// + (stepdelay_max - stepdelay_min)/4;                // Change this line if head-speed is not satisfactory 
  //mDelay = constrain(mDelay, stepdelay_min, stepdelay_max);

  Serial.print("UpdFeed ");
  Serial.print("---"); Serial.print(" ");
  Serial.print("---"); Serial.print(" ");
  Serial.print("---"); Serial.print(" ");
  Serial.print("---"); Serial.print(" ");
  Serial.print("---"); Serial.print(" ");
  Serial.print("---"); Serial.print(" ");
  Serial.print("---"); Serial.print(" ");
  Serial.print("---"); Serial.print(" ");
  Serial.print("-"); Serial.print(" ");
  Serial.print("-"); Serial.print(" ");
  Serial.print("-"); Serial.print(" ");
  Serial.print("-"); Serial.print(" ");
  Serial.print("---"); Serial.print(" ");
  Serial.println("UpdFeed");
  
  while(digitalRead(ylimit_pin2)==1 || digitalRead(xlimit_pin1)==1){ // changed, so Head returns alway to bottom right corner -- Bartosz (03.07.17)
    if (digitalRead(ylimit_pin2)==1) {
      stepperMoveB(motorBbk);
    }
    if (digitalRead(xlimit_pin1)==1) {
      stepperMoveA(motorAfw);           // changed from motorAbk to motorAfw bc our PlotterHead starts on the right
    }   
    delayMicroseconds(mDelay);
  }
//  Serial.println("goHome!");
  posA = 0;
  posB = 0;
  curX = 0;
  curY = 0;
  tarX = 0;
  tarY = 0;
  tarA = 0;
  tarB = 0;
}

void initPosition()
{
  curX=0; curY=0;
  posA = 0;posB = 0;
}

/************** calculate movements ******************/
void parseCordinate(char * cmd)
{
  char * tmp;
  char * str;
  str = strtok_r(cmd, " ", &tmp);
  tarX = curX;
  tarY = curY;
  while(str!=NULL){
    str = strtok_r(0, " ", &tmp);
    if(str[0]=='X'){
      tarX = atof(str+1);
    }else if(str[0]=='Y'){
      tarY = atof(str+1);
    }else if(str[0]=='Z'){
      tarZ = atof(str+1);
    }else if(str[0]=='F'){
      float speed = atof(str+1);
      tarSpd = speed/60; // mm/min -> mm/s
    }else if(str[0]=='A'){
      stepAuxDelay = atol(str+1);
    }
  }
  prepareMove();
}

void echoRobotSetup()
{
  Serial.print("M10 XY ");
  Serial.print(roboSetup.data.width);Serial.print(' ');
  Serial.print(roboSetup.data.height);Serial.print(' ');
  Serial.print(curX);Serial.print(' ');
  Serial.print(curY);Serial.print(' ');
  Serial.print("A");Serial.print((int)roboSetup.data.motoADir);
  Serial.print(" B");Serial.print((int)roboSetup.data.motoBDir);
  Serial.print(" H");Serial.print((int)roboSetup.data.motorSwitch);
  Serial.print(" S");Serial.print((int)roboSetup.data.speed);
  Serial.print(" U");Serial.print((int)roboSetup.data.penUpPos);
  Serial.print(" D");Serial.println((int)roboSetup.data.penDownPos);
}

void echoEndStop()
{
  Serial.print("M11 ");
  Serial.print(digitalRead(ylimit_pin2)); Serial.print(" ");
  Serial.print(digitalRead(ylimit_pin1)); Serial.print(" ");
  Serial.print(digitalRead(xlimit_pin2)); Serial.print(" ");
  Serial.println(digitalRead(xlimit_pin1));
}

void syncRobotSetup()
{
  int i;
  for(i=0;i<64;i++){
    EEPROM.write(i,roboSetup.buf[i]);
  }
}

void parseRobotSetup(char * cmd)
{
  char * tmp;
  char * str;
  str = strtok_r(cmd, " ", &tmp);
  while(str!=NULL){
    str = strtok_r(0, " ", &tmp);
    if(str[0]=='A'){
      roboSetup.data.motoADir = atoi(str+1);
    }else if(str[0]=='B'){
      roboSetup.data.motoBDir = atoi(str+1);
    }else if(str[0]=='H'){
      roboSetup.data.height = atoi(str+1);
    }else if(str[0]=='W'){
      roboSetup.data.width = atoi(str+1);
    }else if(str[0]=='S'){
      roboSetup.data.speed = atoi(str+1);
    }
  }
  syncRobotSetup();
}

void parseAuxDelay(char * cmd)
{
  char * tmp;
  strtok_r(cmd, " ", &tmp);
  stepAuxDelay = atol(tmp);
}

void parseLaserPower(char * cmd)
{
  char * tmp;
  strtok_r(cmd, " ", &tmp);
  int pwm = atoi(tmp);
  if (pwm > 0) {          // apply changes to laser state variable
    laserOn = true;
  } else laserOn = false;
  laser.run(pwm);
}

void parsePen(char * cmd)
{
  char * tmp;
  strtok_r(cmd, " ", &tmp);
  int pos = atoi(tmp);
  servoPen.write(pos);
}

void parsePenPosSetup(char * cmd)
{
  char * tmp;
  char * str;
  str = strtok_r(cmd, " ", &tmp);
  while(str!=NULL){
    str = strtok_r(0, " ", &tmp);
    if(str[0]=='U'){
      roboSetup.data.penUpPos = atoi(str+1);
    }else if(str[0]=='D'){
      roboSetup.data.penDownPos = atoi(str+1);    
    }
  }
  syncRobotSetup();
}

/**
 * Plotter does goHome() to make sure he's in bottom right corner. After that measure 
 * x and y distance in steps to calculate how big the XY-Robot area is.
 * Robot scans the size it's moving on (Bartosz 28.09.17)
 */
void measureRobotSize() {

  long mDelay=STEPDELAY_HEALTHY_MIN;          // [micro seconds] delay that doesn't get resetted every run of the loop. basically the current speed = stepSize/mDelay

  long stepsUp     = 0;
  long stepsLeft   = 0;
  long stepsDown   = 0;
  long stepsRight  = 0;
  long tmp1, tmp2;


  goHome();
  if(true == process_serial())
    return;

  // Bewegung nach oben:    Bis Erreichen von ylimit_pin1:    stepperMoveB(motorBfw);
  while (digitalRead(ylimit_pin1)==1) {
    ++stepsUp;
    stepperMoveB(motorBfw);
    delayMicroseconds(mDelay);
  }
  if(true == process_serial())
    return;
  
  // Bewegung nach links:   Bis Erreichen von xlimit_pin2:    stepperMoveA(motorAbk);
  while (digitalRead(xlimit_pin2)==1) {
    ++stepsLeft;
    stepperMoveA(motorAbk);
    delayMicroseconds(mDelay);
  }
  if(true == process_serial())
    return;
  
  // Bewegung nach unten:   Bis Erreichen von ylimit_pin2:    stepperMoveB(motorBbk);  
  while (digitalRead(ylimit_pin2)==1) {
    ++stepsDown;
    stepperMoveB(motorBbk);
    delayMicroseconds(mDelay);
  }
  if(true == process_serial())
    return;
  
  // Bewegung nach rechts:  Bis Erreichen von xlimit_pin1:    stepperMoveA(motorAfw);
  while (digitalRead(xlimit_pin1)==1) {
    ++stepsRight;
    stepperMoveA(motorAfw);
    delayMicroseconds(mDelay);
  }

  // Format of Data: [RoboSize steps_in_Y_direction steps_in_X_direction STEPS_PER_MM RoboSize]
  Serial.print("RoboSize ");
  tmp1 = (stepsUp + stepsDown) / 2;
  tmp2 = (stepsRight + stepsLeft) / 2;
  Serial.print(tmp1); Serial.print(" ");
  Serial.print(tmp2); Serial.print(" ");
  Serial.print(STEPS_PER_MM); Serial.print(" ");
  Serial.println("RoboSize");
}

void parseMcode(char * cmd)
{
  int code;
  code = atoi(cmd);
  switch(code){
    case 1:
      parsePen(cmd);
      break;
    case 2: // set pen position
      parsePenPosSetup(cmd);
      break;
    case 3:
      parseAuxDelay(cmd);
      break;
    case 4:
      parseLaserPower(cmd);
      break;
    case 5:
      parseRobotSetup(cmd);
      break;      
    case 10:
      echoRobotSetup();
      break;
    case 11:
      echoEndStop();
      break;
    case 12:
      measureRobotSize();
  }
}

void parseGcode(char * cmd)
{
  int code;
  code = atoi(cmd);
  switch(code){
    case 0:
    case 1: // xyz move
      parseCordinate(cmd);
      break;
    case 28: // home
      stepAuxDelay = 0;
      tarX=0; tarY=0;
      servoPen.write(roboSetup.data.penUpPos);  // turning off to be handled by Controls
      laserOn = false;
      laser.run(0);                             // turning off to be handled by Controls
      goHome();
      break; 
  }
}

void parseCmd(char * cmd)
{
  if(cmd[0]=='G'){ // gcode
    parseGcode(cmd+1);  
  }else if(cmd[0]=='M'){ // mcode
    parseMcode(cmd+1);
  }else if(cmd[0]=='P'){
    Serial.print("POS X");Serial.print(curX);Serial.print(" Y");Serial.println(curY);
  }
  Serial.println("OK");
}

// local data
void initRobotSetup()
{
  int i;
  for(i=0;i<64;i++){
    roboSetup.buf[i] = EEPROM.read(i);
  }

  if(strncmp(roboSetup.data.name,"XY4",3)!=0){
    Serial.println("set to default setup");
    // set to default setup
    memset(roboSetup.buf,0,64);
    memcpy(roboSetup.data.name,"XY4",3);
    // default connection move inversely
    roboSetup.data.motoADir = 0;
    roboSetup.data.motoBDir = 0;
    roboSetup.data.width = WIDTH;
    roboSetup.data.height = HEIGHT;
    roboSetup.data.motorSwitch = 0;
    roboSetup.data.speed = 80;
    roboSetup.data.penUpPos = 160;
    roboSetup.data.penDownPos = 90;
    syncRobotSetup();
  }
  // init motor direction
  // yzj, match to standard connection of xy
  // A = x, B = y
  if(roboSetup.data.motoADir==0){
    motorAfw=-1;motorAbk=1;
  }else{
    motorAfw=1;motorAbk=-1;
  }
  if(roboSetup.data.motoBDir==0){
    motorBfw=-1;motorBbk=1;
  }else{
    motorBfw=1;motorBbk=-1;
  }
  int spd = 100 - roboSetup.data.speed;
  stepdelay_min = spd*10;
  stepdelay_max = spd*100;
}


/************** arduino ******************/
void setup() {
  pinMode(ylimit_pin1,INPUT_PULLUP);
  pinMode(ylimit_pin2,INPUT_PULLUP);
  pinMode(xlimit_pin1,INPUT_PULLUP);
  pinMode(xlimit_pin2,INPUT_PULLUP);
  Serial.begin(115200);
  initRobotSetup();
  initPosition();
  servoPen.attach(servopin);
  delay(100);
  servoPen.write(roboSetup.data.penUpPos);
  laserOn = false;
  laser.run(0);
}

char buf[64];
int8_t bufindex;

boolean process_serial(void)
{
  boolean result = false;
  memset(buf,0,64);
  bufindex = 0;
  while(Serial.available()){
    char c = Serial.read();
    buf[bufindex++]=c; 
    if(c=='\n'){
      buf[bufindex]='\0';
      parseCmd(buf);
      result = true;
      memset(buf,0,64);
      bufindex = 0;
    }
    if(bufindex>=64){
      bufindex=0;
    }
  }
  return result;
}

void loop() {
  if(Serial.available()){
    char c = Serial.read();
    buf[bufindex++]=c;
    if(c=='\n'){
      buf[bufindex]='\0';              // Robbo1 2015/6/8 Add     - Null terminate the string 
      parseCmd(buf);
      memset(buf,0,64);
      bufindex = 0;
    }
    if(bufindex>=64){
      bufindex=0;
    }
  }
}

