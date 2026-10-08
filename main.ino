// ESP32 + RLS-08 + TB6612FNG
// White line on black surface
// A1 = LEFT, A8 = RIGHT

const int sensorPins[8] = {36,39,34,35,32,33,25,26};

const int whiteReading[8] = {65,53,231,56,66,754,323,94};
const int blackReading[8] = {3154,2931,3594,2757,3037,3694,2724,2967};

const int PWMA=27,AIN1=14,AIN2=13;
const int PWMB=23,BIN1=18,BIN2=19;
const int STBY=21;

const bool LEFT_MOTOR_REVERSED=true;
const bool RIGHT_MOTOR_REVERSED=true;

const int PWM_FREQ=18000;
const int PWM_BITS=12;
const int PWM_MAX=4095;

const int BASE_SPEED=1600;

const float RIGHT_MOTOR_GAIN=1.15f;

const int TURN_INNER=500;
const int TURN_OUTER=1800;

const int JUNCTION_SPEED=1000;

const float KP=0.88f;
const float KD=1.28f;

const int BRANCH_STRENGTH=450;
const int JUNCTION_CONFIRM=3;
const int TURN_START_TIME=100;
const int TURN_TIMEOUT=1200;
const int DEAD_END_CONFIRM=20;

int lastError=0;
int junctionCount=0;
int deadEndCount=0;

unsigned long lastJunction=0;

int readLinePosition(bool &lineFound){
  long weightedSum=0;
  long strengthSum=0;

  for(int i=0;i<8;i++){
    int raw=analogRead(sensorPins[i]);
    int span=blackReading[i]-whiteReading[i];

    if(span<=0) continue;

    int strength=
      (int)((long)(blackReading[i]-raw)*1000/span);

    strength=constrain(strength,0,1000);

    if(strength>250){
      weightedSum+=(long)strength*(i*1000);
      strengthSum+=strength;
    }
  }

  lineFound=strengthSum>0;

  if(!lineFound)
    return -1;

  return (int)(weightedSum/strengthSum);
}

void setMotor(int pwmPin,int in1,int in2,int speed,bool reversed){
  speed=constrain(speed,-PWM_MAX,PWM_MAX);

  if(reversed)
    speed=-speed;

  if(speed>=0){
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
  }
  else{
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
  }

  ledcWrite(pwmPin,abs(speed));
}

void drive(int leftSpeed,int rightSpeed){

  rightSpeed=(int)(rightSpeed*RIGHT_MOTOR_GAIN);

  leftSpeed=constrain(leftSpeed,-PWM_MAX,PWM_MAX);
  rightSpeed=constrain(rightSpeed,-PWM_MAX,PWM_MAX);

  setMotor(
    PWMA,
    AIN1,
    AIN2,
    leftSpeed,
    LEFT_MOTOR_REVERSED
  );

  setMotor(
    PWMB,
    BIN1,
    BIN2,
    rightSpeed,
    RIGHT_MOTOR_REVERSED
  );
}

void stopMotors(){
  drive(0,0);
}

void readRaw(int raw[8]){
  for(int i=0;i<8;i++)
    raw[i]=analogRead(sensorPins[i]);
}

int whiteStrength(int i,int raw){
  int span=blackReading[i]-whiteReading[i];

  if(span<=0)
    return 0;

  int strength=
    (int)((long)(blackReading[i]-raw)*1000/span);

  return constrain(strength,0,1000);
}

bool seesWhite(int i,int raw[8]){
  return whiteStrength(i,raw[i])>250;
}

bool centerWhite(int raw[8]){
  return seesWhite(3,raw)||seesWhite(4,raw);
}

bool linePresent(int raw[8]){
  for(int i=0;i<8;i++){
    if(whiteStrength(i,raw[i])>250)
      return true;
  }

  return false;
}

bool leftBranch(int raw[8]){
  return
    whiteStrength(0,raw[0])>BRANCH_STRENGTH ||
    whiteStrength(1,raw[1])>BRANCH_STRENGTH ||
    whiteStrength(2,raw[2])>BRANCH_STRENGTH;
}

bool rightBranch(int raw[8]){
  return
    whiteStrength(5,raw[5])>BRANCH_STRENGTH ||
    whiteStrength(6,raw[6])>BRANCH_STRENGTH ||
    whiteStrength(7,raw[7])>BRANCH_STRENGTH;
}

bool junctionDetected(){
  int raw[8];
  readRaw(raw);

  bool left=leftBranch(raw);
  bool right=rightBranch(raw);
  bool center=centerWhite(raw);

  if(center&&(left||right))
    junctionCount++;
  else
    junctionCount=0;

  if(junctionCount>=JUNCTION_CONFIRM){
    junctionCount=0;
    return true;
  }

  return false;
}

void followLine(){
  bool lineFound;
  int position=readLinePosition(lineFound);

  if(!lineFound){
    stopMotors();
    return;
  }

  int error=position-3500;
  int derivative=error-lastError;

  lastError=error;

  int correction=
    (int)(KP*error+KD*derivative);

  int leftSpeed=constrain(
    BASE_SPEED+correction,
    -PWM_MAX,
    PWM_MAX
  );

  int rightSpeed=constrain(
    BASE_SPEED-correction,
    -PWM_MAX,
    PWM_MAX
  );

  drive(leftSpeed,rightSpeed);

  delay(5);
}

char inspectJunction(){
  int raw[8];
  readRaw(raw);

  bool left=leftBranch(raw);
  bool right=rightBranch(raw);

  bool straight=
    whiteStrength(2,raw[2])>BRANCH_STRENGTH ||
    whiteStrength(3,raw[3])>BRANCH_STRENGTH ||
    whiteStrength(4,raw[4])>BRANCH_STRENGTH ||
    whiteStrength(5,raw[5])>BRANCH_STRENGTH;

  if(left)
    return 'L';

  if(straight)
    return 'S';

  if(right)
    return 'R';

  return 'B';
}

void smoothTurn(bool left){
  int raw[8];

  unsigned long start=millis();

  if(left)
    drive(TURN_INNER,TURN_OUTER);
  else
    drive(TURN_OUTER,TURN_INNER);

  delay(TURN_START_TIME);

  while(millis()-start<TURN_TIMEOUT){

    readRaw(raw);

    int position;
    bool lineFound;

    position=readLinePosition(lineFound);

    bool sideLine=
      whiteStrength(0,raw[0])>BRANCH_STRENGTH ||
      whiteStrength(7,raw[7])>BRANCH_STRENGTH;

    if(lineFound &&
       position>2700 &&
       position<4300 &&
       !sideLine){

      stopMotors();
      delay(20);
      return;
    }

    if(left)
      drive(TURN_INNER,TURN_OUTER);
    else
      drive(TURN_OUTER,TURN_INNER);

    delay(3);
  }

  stopMotors();
}

void handleJunction(){
  char direction=inspectJunction();

  if(direction=='L'){
    smoothTurn(true);
  }
  else if(direction=='S'){
    followLine();
  }
  else if(direction=='R'){
    smoothTurn(false);
  }
  else{
    smoothTurn(true);
  }

  lastJunction=millis();
}

void handleDeadEnd(){
  stopMotors();
  delay(30);

  drive(-TURN_OUTER,TURN_OUTER);

  delay(250);

  unsigned long start=millis();

  while(millis()-start<TURN_TIMEOUT){

    int raw[8];
    readRaw(raw);

    bool center=centerWhite(raw);

    if(center){
      stopMotors();
      delay(20);
      return;
    }

    drive(-TURN_OUTER,TURN_OUTER);

    delay(3);
  }

  stopMotors();
}

void setup(){
  Serial.begin(115200);
  analogReadResolution(12);

  pinMode(AIN1,OUTPUT);
  pinMode(AIN2,OUTPUT);
  pinMode(BIN1,OUTPUT);
  pinMode(BIN2,OUTPUT);
  pinMode(STBY,OUTPUT);

  bool leftPwmOk=
    ledcAttach(PWMA,PWM_FREQ,PWM_BITS);

  bool rightPwmOk=
    ledcAttach(PWMB,PWM_FREQ,PWM_BITS);

  digitalWrite(STBY,HIGH);
  drive(0,0);

  if(!leftPwmOk||!rightPwmOk){
    while(true){
      drive(0,0);
      delay(100);
    }
  }

  delay(500);
}

void loop(){
  int raw[8];
  readRaw(raw);

  if(!linePresent(raw)){
    deadEndCount++;

    if(deadEndCount>=DEAD_END_CONFIRM){
      deadEndCount=0;
      handleDeadEnd();
    }

    delay(5);
    return;
  }

  deadEndCount=0;

  if(millis()-lastJunction>300){

    if(junctionDetected()){
      handleJunction();
      delay(50);
      return;
    }
  }

  followLine();
}