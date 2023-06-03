#include <ECE3.h>
//turn 180 is 340ish
uint16_t sensorValues[8];
int turn_around_count = 0;
int last_right;
int last_left;
const int minimums[8] =  {552, 552, 552, 552, 528, 623, 670, 646};
const int maximums = 2100;
int scaling_factor[8] = { -4, -3, -2, -1, 1, 2, 3, 4};

const int left_nslp_pin = 31; // nslp ==> awake & ready for PWM
const int left_dir_pin = 29;
const int left_pwm_pin = 40;

//const uint16_t encoder[4] = [0, 100, 200, 10000000];
//const uint16_t base_speeds[4] = [70, 70, 70, 0];
unsigned char section = 0;

const int right_nslp_pin = 11; // nslp ==> awake & ready for PWM
const int right_dir_pin = 30;
const int right_pwm_pin = 39;

float kp = 0.475;
float kd = 2.5;

bool changedSpeed = false;

const int q_size = 1;

//ArduinoQueue<int> errorQ(q_size);
int errorArr[q_size];
unsigned char loc = 0;
int curError;
int sum;

int encoder_count = 0;

bool turned = false;
int base_speed = 60;

int getDError();
int getError();
//+ error means turn right


void changeWheelSpeed(int li, int lf, int ri, int rf);
void setup()
{
  ECE3_Init();
  
  resetEncoderCount_left();
  resetEncoderCount_right();

  encoder_count = getEncoderCount_left();
  
  Serial.begin(9600); // set the data rate in bits per second for serial data transmission

  pinMode(left_nslp_pin, OUTPUT);
  pinMode(left_dir_pin, OUTPUT);
  pinMode(left_pwm_pin, OUTPUT);

  pinMode(right_nslp_pin, OUTPUT);
  pinMode(right_dir_pin, OUTPUT);
  pinMode(right_pwm_pin, OUTPUT);

  digitalWrite(right_nslp_pin, HIGH);
  digitalWrite(left_nslp_pin, HIGH);

  digitalWrite(left_dir_pin, LOW);
  digitalWrite(right_dir_pin, LOW);

  ECE3_read_IR(sensorValues);
  curError = getError();

  for (int i = 0; i < q_size; i++)
  {
    //errorQ.enqueue(curError);
    errorArr[i] = curError;
  }
  sum = curError * q_size;
  
  delay(2000);

  //changeWheelSpeed(0, base_speed, 0, base_speed);
  analogWrite(left_pwm_pin, base_speed);
  analogWrite(right_pwm_pin, base_speed);  
  
}


void loop()
{
  encoder_count = (getEncoderCount_left() + getEncoderCount_right()) / 2;
  // read raw sensor values
  ECE3_read_IR(sensorValues);

  curError = getError();
  int d_error = getDError();

  //   for (unsigned char i = 0; i < 8; i++)
  //   {
  //    Serial.print(sensorValues[i]);
  //    Serial.print('\t'); // tab to format the raw data into columns in the Serial monitor
  //   }
  //   Serial.println();

   //Serial.println(encoder_count);

  if (encoder_count >= 2010 && encoder_count < 2015) {
    base_speed = 150;
    int new_right = base_speed + kp * curError + kd * d_error;
    int new_left = base_speed - kp * curError - kd * d_error;
    ChangeBaseSpeeds(last_left, new_left, last_right, new_right);
  }
  else if (encoder_count >= 3400 && encoder_count < 3410)
  {
    scaling_factor[0] = 0;
    scaling_factor[7] = 0;
    kp = 0.55;
    kd = 0.3;
    
    base_speed = 20;
    int new_right = base_speed + kp * curError + kd * d_error;
    int new_left = base_speed - kp * curError - kd * d_error;
    ChangeBaseSpeeds(last_left, new_left, last_right, new_right);
    last_left = new_left;
    last_right = new_right;
//    delay(100);
//    analogWrite(right_pwm_pin, 20);
//    delay(5);
//    analogWrite(left_pwm_pin, 20);
//    delay(10);
  }
  else if (encoder_count >= 3950 && encoder_count < 3955)
  {
    scaling_factor[0] = -4;
    scaling_factor[7] = 4;
    
    base_speed = 60;
    kp = 0.45;
    kd = 2.5;

    int new_right = base_speed + kp * curError + kd * d_error;
    int new_left = base_speed - kp * curError - kd * d_error;
    ChangeBaseSpeeds(last_left, new_left, last_right, new_right);
    last_left = new_left;
    last_right = new_right;
  }
  if (encoder_count >= 6650 && encoder_count < 6655) {
    base_speed = 130;
    int new_right = base_speed + kp * curError + kd * d_error;
    int new_left = base_speed - kp * curError - kd * d_error;
    ChangeBaseSpeeds(last_left, new_left, last_right, new_right);
  }
  if (encoder_count >= 7950 && encoder_count < 7960) {
    base_speed = 50;
    int new_right = base_speed + kp * curError + kd * d_error;
    int new_left = base_speed - kp * curError - kd * d_error;
    ChangeBaseSpeeds(last_left, new_left, last_right, new_right);
  }
  turnAroundTest();
  if (turn_around_count > 1)
  {
    if (!turned)
    {
      turnAround();
      turned = true;
    }
    else
    {
//      base_speed = 0;
//      ChangeBaseSpeeds(last_left, 0, last_right,0);
      analogWrite(left_pwm_pin, 0);
      analogWrite(right_pwm_pin, 0);
      exit(0);
    }
  } else {
    last_left = base_speed - kp * curError - kd * d_error;
    last_right = base_speed + kp * curError + kd * d_error;
    analogWrite(left_pwm_pin, last_left);
    analogWrite(right_pwm_pin, last_right);
  }
  
  //delay(1000);
}


int getDError()
{
  int change = curError - (sum / q_size);
  sum += curError - errorArr[loc];
  errorArr[loc] = curError;
  loc = (loc + 1) % q_size;
  return change;
}

int getError()
{
  int error = 0;
  for (unsigned char i = 0;  i < 8; i++)
  {
    if (sensorValues[i] < minimums[i])
    {
      sensorValues[i] = minimums[i];
    }
    error += scaling_factor[i] * (sensorValues[i] - minimums[i]) / (float) (maximums - minimums[i]) * 25;
  }
  return error;
}

void turnAroundTest()
{
  bool b = false;
  uint16_t s = 0;
  for (unsigned char i = 1; i < 7; i++)
  {
    s += sensorValues[i];
  }
  if (s / 6 > 1950)
  {
    b = true;
  }
  if (b)
  {
    turn_around_count++;
  }
  else
  {
    turn_around_count = 0;
  }
}

void turnAround()
{

//  if (getEncoderCount_left() < encoder[section + 1])
//  {
//    base_speed = encoder[section + 1];
//    section++;
//  }

  int cur_left = getEncoderCount_left();

  analogWrite(left_pwm_pin, 0);
  analogWrite(right_pwm_pin, 0);
  digitalWrite(left_dir_pin, LOW);
  digitalWrite(right_dir_pin, HIGH);
  while (getEncoderCount_left() - cur_left < 350)
  {
    
    analogWrite(left_pwm_pin, 90);
    analogWrite(right_pwm_pin, 90);
    
  }

  analogWrite(left_pwm_pin, 0);
  analogWrite(right_pwm_pin, 0);
  digitalWrite(left_dir_pin, LOW);
  digitalWrite(right_dir_pin, LOW);

  turn_around_count = 0;
}

void changeWheelSpeed(int li, int lf, int ri, int rf)
{
  int dl = (lf - li);
  int dr = (rf - ri);
  int steps = min(abs(dl), abs(dr)) >> 3;
  int lc = li;
  int rc = rc;
  for (unsigned char i = 0; i < steps; i++)
  {
    lc += 24;
    rc += 24;
    analogWrite(left_pwm_pin, lc);
    analogWrite(right_pwm_pin, rc);
    delay (30);
  }
  analogWrite(left_pwm_pin, lf);
  analogWrite(right_pwm_pin, rf);

  last_right = rf;
  last_left = lf;
}

//---------------------------------------------------------
void ChangeBaseSpeeds(int initialLeftSpd,int finalLeftSpd,int
initialRightSpd,int finalRightSpd) {
  /*
  * This function changes the car speed gradually (in about 30 ms) from
  initial
  * speed to final speed. This non-instantaneous speed change reduces the
  load
  * on the plastic geartrain, and reduces the failure rate of the motors.
  */
  int diffLeft = finalLeftSpd-initialLeftSpd;
  int diffRight = finalRightSpd-initialRightSpd;
  int stepIncrement = 20;
  int numStepsLeft = abs(diffLeft)/stepIncrement;
  int numStepsRight = abs(diffRight)/stepIncrement;
  int numSteps = max(numStepsLeft,numStepsRight);
  int pwmLeftVal = initialLeftSpd; // initialize left wheel speed
  int pwmRightVal = initialRightSpd; // initialize right wheel speed
  int deltaLeft = (diffLeft)/numSteps; // left in(de)crement
  int deltaRight = (diffRight)/numSteps; // right in(de)crement
  for(int k=0;k<numSteps;k++) {
    pwmLeftVal = pwmLeftVal + deltaLeft;
    pwmRightVal = pwmRightVal + deltaRight;
    analogWrite(left_pwm_pin,pwmLeftVal);
    analogWrite(right_pwm_pin,pwmRightVal);
    delay(30);
  } // end for int k
  analogWrite(left_pwm_pin,finalLeftSpd);
  analogWrite(right_pwm_pin,finalRightSpd);
} 
