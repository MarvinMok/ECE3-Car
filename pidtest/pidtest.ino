#include <ECE3.h>
//turn 180 is 340ish
uint16_t sensorValues[8];
int turn_around_count = 0;
int last_right;
int last_left;
const int minimums[8] =  {574, 597, 597, 597, 592, 666, 727, 690} ;
const int maximums[8] = {1854, 1730, 1801, 1265, 1450, 1781, 1773, 1810};
const int scaling_factor[8] = { -4, -3, -2, -1, 1, 2, 3, 4};

const int left_nslp_pin = 31; // nslp ==> awake & ready for PWM
const int left_dir_pin = 29;
const int left_pwm_pin = 40;

//const uint16_t encoder[4] = [0, 100, 200, 10000000];
//const uint16_t base_speeds[4] = [70, 70, 70, 0];
unsigned char section = 0;

const int right_nslp_pin = 11; // nslp ==> awake & ready for PWM
const int right_dir_pin = 30;
const int right_pwm_pin = 39;

const float kp = 0.375;
const float kd = 2.75;


const int q_size = 3;

//ArduinoQueue<int> errorQ(q_size);
int errorArr[q_size];
unsigned char loc = 0;
int curError;
int sum;

int base_speed = 70;

int getDError();
int getError();
//+ error means turn right


void changeWheelSpeed(int li, int lf, int ri, int rf);
void setup()
{

  resetEncoderCount_left();
  resetEncoderCount_right();
  
  ECE3_Init();
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

  curError = getError();

  for (int i = 0; i < q_size; i++)
  {
    //errorQ.enqueue(curError);
    errorArr[i] = curError;
  }
  sum = curError + curError >> 2;
  
  delay(2000);

  changeWheelSpeed(0, base_speed, 0, base_speed);  
  
}


void loop()
{
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


  //Serial.println(curError);
  //Serial.println(d_error);

//  turnAroundTest();
//  if (turn_around_count > 3)
//  {
//    turnAround();
//  }
//  else
//  {
    analogWrite(left_pwm_pin, base_speed - kp * curError - kd * d_error);
    analogWrite(right_pwm_pin, base_speed + kp * curError + kd * d_error);
    
 // }


  //delay(100);
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
    error += scaling_factor[i] * (sensorValues[i] - minimums[i]) / (float) (maximums[i] - minimums[i]) * 25;
  }
  return error;
}

void turnAroundTest()
{

  bool b = true;
  for (unsigned char i = 0; i < 8; i++)
  {
    if (sensorValues[i] < 2000)
    {
      b = false;
    }
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

  changeWheelSpeed(last_right, 0, last_right, 0);
  
  while (getEncoderCount_left() - cur_left < 310)
  {
    analogWrite(left_pwm_pin, base_speed);
    analogWrite(right_pwm_pin, -base_speed);
  }

  while (getEncoderCount_left() - cur_left < 340) //dont pound gears too hard;
  {
    analogWrite(left_pwm_pin, base_speed / 2);
    analogWrite(right_pwm_pin, -base_speed / 2);
  }
  
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
