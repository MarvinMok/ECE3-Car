#include <ECE3.h>
#include <ArduinoQueue.h>

uint16_t sensorValues[8];

const int minimums[8] =  {574, 597, 597, 597, 592, 666, 727, 690} ;
const int maximums[8] = {1854, 1730, 1801, 1265, 1450, 1781, 1773, 1810};
const int scaling_factor[8] = {-4,-3,-2,-1,1,2,3,4};

const int left_nslp_pin=31; // nslp ==> awake & ready for PWM
const int left_dir_pin=29;
const int left_pwm_pin=40;

const int right_nslp_pin=11; // nslp ==> awake & ready for PWM
const int right_dir_pin=30;
const int right_pwm_pin=39;

const float kp = 0.01;
const float kd = 0.01;
const float ki = 0.01;

const int q_size = 3;

//ArduinoQueue<int> errorQ(q_size);
int errorArr[q_size];
int loc = 0;
int curError;
int sum;

const int base_speed = 50;

int getDError();
int getError();
//+ error means turn right

void setup()
{
  ECE3_Init();
  Serial.begin(9600); // set the data rate in bits per second for serial data transmission

  pinMode(left_nslp_pin,OUTPUT);
  pinMode(left_dir_pin,OUTPUT);
  pinMode(left_pwm_pin,OUTPUT);

  pinMode(right_nslp_pin,OUTPUT);
  pinMode(right_dir_pin,OUTPUT);
  pinMode(right_pwm_pin,OUTPUT);

  digitalWrite(right_nslp_pin,HIGH);
  digitalWrite(left_nslp_pin,HIGH);
  
  digitalWrite(left_dir_pin,LOW);
  digitalWrite(right_dir_pin,LOW);

  curError = getError();

  for (int i = 0; i < q_size; i++)
  {
    //errorQ.enqueue(curError);
    errorArr[i] = curError;
  }
  sum = 3 * curError;
  
  delay(2000);
}


void loop()
{
  // read raw sensor values
  ECE3_read_IR(sensorValues);

  curError = getError();
  int d_error = getDError();

  analogWrite(left_pwm_pin, base_speed + kp * curError + kd * d_error);
  analogWrite(right_pwm_pin, base_speed - kp * curError - kd * d_error);
 
//  delay(100);
}

//int getDError2()
//{
//  int change = curError - (sum / q_size);
//  sum += curError - errorQ.dequeue();
//  errorQ.enqueue(curError);
//  return change;
//}

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
  for (int i = 0;  i < 8; i++)
  {
    error += scaling_factor[i] * (sensorValues[i] - minimums[i]) / (maximums[i] - minimums[i]) * 1000;
  }
  return error / 4;
}
