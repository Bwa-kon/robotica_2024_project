#define USE_USBCON
#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>

void motor_setpoint_callback(const std_msgs::Float32& x);
void motor_direction_callback(const std_msgs::Bool& x);

ros::NodeHandle  nh;

std_msgs::Float32 ros_error;
std_msgs::Float32 ros_current_speed;

ros::Subscriber<std_msgs::Float32> sub_motor_speed("/motor_info/setpoint", &motor_setpoint_callback);
ros::Subscriber<std_msgs::Bool> sub_motor_direction("/motor_info/direction", &motor_direction_callback);
ros::Publisher pub_error("/motor_info/error", &ros_error);
ros::Publisher pub_current_speed("/motor_info/current_speed", &ros_current_speed);

// Defines global setpoint
bool direction = false;
float speed = 0;

// Define encoder pulses
unsigned int pulses_per_rotation = 11;
volatile long encoder_pos = 0;
long encoder_pos_last = 0;
unsigned long time_loop = 0;

// Defines pins numbers
const int inputPin1 = 5;
const int inputPin2 = 6;
const int encoderPinA = 13;
const int encoderPinB = 2;

// Defines control const
const float kp = 0.55;
const float ki = 0.45;
const float kd = 0.060;

float v1Filt = 0;
float v1Prev = 0;

//defines control limit 
const float upper_limit = 70 + 0.5;
const float lower_limit = 70 - 0.5;

// Variables for the PID controller
float proportional = 0, integral = 0, derivative = 0;
float old_error = 0;

void motor_setpoint_callback(const std_msgs::Float32& x)
{
  speed = x.data;
}

void motor_direction_callback(const std_msgs::Bool& x)
{
  direction = x.data;
}

void read_encoder(void)
{
  // If encoder B is active
  if (digitalRead(encoderPinB) > 0)
  {
    // Motor has moved forward
    encoder_pos += 1;
  }
  else
  {
    // Motor has moved backward
    encoder_pos -= 1;
  }
}

void set_motorspeed(unsigned int s, bool d)
{
  // Limit speed to 0-255
  s = min(s, 255);
  s = max(s, 0);

  if (d)
  {
    analogWrite(inputPin1, 0);
    analogWrite(inputPin2, s);
  }

  else
  {
    analogWrite(inputPin2, 0);
    analogWrite(inputPin1, s);
  }
}

void setup()
{
  pinMode(inputPin1, OUTPUT);
  pinMode(inputPin2, OUTPUT);
  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);

  nh.initNode();

  nh.subscribe(sub_motor_speed);
  nh.subscribe(sub_motor_direction);
  nh.advertise(pub_error);
  nh.advertise(pub_current_speed);

  // Stop motor
  analogWrite(inputPin1,0);
  analogWrite(inputPin2,0);

  attachInterrupt(digitalPinToInterrupt(encoderPinA), read_encoder, RISING);
}

void loop()
{
  unsigned long time_now = micros();
  unsigned long time_old = time_loop;
  time_loop = time_now;
  float time_diff = 0;
  float current_speed = 0;

  // Atomically read position to avoid potential misreads
  noInterrupts();
  long pulse = encoder_pos;
  interrupts();

  // Calculate position difference
  long pos = pulse - encoder_pos_last;
  float rad = (float) pos*2*PI/pulses_per_rotation;

  if (rad != 0 && time_now > time_old)
  {
    // Calculate speed using difference in time
    time_diff = ((float) time_now - time_old) / 1e6;
    current_speed = rad / time_diff;
  }

   // Low-pass filter
  v1Filt = 0.8856*v1Filt + 0.0155*current_speed + 0.0155*v1Prev;
  v1Prev = current_speed;

  // Save position for later
  encoder_pos_last = encoder_pos;

  // Initialize error variables
  float error;
  if (direction)
  {
    error = -1 * speed - v1Filt;
  }

  else
  {
    error = speed - v1Filt;
  }

  // Calculate the difference in error compared to the previous iteration
  float change = error - old_error;

  // Proportional control
  proportional = error;

  // Integral control
  integral = integral + error * time_diff;

  // Derivative control
  derivative = change/time_diff;
  
  // Calculate the needed voltage
  float output = kp * proportional + ki * integral + kd * derivative;

  // Record the current error for the next iteration
  old_error = error;

  // Determine direction
  bool dir = false;
  if (output < 0)
  {
    dir = true;
  }

  // Prevent very slow speed
  if (speed < 1)
  {
    // Force stopping motor
    output = 0;
  }

  // Limit setpoint and make it absolute
  unsigned int setpoint = abs(fabs(output));
  if (setpoint >= 255)
  {
    setpoint = 255;
  }

  // Publisher ROS data
  ros_error.data = error;
  ros_current_speed.data = current_speed;
  pub_error.publish(&ros_error);
  pub_current_speed.publish(&ros_current_speed);

  set_motorspeed(setpoint, dir);
  nh.spinOnce();
  delay(50);
}
