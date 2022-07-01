// Author: Sung Jik Cha
// Credits:
//   http://forum.arduino.cc/index.php?topic=8652.0
//   Dallaby   http://letsmakerobots.com/node/19558#comment-49685
//   Bill Porter  http://www.billporter.info/?p=286
//   bobbyorr (nice connection diagram) http://forum.pololu.com/viewtopic.php?f=15&t=1923

//ROS headers
#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif
#include <ros.h>
//#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Twist.h>
#include <ros/time.h>
#include "robot_specs.h"

//Motor Shield headers
#include <Wire.h>
//#include <Adafruit_MotorShield.h>
//#include "utility/Adafruit_PWMServoDriver.h"

/*
 #define encodPinA1      3     // encoder A pin
#define encodPinB1      8     // encoder B pin
#define encodPinA2      2
#define encodPinB2      7

*/
#define encoderL_A  18                  //#define encoderL_B  19
#define encoderR_A  20                  //#define encoderR_B  21
#define pwm_L       6
#define direction_L 7
#define pwm_R       8
#define direction_R 9

volatile long encoderL_APos=0;
volatile long encoderR_APos=0;

long LoldPosition = 0;
long RoldPosition = 0;

long LnewPosition;
long RnewPosition;


#define LOOPTIME        100   // PID loop time(ms)
#define SMOOTH      10 
#define sign(x) (x > 0) - (x < 0)

// Create the motor shield object with the default I2C address
//Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
// Select which 'port' M1, M2, M3 or M4. 
//Adafruit_DCMotor *motor1 = AFMS.getMotor(1);
//Adafruit_DCMotor *motor2 = AFMS.getMotor(2);

unsigned long lastMilli = 0;       // loop timing 
unsigned long lastMilliPub = 0;
double rpm_req1 = 0;
double rpm_req2 = 0;
double rpm_act1 = 0;
double rpm_act2 = 0;
double rpm_req1_smoothed = 0;
double rpm_req2_smoothed = 0;
int direction1 = FORWARD;
int direction2 = FORWARD;
int prev_direction1 = RELEASE;
int prev_direction2 = RELEASE;
int PWM_val1 = 0;
int PWM_val2 = 0;
volatile long count1 = 0;          // rev counter
volatile long count2 = 0;
long countAnt1 = 0;
long countAnt2 = 0;
float Kp =   0.5;
float Kd =   0;
float Ki =   0;
ros::NodeHandle nh;

geometry_msgs::Point32 Point_msg;

ros::Publisher enc_pub("/encoder", &Point_msg); 

void handle_cmd( const geometry_msgs::Twist& msg) {
  double x = msg.linear.x;
  double z = msg.angular.z;
  if (z == 0) {     // go straight
    // convert m/s to rpm
    rpm_req1 = x*60/(pi*wheel_diameter);
    rpm_req2 = rpm_req1;
  }
  else if (x == 0) {
    // convert rad/s to rpm
    rpm_req2 = z*track_width*60/(wheel_diameter*pi*2);
    rpm_req1 = -rpm_req2;
  }
  else {
    rpm_req1 = x*60/(pi*wheel_diameter)-z*track_width*60/(wheel_diameter*pi*2);
      rpm_req2 = x*60/(pi*wheel_diameter)+z*track_width*60/(wheel_diameter*pi*2);
  }
}


ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &handle_cmd);
//geometry_msgs::Vector3Stamped rpm_msg;
//ros::Publisher rpm_pub("rpm", &rpm_msg);
ros::Time current_time;
ros::Time last_time;

void setup() {
 Serial.begin (57600);
 //AFMS.begin();  // create with the default frequency 1.6KHz
 count1 = 0;
 count2 = 0;
 countAnt1 = 0;
 countAnt2 = 0;
 rpm_req1 = 0;
 rpm_req2 = 0;
 rpm_act1 = 0;
 rpm_act2 = 0;
 PWM_val1 = 0;
 PWM_val2 = 0;
 nh.initNode();
 nh.getHardware()->setBaud(57600);
 //nh.subscribe(sub);
 //nh.advertise(rpm_pub);
 nh.advertise(enc_pub);  
 nh.subscribe(sub); 
 
 pinMode(encoderL_APos, INPUT);
 digitalWrite(encoderL_APos, HIGH);       // turn on pullup resistor
      
 pinMode(encoderR_APos, INPUT);
 digitalWrite(encoderR_APos, HIGH);       // turn on pullup resistor
      
 attachInterrupt(digitalPinToInterrupt(18), doEncoderL_A, RISING);  // encoDER ON PIN 2
 attachInterrupt(digitalPinToInterrupt(20), doEncoderR_A, RISING);  // encoDER ON PIN 20
 /*
 pinMode(encodPinA1, INPUT); 
 pinMode(encodPinB1, INPUT); 
 digitalWrite(encodPinA1, HIGH);                // turn on pullup resistor
 digitalWrite(encodPinB1, HIGH);
 attachInterrupt(1, encoder1, RISING);

 pinMode(encodPinA2, INPUT); 
 pinMode(encodPinB2, INPUT); 
 digitalWrite(encodPinA2, HIGH);                // turn on pullup resistor
 digitalWrite(encodPinB2, HIGH);
 attachInterrupt(0, encoder2, RISING);
 */
 Serial.println("start");        
 pinMode(pwm_R ,OUTPUT);
 pinMode(direction_R ,OUTPUT);
 pinMode(pwm_R,OUTPUT);
 pinMode(direction_L ,OUTPUT);
 digitalWrite(direction_R, FORWARD);
 analogWrite(pwm_R, 200);
 analogWrite(pwm_R, 0);
 digitalWrite(direction_L, FORWARD);
 analogWrite(pwm_L, 200);
 analogWrite(pwm_L, 0);     
 /*
 motor1->setSpeed(0);
 motor2->setSpeed(0);
 motor1->run(FORWARD);
 motor1->run(RELEASE);
 motor2->run(FORWARD);
 motor2->run(RELEASE);
 */
 
}

void loop() {
  nh.spinOnce();
  unsigned long time = millis();
  if(time-lastMilli>= LOOPTIME)   {      // enter tmed loop
    getMotorData(time-lastMilli);
    PWM_val1 = updatePid(1, PWM_val1, rpm_req1, rpm_act1);
    PWM_val2 = updatePid(2, PWM_val2, rpm_req2, rpm_act2);

    if(PWM_val1 > 0) direction1 = FORWARD;
    else if(PWM_val1 < 0) direction1 = BACKWARD;
    if (rpm_req1 == 0) direction1 = RELEASE;
    if(PWM_val2 > 0) direction2 = FORWARD;
    else if(PWM_val2 < 0) direction2 = BACKWARD;
    if (rpm_req2 == 0) direction2 = RELEASE;

    digitalWrite(direction_L, direction1);
    analogWrite(pwm_L, PWM_val1);
    
    digitalWrite(direction_R, direction2);
    analogWrite(pwm_R, PWM_val2);
    
    /*
    motor1->run(direction1);
    motor2->run(direction2);

    motor1->setSpeed(abs(PWM_val1));
    motor2->setSpeed(abs(PWM_val2));
    */

    puplish_encoder();
    
    //publishRPM(time-lastMilli);
    lastMilli = time;
  }
  if(time-lastMilliPub >= LOOPTIME) {
  //  publishRPM(time-lastMilliPub);
    lastMilliPub = time;
  }
}

void getMotorData(unsigned long time)  {
 rpm_act1 = double((count1-countAnt1)*60*1000)/double(time*encoder_pulse);           //*gear_ratio 
 rpm_act2 = double((count2-countAnt2)*60*1000)/double(time*encoder_pulse);          //*gear_ratio
 countAnt1 = count1;
 countAnt2 = count2;
}

int updatePid(int id, int command, double targetValue, double currentValue) {
  double pidTerm = 0;                            // PID correction
  double error = 0;
  double new_pwm = 0;
  double new_cmd = 0;
  static double last_error1 = 0;
  static double last_error2 = 0;
  static double int_error1 = 0;
  static double int_error2 = 0;
  
  error = targetValue-currentValue;
  if (id == 1) {
    int_error1 += error;
    pidTerm = Kp*error + Kd*(error-last_error1) + Ki*int_error1;
    last_error1 = error;
  }
  else {
    int_error2 += error;
    pidTerm = Kp*error + Kd*(error-last_error2) + Ki*int_error2;
    last_error2 = error;
  }
  new_pwm = constrain(double(command)*MAX_RPM/4095.0 + pidTerm, -MAX_RPM, MAX_RPM);
  new_cmd = 4095.0*new_pwm/MAX_RPM;
  return int(new_cmd);
}

void puplish_encoder(){
  RnewPosition = encoderR_APos;
     if (RnewPosition != RoldPosition) 
     {
          RoldPosition = RnewPosition; 
         Serial.print ("Right Encder = ");
         Serial.println(RnewPosition);
      } 
  LnewPosition = encoderL_APos;
  if (LnewPosition != LoldPosition) 
  {
      LoldPosition = LnewPosition; //update positions
      Serial.print ("left Encoder = ");
      Serial.println(LnewPosition);
  }        
  Point_msg.x=RnewPosition;
  Point_msg.y=LnewPosition;
  enc_pub.publish(&Point_msg);
  nh.spinOnce();
  // delay(10);
}

/*
  void publishRPM(unsigned long time) {
  rpm_msg.header.stamp = nh.now();
  rpm_msg.vector.x = rpm_act1;
  rpm_msg.vector.y = rpm_act2;
  rpm_msg.vector.z = double(time)/1000;
  rpm_pub.publish(&rpm_msg);
  nh.spinOnce();
}
*/

void doEncoderL_A() {
  if(digitalRead(direction_L) == 0)
    {
      count1++;                        //encoderL_APos++;
    }
    else
    {
      count1--;                         //encoderL_APos--;
    } 
  /*
  if (digitalRead(encodPinA1) == digitalRead(encodPinB1)) count1++;
  else count1--;
  */
}
void doEncoderR_A() {
  if(digitalRead(direction_R) == 0)
    { 
      count2--;                        //encoderR_APos++;
    }
    else
    { 
      count2++;                        //encoderR_APos--;
    } 
  /*
  if (digitalRead(encodPinA2) == digitalRead(encodPinB2)) count2--;
  else count2++;
   */
  
}
