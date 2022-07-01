
#include <ros.h>
#include <geometry_msgs/Point32.h> //For sending encoder msg
#include<geometry_msgs/Twist.h> //For cmd_vel subscription
// #include <AFMotor.h> //Adafruit mototr library
// #include <Encoder.h> //Encoder library

geometry_msgs::Twist _msg;
ros::Publisher mineMessage("mineMessage", &_msg);

//macros for Encoders and motors
#define encoderL_A  18                  
#define encoderL_B  19
#define encoderR_A  20                 
#define encoderR_B  21
#define pwm_L       6
#define direction_L 7
#define pwm_R       8
#define direction_R 9

long RnewPosition;
long LnewPosition;

long R_AnewPosition;
long L_AnewPosition;

long R_BnewPosition;
long L_BnewPosition;

volatile long encoderL_APos=0;
volatile long encoderR_APos=0;
volatile long encoderL_BPos=0;
volatile long encoderR_BPos=0;

//long newposition;
long L_AoldPosition = 0;
long R_AoldPosition = 0;

long L_BoldPosition = 0;
long R_BoldPosition = 0;



//----- DC Motor definitions------------
//AF_DCMotor motorL(4);  //left motor goes for pin 4
//AF_DCMotor motorR(3);  //right motor goes for pin 3

//-------------Encoder definitions
//Encoder R_enc(19, 18); //right motor encoder goes for pins 19 18
//Encoder L_enc(21, 20); //left motor encoder goes for pins 21 20
//long RoldPosition  = -999;
//long LoldPosition  = -999;
 //-----------------------------------------Robot parameters definition------------ 
#define L 0.40
#define R 0.1
//--------------------------------Motors VARS-----------------------------------
// initializing variables
float vel=0.0; //as Twist msgs depend on Vector3 which is float64
float omega=0.0;
float VR,VL;
//-----------------------------------------------------------------------------------------
ros::NodeHandle  nh;
//------------------------------Publish init----------------------------------------------
geometry_msgs::Point32 Point_msg;

ros::Publisher enc_pub("/encoder", &Point_msg); 


//-----------------------------------DC Motors Callback subscribers

void doEncoderL_A()
{
    if(digitalRead(direction_L) == 0)
    {
       encoderL_APos++;
    }
    else
    {
      encoderL_APos--;
    }       
}

void doEncoderR_A()
{
    if(digitalRead(direction_R) == 0)
    {
       encoderR_APos++;
    }
    else
    {
      encoderR_APos--;
    }  
}
void doEncoderL_B()
{
    if(digitalRead(direction_L) == 0)
    {
       encoderL_BPos++;
    }
    else
    {
      encoderL_BPos--;
    }       
}

void doEncoderR_B()
{
    if(digitalRead(direction_R) == 0)
    {
       encoderR_BPos++;
    }
    else
    {
      encoderR_BPos--;
    }  
}

void printDebug()
{
    char buffer[50];

    sprintf (buffer, "Encoder FrontLeft  : %ld", LnewPosition);
    nh.loginfo(buffer);
    sprintf (buffer, "Encoder FrontRight : %ld", RnewPosition);
    nh.loginfo(buffer);
    //sprintf (buffer, "Encoder RearLeft   : %ld", L_BnewPosition);
    //nh.loginfo(buffer);
    //sprintf (buffer, "Encoder RearRight  : %ld", R_BnewPosition);
    //nh.loginfo(buffer);
}


void motors_cb(const geometry_msgs::Twist& msg)
{
 
    vel=msg.linear.x;    
    omega=msg.angular.z;  
    
     
    VR=(2*vel+omega*L)/(2*R); 
    VL=(2*vel-omega*L)/(2*R); 

    //-----right motor------

    if (VR<0)
    {
       digitalWrite(direction_R, 0);
       analogWrite(pwm_R, abs(VR));
       //motorR.run(BACKWARD);
       //motorR.setSpeed(abs(VR));   
    }

    else 
    {
      digitalWrite(direction_R, 1);
      analogWrite(pwm_R, abs(VR));   
      //motorR.run(FORWARD);  
      //motorR.setSpeed(VR); 
        
    }

    //-----left motor------

     if (VL<0)
    {
       //motorL.run(BACKWARD);
       //motorL.setSpeed(abs(VL));   
       digitalWrite(direction_L, 0);
       analogWrite(pwm_L, abs(VL));
    }

    else 
    {
      //motorL.run(FORWARD);  
      //motorL.setSpeed(VL); 
      digitalWrite(direction_L, 1);
      analogWrite(pwm_L, abs(VL));  
    }



}

//--------------------subscribers---------------------------
ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", &motors_cb);



 void setup() 
{
      Serial.begin (57600);
      pinMode(encoderL_APos, INPUT);
      digitalWrite(encoderL_APos, HIGH);       // turn on pullup resistor
      pinMode(encoderL_BPos, INPUT);
      digitalWrite(encoderL_BPos, HIGH);
      
      pinMode(encoderR_APos, INPUT);
      digitalWrite(encoderR_APos, HIGH);      // turn on pullup resistor
      pinMode(encoderR_BPos, INPUT);
      digitalWrite(encoderR_BPos, HIGH);
      
      attachInterrupt(digitalPinToInterrupt(18), doEncoderL_A, RISING);  // encoDER ON PIN 2
      attachInterrupt(digitalPinToInterrupt(20), doEncoderR_A, RISING);  // encoDER ON PIN 20
      attachInterrupt(digitalPinToInterrupt(19), doEncoderL_B, RISING);  // encoDER ON PIN 2
      attachInterrupt(digitalPinToInterrupt(21), doEncoderR_B, RISING);  // encoDER ON PIN 20
      //Serial.begin (9600);
      Serial.println("start");        

      pinMode(pwm_R ,OUTPUT);
      pinMode(direction_R ,OUTPUT);
      pinMode(pwm_R,OUTPUT);
      pinMode(direction_L ,OUTPUT);
      //-----------------------------
       // turn on motor
      //motorR.setSpeed(200);
      //motorR.run(RELEASE);
      analogWrite(pwm_R, 200);
      analogWrite(pwm_R, 0);
      
      //motorL.setSpeed(200);
      //motorL.run(RELEASE);
      analogWrite(pwm_L, 200);
      analogWrite(pwm_L, 0);

      

     //---------------------------ROS Setup
      nh.advertise(enc_pub);  
      nh.subscribe(sub);      
      nh.advertise (mineMessage);
      }


 void loop() { 

    // Serial.print ("Right velocity = ");
    // Serial.println(abs(VR)); 
   //Right Encoder
     //long RnewPosition = R_enc.read();
     R_AnewPosition = encoderR_APos;
     if (R_AnewPosition != R_AoldPosition) {
          R_AoldPosition = R_AnewPosition;   
          //Serial.print ("Right Encder = ");
          //Serial.println(RnewPosition);
        }
     
     R_BnewPosition = encoderR_BPos;
     if (R_BnewPosition != R_BoldPosition) {
          R_BoldPosition = R_BnewPosition; 
     }        
        
  //----left encoder
  //long LnewPosition = L_enc.read();
  L_AnewPosition = encoderL_APos;
  if (L_AnewPosition != L_AoldPosition) {
      L_AoldPosition = L_AnewPosition; //update positions
       //Serial.print ("left Encoder = ");
      //Serial.println(LnewPosition);
      }  
  L_BnewPosition = encoderL_BPos;    
  if (L_BnewPosition != L_BoldPosition) {
      L_BoldPosition = L_BnewPosition; //update positions
  }   
//-------end of encoder

//-----------------------ROS publishing  
        LnewPosition =(R_AnewPosition) ;  //+ R_BnewPosition) /2
        RnewPosition =(L_AnewPosition + L_BnewPosition) /2;
        Point_msg.x= RnewPosition;
        Point_msg.y= LnewPosition;
        enc_pub.publish(&Point_msg);
        
        mineMessage.publish( &_msg );
        printDebug();
//-------------        
      nh.spinOnce(); 
      delay(10);
 }




 
