

#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int16.h>






/*create a node handler*/
ros::NodeHandle  nh;


/*create two messages of type int16 to hold encoder values*/
std_msgs::Int16 EncR;
std_msgs::Int16 EncL;


double pulses_per_rotation = 600.0; // no. of pulses generated by the encoder per revolution
volatile int encR_value = 0; // no. of pulses right
volatile int encL_value = 0; // no. of pulses left

volatile int encR_value_odom = 0; // no. of pulses right
volatile int encL_value_odom = 0;
long previousMillis = 0;
long currentMillis = 0;
double rpmR;
double rpmL;



double vR;
double vL;
double R;
double X;
double Z;

double actual_speed=0;

double actual_Z=0;

double l = 400.0;   //mm
double r = 100.0;    //mm


double Ts;         // sampling time

double KiR,KpR,KdR;   // PID parameters
double eR=0;        // error
double last_eR=0;   //previous error
double deR=0;       // the change in error
double set_pointR_c;  // required speed in RPM
double set_pointR;
double inputR;      // current speed i RPM
double outputR;     // control action (pwm signal given to the motor driver)
double inte_R=0;  //integrator term
double diff_R=0;  //differentaitor term

double KiL,KpL,KdL;   // PID parameters
double eL=0;        // error
double last_eL=0;   //previous error
double deL=0;       // the change in error
double set_pointL_c;  // required speed in RPM
double set_pointL;
double inputL;      // current speed i RPM
double outputL;     // control action (pwm signal given to the motor driver)
double inte_L=0;  //integrator term
double diff_L=0;  //differentaitor term




#define AR 3
#define BR 2
#define AL 18
#define BL 19 
#define DIRR 4
#define DIRL 5
#define PWMR 10
#define PWML 11




/*call back function for subscriber*/
void GetVelocity( const geometry_msgs::Twist& s_msg)
{
  X=s_msg.linear.x;
  Z=s_msg.angular.z; 
}

/*create two publishers to publish the encoder values*/
ros::Publisher RightEncoder("/rwheel", &EncR);
ros::Publisher LeftEncoder("/lwheel", &EncL);

/*create a ros subscriber to get the required velocity*/
ros::Subscriber<geometry_msgs::Twist> VelocitySub("/cmd_vel", &GetVelocity );


void setup() 
{
  
  /*initialize a node*/
  nh.initNode();

  /*start publisher called RightEncoder*/
  nh.advertise(RightEncoder);

  /*start publisher called LeftEncoder*/
  nh.advertise(LeftEncoder);

  /*start subscriber called VelocitySub*/
  nh.subscribe(VelocitySub);

  /*confighure the encoder pins*/
  pinMode(AR,INPUT_PULLUP);
  pinMode(BR,INPUT_PULLUP);
  pinMode(AL,INPUT_PULLUP);
  pinMode(BL,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(AR),updateEncoderR,RISING);
  attachInterrupt(digitalPinToInterrupt(AL),updateEncoderL,RISING);

  /*configure the motor pins*/
  pinMode(DIRR,OUTPUT);
  pinMode(DIRL,OUTPUT);
  pinMode(PWMR,OUTPUT);
  pinMode(PWML,OUTPUT);
  

  /*set the PID controller parameters*/
  Ts = 50.0;       //setting the sampling time
  KpR = 5.0;         //setting the propotional gain value
  KdR = 10.0;        //setting the diffrential gain value
  KiR = 0.001;     //setting the intefral gain value
  KpL = 5.0;         //setting the propotional gain value
  KdL = 10.0;        //setting the diffrential gain value
  KiL = 0.001;     //setting the intefral gain value
  

  previousMillis = millis();
}

void loop()
{
   
  /*calculate the set points for the PID controllers*/
  currentMillis = millis();
  if(currentMillis - previousMillis > Ts)
  {
    previousMillis = currentMillis;
    if(X!=0.0)
  {
    set_pointR_c = ((X*1000.0*60)/(r*6.283));
    set_pointL_c = ((X*1000.0*60)/(r*6.283));
  }
  else if (Z!=0.0)
  {
    set_pointR_c = (Z*60*l)/(r*6.283);
    set_pointL_c = -(Z*l*60)/(r*6.283);
  }
  else
  {
    set_pointR_c = 0;
    set_pointL_c = 0;
  }

    clac_speedR();
    clac_speedL();
    pidL();
    pidR();
    Move();


  /*publish the left and right encoder values*/
  RightEncoder.publish( &EncR);
  LeftEncoder.publish( &EncL);

  
  nh.spinOnce();
    
  }
  //Serial.print("SPEED"); Serial.print("     "); Serial.print(rpmR);  Serial.print("     ");   Serial.println(rpmL);
  //Serial.print("setpoint");Serial.print("     ");  Serial.print(set_pointR_c);  Serial.print("     ");   Serial.println(set_pointL_c);
  
}

void updateEncoderR()
{
  if(digitalRead(BR) > 0)
  {
    encR_value++;
    encR_value_odom++; 
  }

  else
  {
    encR_value--;
    encR_value_odom--;
  } 

  EncR.data = encR_value_odom;
}


void updateEncoderL()
{
  if(digitalRead(BL) > 0)
  {
    encL_value++;
    encL_value_odom++;  
  }

  else
  {
    encL_value--;
    encL_value_odom--;
  } 

  EncL.data = encL_value_odom;
}

void clac_speedR()
{
  rpmR =  ((float) (encR_value*60.0*1000.0) / (pulses_per_rotation*Ts));
  inputR = abs(rpmR);
  encR_value =0;

  vR=(rpmR*2*3.14*r)/(60.0*1000);
}

void clac_speedL()
{
  rpmL =  ((float) (encL_value*60.0*1000.0) / (pulses_per_rotation*Ts));
  inputL = abs(rpmL);
  encL_value =0;
   vL=(rpmL*2*3.14*r)/(60.0*1000);
}

void pidR() // pid output calculation
{
  if(set_pointR_c<0)
  {
    digitalWrite(DIRR,HIGH);
  }
  else
  {
    digitalWrite(DIRR,LOW);
  }
   set_pointR = abs(set_pointR_c);
   eR = set_pointR - inputR;   // calculate error
   deR = eR - last_eR;         // the change in error
   diff_R = deR/Ts;           // the differentiator term
   inte_R = inte_R + eR*Ts;    //integrator term
   outputR = (KpR*eR) + (KdR*diff_R) + (KiR*inte_R); // PID controller term
   outputR = abs(outputR);    //controller action
   if(outputR>255)           // max output is 255 for arduino (throgh 8_bit register ( 0 to 255 values)
   {
    outputR = 255;
   }
   if(eR<0)
   {
    outputR = outputR - (outputR*0.1) ;
   }
   last_eR = eR;

  if(set_pointR_c==0)
  {
    outputR = 0;
  }
}



void pidL() // pid output calculation
{
  if((set_pointL_c)<0)
  {
    digitalWrite(DIRL,HIGH);
  }
  else
  {
    digitalWrite(DIRL,LOW);
  }
   set_pointL = abs(set_pointL_c);
   eL = set_pointL - inputL;   // calculate error
   deL = eL - last_eL;         // the change in error
   diff_L = deL/Ts;           // the differentiator term
   inte_L = inte_L + eL*Ts;    //integrator term
   outputL = (KpL*eL) + (KdL*diff_L) + (KiL*inte_L); // PID controller term
   outputL = abs(outputL);    //controller action
   if(outputL>255)           // max output is 255 for arduino (throgh 8_bit register ( 0 to 255 values)
   {
    outputL = 255;
   }
   if(eL<0)
   {
    outputL = outputL - (outputL*0.1) ;
   }
   last_eL = eL;

  if(set_pointL_c==0)
  {
    outputL = 0;
  }
}

void Move()
{
  analogWrite(PWMR,outputR);
  analogWrite(PWML,outputL);
}
