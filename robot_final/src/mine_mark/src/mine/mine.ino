#include <ros.h>
#include <geometry_msgs/Twist.h>
ros::NodeHandle  nh;
geometry_msgs::Twist _msg;
ros::Publisher mineMessage("mineMessage", &_msg);

void setup()
{
  nh.initNode();
  nh.advertise (mineMessage);
  pinMode(2,INPUT);    //metal_detector
  pinMode(5,INPUT);     //proximity
  pinMode(7,OUTPUT);   //Relay
}

void loop(){
int  Prox=digitalRead(4);
int metal_detector= digitalRead(2);

if (metal_detector== 1 && Prox==0)
{ 
   _msg.linear.x = 2; // Surface Mine
   mineMessage.publish( &_msg );
   digitalWrite(7,LOW);
   delay(30);
    }
     
else if (metal_detector==1  && Prox==1) 
{  
  _msg.linear.x = 1; // Buried Mine 
  mineMessage.publish( &_msg );
  digitalWrite(7,LOW);
  delay(30);
}
else
{
  _msg.linear.x = 0; // No Mine
  mineMessage.publish( &_msg );
  digitalWrite(7,HIGH);
  delay(30); 
  }
    
   nh.spinOnce();
}
