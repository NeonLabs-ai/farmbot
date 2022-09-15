#include <ros.h>
#include <std_msgs/UInt16.h>

const int pwm = 10; 
const int in_1 = 2 ;
const int in_2 = 9 ;

const int pwm2 = 3; 
const int in_3 = 5 ;
const int in_4 = 6 ;

ros::NodeHandle  nh;


void vel_cb( const std_msgs::UInt16& cmd_msg){
    
}

ros::Subscriber<std_msgs::UInt16> sub("cmd_vel", vel_cb);

void setup(){
   pinMode(pwm,OUTPUT) ; //we have to set PWM pin as output
   pinMode(in_1,OUTPUT) ; //Logic pins are also set as output
   pinMode(in_2,OUTPUT) ;  
   pinMode(pwm2,OUTPUT) ; //we have to set PWM pin as output
   pinMode(in_3,OUTPUT) ; //Logic pins are also set as output
   pinMode(in_4,OUTPUT) ;  
   nh.initNode();
   nh.subscribe(sub);
}


void loop(){

   analogWrite(pwm,255) ;
   analogWrite(pwm2, 255);
   
//   //Clockwise for 3 secs
//   delay(3000) ;
//   
//   analogWrite(pwm,100) ;
//   delay(3000) ;
//
//   analogWrite(pwm,255) ;
//   delay(3000) ;
//
//   analogWrite(pwm,100) ;
//   delay(3000) ;
   nh.spinOnce();
   delay(1);
}
