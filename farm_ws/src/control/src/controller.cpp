// Standard libs
#include <ros/ros.h>

// Msg types
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/JointState.h"

using namespace std;

class controller {

    public:

    ros::Subscriber joint_sub;
    ros::Publisher pub;

    geometry_msgs::Twist data;
    sensor_msgs::JointState joints;

    void jointCB(const sensor_msgs::JointState::ConstPtr &msg){

        // Calculate Twist from Joint states
        
        this->joints = *msg;

        data.linear.x = 5.0;
        data.linear.y = 0.0;
        data.linear.z = 0.0;

        data.angular.x = 0.0;
        data.angular.y = 0.0;
        data.angular.z = 0.5;

        pub.publish(data);
    
    }

    controller (ros::NodeHandle &nh){
        joint_sub = nh.subscribe<sensor_msgs::JointState>("/joint_states", 1, &controller::jointCB, this);
        pub = nh.advertise<geometry_msgs::Twist> ("/cmd_vel", 1);
    }  
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "controller");
    ros::NodeHandle nh;
    controller object_detector = controller(nh);
    ros::spin();
}


//////////// JOINT STATE

// std_msgs/Header header
//   uint32 seq
//   time stamp
//   string frame_id
// string[] name
// float64[] position
// float64[] velocity
// float64[] effort

//////////// TWIST

// geometry_msgs/Vector3 linear
//   float64 x
//   float64 y
//   float64 z
// geometry_msgs/Vector3 angular
//   float64 x
//   float64 y
//   float64 z