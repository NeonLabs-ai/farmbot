// Standard libs
#include <ros/ros.h>
#include <math.h>
#include <queue>

// Msg types
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Point.h"

using namespace std;

class sprayer {

    public:

    ros::Subscriber joint_sub;
    ros::Subscriber bbox_sub;
    ros::Subscriber sprayer_pos_sub;
    ros::Publisher sprayer_pos_pub;

    sensor_msgs::JointState joints;

    float vel = 0;
    geometry_msgs::Point bbox;
    geometry_msgs::Point mid;
    
    bool use_sprayer = false;
    float robot_len = 1.5;
    float time = 500.0;
    geometry_msgs::Point sprayer_state;

    void jointCB(const sensor_msgs::JointState::ConstPtr &msg){

        // For getting robot velocity 
        
        joints = *msg;
        vel = joints.velocity[0];
    }
    
    void bboxCB(const geometry_msgs::Point::ConstPtr &msg){
        bbox = *msg;

        mid.x = bbox.x;
        mid.y = bbox.y + robot_len;     // Distance from sprayer
        mid.z = bbox.z;
 
        sprayer_pos_pub.publish(mid);

    }

    void sprayerCB(const geometry_msgs::Point::ConstPtr &msg){
        
        this->sprayer_state = *msg;

        

    }

    

    sprayer (ros::NodeHandle &nh){
        joint_sub = nh.subscribe<sensor_msgs::JointState>("/joint_states", 1, &sprayer::jointCB, this);
        bbox_sub = nh.subscribe<geometry_msgs::Point>("/detections", 1, &sprayer::bboxCB, this);
        sprayer_pos_sub = nh.subscribe<geometry_msgs::Point>("/sprayer_state", 1, &sprayer::sprayerCB, this);
        sprayer_pos_pub = nh.advertise<geometry_msgs::Point> ("/desired_sprayer_position", 1);
    }  
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sprayer");
    ros::NodeHandle nh;
    sprayer object_detector = sprayer(nh);
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