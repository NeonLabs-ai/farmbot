#include "ros/ros.h"
#include "farm_msgs/GlobalState.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Imu.h"

// include std lib
#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <algorithm>

class Localizer
{
    public:

    farm_msgs::GlobalState state;
    sensor_msgs::NavSatFix gps;
    sensor_msgs::Imu imu;

    ros::Subscriber sub;
    ros::Publisher pub;

    Localizer(ros::NodeHandle nh){
        sub = nh.subscribe<sensor_msgs::NavSatFix>("/rtk_gps", 1, &Localizer::gpsCallback, this);
        sub = nh.subscribe<sensor_msgs::Imu>("/imu", 1, &Localizer::imuCallback, this);
        pub = nh.advertise<farm_msgs::GlobalState>("/global_state", 1);
    }
    
    void gpsCallback(const sensor_msgs::NavSatFix msg){
        gps = msg;
    }  

    void imuCallback(const sensor_msgs::Imu msg){
        imu = msg;
    }     

    void localizer(){

        // Estimate state using imu and gps by sensor fusion 

        // Implement Extended kalman filter considering uncertainity of both sensors

        // Publish State

    }

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "localizer");
    ros::NodeHandle nh;
    Localizer localizer(nh);
    ros::spin();
    return 0;
}