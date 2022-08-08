#include "ros/ros.h"
#include "farm_msgs/NozzlePosition.h"
#include "farm_msgs/WeedList.h"

// include std lib
#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <algorithm>

class SprayPlanner
{
    public:
    ros::Subscriber sub;
    ros::Publisher pub;

    SprayPlanner(ros::NodeHandle nh){
        sub = nh.subscribe<farm_msgs::WeedList>("/weed_detected", 1, &SprayPlanner::planCallback, this);
        pub = nh.advertise<farm_msgs::NozzlePosition>("/Nozzle_position", 1);
                
    }
    
    void planCallback(const farm_msgs::WeedList msg){
        
        farm_msgs::NozzlePosition position;


    }       
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "spray_planner");
    ros::NodeHandle nh;
    SprayPlanner spray_planner(nh);
    ros::spin();
    return 0;
}