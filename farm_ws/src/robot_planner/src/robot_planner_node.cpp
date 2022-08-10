#include "ros/ros.h"
#include "farm_msgs/GlobalState.h"
#include "farm_msgs/Lane.h"
#include "farm_msgs/NozzlePosition.h"
#include "farm_msgs/Trajectory.h"

// include std lib
#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <algorithm>

class RobotPlanner
{
    public:

    farm_msgs::GlobalState state;
    farm_msgs::Lane lane;
    farm_msgs::Trajectory traj;

    ros::Subscriber sub;
    ros::Publisher pub;

    RobotPlanner(ros::NodeHandle nh){
        sub = nh.subscribe<farm_msgs::GlobalState>("/global_state", 1, &RobotPlanner::stateCallback, this);
        sub = nh.subscribe<farm_msgs::Lane>("/lane_detected", 1, &RobotPlanner::laneCallback, this);
        pub = nh.advertise<farm_msgs::Trajectory>("/robot_trajectory", 1);
    }
    
    void stateCallback(const farm_msgs::GlobalState msg){
        state = msg;
    }  

    void laneCallback(const farm_msgs::Lane msg){
        lane = msg;
    }     

    void planner(){

        // calculate best trajectory for navigation using lane and state and goal position
        

        // implement trajectory planner

        // publish trajectory


    }

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_planner");
    ros::NodeHandle nh;
    RobotPlanner robot_planner(nh);
    ros::spin();
    return 0;
}