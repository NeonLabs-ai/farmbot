#include "weed_planner.h"

class WeedPlanner {
    
public:
    float threshold = 20.0; // Should be taken from param server
    ros::Publisher pub;
    ros::Subscriber sub;

    WeedPlanner(ros::NodeHandle nh) {
        sub = nh.subscribe<farm_msgs::WeedList>("/weed_planning_list", 1,&WeedPlanner::planCB, this);
        pub = nh.advertise<farm_msgs::NozzlePositionList>("/nozzle_trajectory", 1);
    }

    void planCB(farm_msgs::WeedList msg)
    {
        farm_msgs::WeedList planlist = msg;
        plan_and_reach_weeds(planlist); 
    }

    void plan_and_reach_weeds(const farm_msgs::WeedList planlist)
    {
        farm_msgs::NozzlePositionList spray_list;

        for(auto weed: planlist.weeds){
            
            farm_msgs::NozzleTrajectory nozzle_position;

            if(weed.bbox[1] > threshold){
                nozzle_position.position = weed.bbox[0];                // Note: Nozzle position has 1 DOF: in X axis
                spray_list.nozzle_pose.push_back(nozzle_position);
            }
        }

        pub.publish(spray_list);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lane_detector");
    ros::NodeHandle nh;
    WeedPlanner ic(ros::NodeHandle nh);
    ros::spin();
    return 0;
}