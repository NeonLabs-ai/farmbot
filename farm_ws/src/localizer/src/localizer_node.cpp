#include "localizer.h"

class Localizer
{
    public:

    farm_msgs::GlobalState sensor_data;
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

        sensor_data.x = gps.latitude;
        sensor_data.y = gps.longitude;
        sensor_data.z = gps.altitude;
        sensor_data.roll = imu.orientation.x;
        sensor_data.pitch = imu.orientation.y;
        sensor_data.yaw = imu.orientation.z;
        sensor_data.vx = imu.linear_acceleration.x;
        sensor_data.vy = imu.linear_acceleration.y;
        sensor_data.vz = imu.linear_acceleration.z;
        sensor_data.ax = imu.angular_velocity.x;
        sensor_data.ay = imu.angular_velocity.y;
        sensor_data.az = imu.angular_velocity.z;

        // Estimate state using imu and gps by sensor fusion 
        // Implement Extended kalman filter considering uncertainity of both sensors in mobile robot
        
        // x_ : state vector
        
        vector<float> x_(12) ;
                

        // Publish state estimate

        state.x = x_[0];
        state.y = x_[1];
        state.z = x_[2];
        state.roll = x_[3];
        state.pitch = x_[4];
        state.yaw = x_[5];
        state.vx = x_[6];
        state.vy = x_[7];
        state.vz = x_[8];
        state.ax = x_[9];
        state.ay = x_[10];
        state.az = x_[11];

        pub.publish(state);
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