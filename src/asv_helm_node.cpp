// Roland Arsenault
// Center for Coastal and Ocean Mapping
// University of New Hampshire
// Copyright 2017, All rights reserved.

#include "ros/ros.h"
#include "asv_msgs/HeadingHold.h"
#include "geometry_msgs/TwistStamped.h"

ros::Publisher asv_helm_pub;

double heading;
ros::Time last_time;

void twistCallback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    asv_msgs::HeadingHold asvMsg;
    if (!last_time.isZero())
    {
        ros::Duration delta_t = msg->header.stamp - last_time;
        heading -= msg->twist.angular.z*delta_t.toSec();
        heading = fmod(heading,M_PI*2.0);
        if(heading < 0.0)
            heading += M_PI*2.0;
    }
    asvMsg.heading.heading = heading;
    asvMsg.thrust.type = asv_msgs::Thrust::THRUST_THROTTLE;
    asvMsg.thrust.value = msg->twist.linear.x;
    asvMsg.header.stamp = msg->header.stamp;
    asv_helm_pub.publish(asvMsg);
    last_time = msg->header.stamp;
}

int main(int argc, char **argv)
{
    heading = 0.0;
    ros::init(argc, argv, "asv_helm");
    ros::NodeHandle n;
    
    asv_helm_pub = n.advertise<asv_msgs::HeadingHold>("/control/drive/heading_hold",1);
    heading = 0.0;

    ros::Subscriber asv_helm_sub = n.subscribe("/cmd_vel",5,twistCallback);
    
    ros::spin();
    
    return 0;
    
}
