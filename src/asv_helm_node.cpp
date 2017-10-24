// Roland Arsenault
// Center for Coastal and Ocean Mapping
// University of New Hampshire
// Copyright 2017, All rights reserved.

#include "ros/ros.h"
#include "asv_msgs/HeadingHold.h"
#include "geometry_msgs/Twist.h"

ros::Publisher asv_helm_pub;

double heading;

void twistCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    asv_msgs::HeadingHold asvMsg;
    // todo integrate rotation into heading
    asvMsg.heading.heading = heading;
    asvMsg.thrust.type = asv_msgs::Thrust::THRUST_THROTTLE;
    asvMsg.thrust.value = msg->linear.x;
    asv_helm_pub.publish(asvMsg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "asv_helm");
    ros::NodeHandle n;
    
    asv_helm_pub = n.advertise<asv_msgs::HeadingHold>("/control/drive/heading_hold",1);
    heading = 0.0;

    ros::Subscriber asv_helm_sub = n.subscribe("/cmd_vel",5,twistCallback);
    
    ros::spin();
    
    return 0;
    
}
