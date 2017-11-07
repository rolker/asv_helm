// Roland Arsenault
// Center for Coastal and Ocean Mapping
// University of New Hampshire
// Copyright 2017, All rights reserved.

#include "ros/ros.h"
#include "asv_msgs/HeadingHold.h"
#include "geometry_msgs/TwistStamped.h"

ros::Publisher asv_helm_pub;

double heading;
double rudder;
double throttle;
ros::Time last_time;

void twistCallback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    throttle = msg->twist.linear.x;
    rudder = -msg->twist.angular.z;
    
    last_time = msg->header.stamp;
}

void sendHeadingHold(const ros::TimerEvent event)
{
    asv_msgs::HeadingHold asvMsg;
    if (!last_time.isZero())
    {
        if(event.last_real-last_time>ros::Duration(.5))
        {
            throttle = 0.0;
            rudder = 0.0;
        }
    }

    ros::Duration delta_t = event.current_real-event.last_real;
    heading += rudder*delta_t.toSec();
    heading = fmod(heading,M_PI*2.0);
    if(heading < 0.0)
        heading += M_PI*2.0;
    
    asvMsg.heading.heading = heading;
    asvMsg.thrust.type = asv_msgs::Thrust::THRUST_THROTTLE;
    asvMsg.thrust.value = throttle;
    asvMsg.header.stamp = event.current_real;
    asv_helm_pub.publish(asvMsg);
}

int main(int argc, char **argv)
{
    heading = 0.0;
    throttle = 0.0;
    rudder = 0.0;
    
    ros::init(argc, argv, "asv_helm");
    ros::NodeHandle n;
    
    asv_helm_pub = n.advertise<asv_msgs::HeadingHold>("/control/drive/heading_hold",1);

    ros::Subscriber asv_helm_sub = n.subscribe("/cmd_vel",5,twistCallback);
    
    ros::Timer timer = n.createTimer(ros::Duration(0.1),sendHeadingHold);
    
    ros::spin();
    
    return 0;
    
}
