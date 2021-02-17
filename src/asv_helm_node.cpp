// Roland Arsenault
// Center for Coastal and Ocean Mapping
// University of New Hampshire
// Copyright 2017, All rights reserved.

#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Twist.h"
#include "geographic_msgs/GeoPointStamped.h"
#include "marine_msgs/NavEulerStamped.h"
#include "marine_msgs/Heartbeat.h"
#include "marine_msgs/Contact.h"
#include "marine_msgs/Helm.h"
//#include "project11/mutex_protected_bag_writer.h"
#include <regex>
#include "boost/date_time/posix_time/posix_time.hpp"
#include "std_msgs/Int32.h"

ros::Publisher asv_helm_pub;
ros::Publisher asv_inhibit_pub;
ros::Publisher position_pub;
ros::Publisher heading_pub;
ros::Publisher speed_pub;
ros::Publisher speed_modulation_pub;
ros::Publisher heartbeat_pub;
ros::Publisher contact_pub;


double heading;
double rudder;
double throttle;
ros::Time last_time;

double last_boat_heading;

double desired_speed;
ros::Time desired_speed_time;
double desired_heading;
ros::Time desired_heading_time;

float obstacle_distance;
float speed_modulation;

std::string piloting_mode;

void twistCallback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    throttle = msg->twist.linear.x/2.75;
    rudder = -msg->twist.angular.z;
    
    last_time = msg->header.stamp;
}

void helmCallback(const marine_msgs::Helm::ConstPtr& msg)
{
    throttle = msg->throttle;
    rudder = msg->rudder;
    
    last_time = msg->header.stamp;
}

void headingCallback(const marine_msgs::NavEulerStamped::ConstPtr& msg)
{
    last_boat_heading = msg->orientation.heading;
}


void obstacleDistanceCallback(const std_msgs::Float32::ConstPtr& inmsg)
{
    float stop_distance = 25.0;
    float start_slowing_down_distance = 50.0;

    obstacle_distance = inmsg->data;
    if(obstacle_distance < 0 || obstacle_distance > start_slowing_down_distance)
        speed_modulation = 1.0;
    else if (obstacle_distance < stop_distance)
        speed_modulation = 0.0;
    else
        speed_modulation = (obstacle_distance-stop_distance)/(start_slowing_down_distance-stop_distance);
    std_msgs::Float32 sm;
    sm.data = speed_modulation;
    speed_modulation_pub.publish(sm);
//     if(ros::Time::now() > ros::TIME_MIN)
//         log_bag.write("/speed_modulation",ros::Time::now(),sm);

}

void desiredSpeedCallback(const geometry_msgs::TwistStamped::ConstPtr& inmsg)
{
    desired_speed = inmsg->twist.linear.x;
    desired_speed_time = inmsg->header.stamp;
}

void desiredHeadingCallback(const marine_msgs::NavEulerStamped::ConstPtr& inmsg)
{
    desired_heading = inmsg->orientation.heading;
    desired_heading_time = inmsg->header.stamp;
}

void helmModeCallback(const std_msgs::String::ConstPtr& inmsg)
{
    piloting_mode = inmsg->data;
}

std::string boolToString(bool value)
{
    if(value)
        return "true";
    return "false";
}


void vehicleSatusCallback(const ros::TimerEvent event)
{
    //std::cerr << "status callback: " << " active: " << active << std::endl;
    if(piloting_mode != "standby")
    {
        bool doDesired = true;
        //std::cerr << "last_real: " << event.last_real << " last_time: " << last_time << std::endl;
        if (!last_time.isZero())
        {
            if(event.last_real-last_time>ros::Duration(.5))
            {
                throttle = 0.0;
                rudder = 0.0;
            }
            else
                doDesired = false;
        }
        if(doDesired)
        {
            if (event.current_real - desired_heading_time < ros::Duration(.5) && event.current_real - desired_speed_time < ros::Duration(.5))
            {
                //std::cerr << "desired: " << desired_heading << "\tlast boat heading: " << last_boat_heading << std::endl;
                double steering_angle = (desired_heading-last_boat_heading)*M_PI/180.0;
                if(steering_angle > M_PI)
                    steering_angle -= 2.0*M_PI;
                if(steering_angle < -M_PI)
                    steering_angle += 2.0*M_PI;

                //std::cerr << "steering_angle: " << steering_angle << std::endl;
                geometry_msgs::Twist t;
                t.linear.x = desired_speed;
                t.angular.z = -steering_angle;
                asv_helm_pub.publish(t);
            }
        }
        else
        {
            geometry_msgs::Twist t;
            t.linear.x = throttle*2.75;
            t.angular.z = -rudder;
            asv_helm_pub.publish(t);
        }
    }
    
    marine_msgs::Heartbeat hb;
    hb.header.stamp = ros::Time::now();

    marine_msgs::KeyValue kv;

    kv.key = "piloting_mode";
    kv.value = piloting_mode;
    hb.values.push_back(kv);
    
    heartbeat_pub.publish(hb);
}



int main(int argc, char **argv)
{
    heading = 0.0;
    throttle = 0.0;
    rudder = 0.0;
    last_boat_heading = 0.0;
    obstacle_distance = -1.0;
    speed_modulation = 1.0;
    piloting_mode = "standby";
    
    ros::init(argc, argv, "asv_helm");
    ros::NodeHandle n;

    asv_helm_pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1);
    speed_modulation_pub = n.advertise<std_msgs::Float32>("speed_modulation",1);
    heartbeat_pub = n.advertise<marine_msgs::Heartbeat>("project11/heartbeat", 10);

    ros::Subscriber asv_helm_sub = n.subscribe("/remote/0/cmd_vel",5,twistCallback);
    ros::Subscriber piloting_mode_sub = n.subscribe("project11/piloting_mode",10,helmModeCallback);
    ros::Subscriber dspeed_sub = n.subscribe("project11/desired_speed",10,desiredSpeedCallback);
    ros::Subscriber dheading_sub = n.subscribe("project11/desired_heading",10,desiredHeadingCallback);
    ros::Subscriber obstacle_distance_sub =  n.subscribe("obstacle_distance",10,obstacleDistanceCallback);
    ros::Subscriber heading_sub = n.subscribe("heading",10,headingCallback);
    ros::Subscriber helm_sub = n.subscribe("project11/helm",10,helmCallback);
    
    ros::Timer timer = n.createTimer(ros::Duration(0.2),vehicleSatusCallback);
    
    ros::spin();
    
    //log_bag.close();
    
    return 0;
}
