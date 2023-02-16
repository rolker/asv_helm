// Roland Arsenault
// Center for Coastal and Ocean Mapping
// University of New Hampshire
// Copyright 2017, All rights reserved.

#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include "project11_msgs/Helm.h"
#include "project11_msgs/Heartbeat.h"

ros::Publisher throttle_pub;
ros::Publisher rudder_pub;
ros::Publisher status_pub;

void helmCallback(const project11_msgs::Helm::ConstPtr& msg)
{
  std_msgs::Float32 throttle_msg;
  throttle_msg.data = msg->throttle;
  throttle_pub.publish(throttle_msg);
  std_msgs::Float32 rudder_msg;
  rudder_msg.data = msg->rudder;
  rudder_pub.publish(rudder_msg);
}

void haveCommandsCallback(const std_msgs::Bool::ConstPtr& msg)
{
  project11_msgs::Heartbeat hb;
  hb.header.stamp = ros::Time::now();
  project11_msgs::KeyValue kv;
  kv.key = "sim";
  kv.value = "asv_sim";
  hb.values.push_back(kv);
  kv.key = "have_commands";
  if(msg->data)
    kv.value = "true";
  else
    kv.value = "false";
  hb.values.push_back(kv);
  status_pub.publish(hb);
}

void gazeboStatusCallback(const ros::TimerEvent& event)
{
  project11_msgs::Heartbeat hb;
  hb.header.stamp = event.current_real;
  project11_msgs::KeyValue kv;
  kv.key = "sim";
  kv.value = "gazebo";
  hb.values.push_back(kv);
  status_pub.publish(hb);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "asv_helm");
  ros::NodeHandle n;

  throttle_pub = n.advertise<std_msgs::Float32>("throttle",1);
  rudder_pub = n.advertise<std_msgs::Float32>("rudder",1);
  status_pub = n.advertise<project11_msgs::Heartbeat>("project11/status/helm",1);

  ros::Subscriber helm_sub = n.subscribe("helm",10,helmCallback);
  
  std::string sim_type;
  ros::param::param<std::string>("~simType", sim_type, "asv_sim");
  
  ros::Subscriber have_commands_sub;
  if(sim_type == "asv_sim")
    have_commands_sub = n.subscribe("have_commands", 10, haveCommandsCallback);
  
  ros::Timer gazebo_status_timer;
  if(sim_type == "gazebo")
  {
    gazebo_status_timer = n.createTimer(ros::Duration(0.2), gazeboStatusCallback);
  }
  
  ros::spin();
  
  return 0;
}
