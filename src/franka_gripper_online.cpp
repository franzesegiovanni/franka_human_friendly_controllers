#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include <iostream>
#include <sstream>
#include <string>
#include <thread>

#include <franka/exception.h>
#include <franka/gripper.h>
#include <franka/gripper_state.h>
#include <franka_gripper/franka_gripper.h>

double width =0;
double width_old=1;
double flag =0;
double tolerance=0.0;
void chatterCallback(const std_msgs::Float32::ConstPtr& msg)
{
  width=msg->data;
  flag = 1;
  //ROS_INFO("I heard: [%s]", msg->data.c_str());
}
// %EndTag(CALLBACK)%

int main(int argc, char **argv)
{

  ros::init(argc, argv, "gripper");


  ros::NodeHandle n;
  ros::Rate loop_rate(30);

  ros::Subscriber sub = n.subscribe("gripper_online", 0, chatterCallback);
  ros::Publisher pub_move = n.advertise<franka_gripper::MoveActionGoal>("/franka_gripper/move/goal", 0);
  //ros::Publisher pub_stop = n.advertise<franka_gripper::StopAction>("/franka_gripper/stop", 1);
  ros::Publisher pub_grasp = n.advertise<franka_gripper::GraspActionGoal>("/franka_gripper/grasp/goal", 0);
  franka_gripper::MoveActionGoal msg_move;
  franka_gripper::GraspActionGoal msg_grasp;
  msg_move.goal.speed = 1;
  msg_grasp.goal.speed = 1;

  //franka_gripper::StopActionGoal msg_stop;
  while (ros::ok())
  {
   if(flag==1)
   {
     if(width<=(width_old+tolerance)) {
     msg_move.goal.width = width;
     pub_move.publish(msg_move);}
     else {
       msg_move.goal.width = width;
       pub_move.publish(msg_move);
       msg_grasp.goal.width = width;
       pub_grasp.publish(msg_grasp);}
     width_old=width;
     flag = 0;
    }

    ros::spinOnce();

    loop_rate.sleep();
  }





  //ros::spin();


  return 0;
}
