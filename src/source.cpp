// Include the header file for Trigger.
#include "std_srvs/Trigger.h"
#include "ros/ros.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "std_msgs/String.h"

void recieveOrder(std::vector<trajectory_msgs::JointTrajectory> trajectory_vector)
{
  ROS_INFO("I heard: [%s]", trajectory_vector);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ariac_challenge");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("ariac/arm/command", 1000, recieveOrder);

  // To declare the variable in this way where necessary in the code. 
  std_srvs::Trigger begin_comp;
  // Create the service client.
  ros::ServiceClient begin_client = n.serviceClient<std_srvs::Trigger>("ariac_challenge_service");
  // Call the Service
  begin_client.call(begin_comp);
  // Sample output
  ROS_WARN("Competition service returned failure: %s",begin_comp.response.message.c_str());

  std::vector<trajectory_msgs::JointTrajectory> trajectory_vector;
  trajectory_vector.clear();

  ros::spin();

  return 0;
}

