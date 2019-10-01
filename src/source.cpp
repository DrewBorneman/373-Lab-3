// Include the header file for Trigger.
#include "std_srvs/Trigger.h"
#include "ros/ros.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "osrf_gear/Order.h"
#include "osrf_gear/LogicalCameraImage.h"
#include "std_msgs/String.h"
#include <vector>

std::vector<std::String> order_vector;
std::String ObjectType = "piston_rod_part"
void recieveOrder(const osrf_gear::Order::ConstPtr & order)
{

  ROS_INFO("Order recieved:\n", << *order);
  order_vector.push_back(*order);
}

void logicalCameraCallback(const osrf_gear::LogicalCameraImage::ConstPtr & imageMsg)
{
  for(int i=0;i<imageMsg->models.size();i++){
    ROS_INFO("Model: \n" << imageMsg->models[i]);
  }
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "ariac_challenge");
  order_vector.clear();
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("ariac/orders", 1000, recieveOrder);
  ros::Subscriber sub = n.subscribe("ariac/logial_camera", 1000, logicalCameraCallback);

  // To declare the variable in this way where necessary in the code. 
  //std_srvs::Trigger begin_comp;
  // Create the service client.
  //ros::ServiceClient begin_client = n.serviceClient<std_srvs::Trigger>("ariac_challenge_service");
  // Call the Service
  //begin_client.call(begin_comp);
  // Sample output
  ROS_WARN("Competition service returned failure: %s",begin_comp.response.message.c_str());

  ros::spin();

  return 0;
}

