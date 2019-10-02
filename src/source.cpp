// Include the header file for Trigger.
#include "std_srvs/Trigger.h"
#include "ros/ros.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "osrf_gear/Order.h"
#include "osrf_gear/LogicalCameraImage.h"
#include "std_msgs/String.h"
#include <vector>

// MoveIt header files
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
//Transformation header files
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/TransformStamped.h"


std::vector<osrf_gear::Order> order_vector;
std::string ObjectType = "piston_rod_part";
tf2_ros::Buffer tfBuffer;
moveit::planning_interface::MoveGroupInterface move_group("manipulator");

void recieveOrder(const osrf_gear::Order::ConstPtr & order)
{

  ROS_INFO("Order recieved:\n" << *order);
  order_vector.push_back(*order);
}

void logicalCameraCallback(const osrf_gear::LogicalCameraImage::ConstPtr & imageMsg)
{ 
  for(int i=0;i<imageMsg->models.size();i++){
    ROS_INFO("Model: %s\n",imageMsg->models[i]);
  }
}

tf2_ros::TransformListener tfListener(tfBuffer);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ariac_challenge");
  ros::NodeHandle n;

  ros::Subscriber orderSub = n.subscribe("ariac/orders", 1000, recieveOrder);
  ros::Subscriber cameraSub = n.subscribe("ariac/logial_camera", 1000, logicalCameraCallback);
	
  order_vector.clear();

  // To declare the variable in this way where necessary in the code. 
  std_srvs::Trigger begin_comp;
  // Create the service client.
  ros::ServiceClient begin_client = n.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
  // Call the Service
  begin_client.call(begin_comp);
  // Sample output
  ROS_WARN("Competition service returned failure: %s",begin_comp.response.message.c_str());

//Retrieve the transformation
geometry_msgs::TransformStamped tfStamped;
try {
  tfStamped = tfBuffer.lookupTransform(move_group.getPlanningFrame().c_str(), "logical_camera_frame", ros::Time(0.0), ros::Duration(1.0));
  ROS_DEBUG("Transform to [%s] from [%s]", tfStamped.header.frame_id.c_str(), tfStamped.child_frame_id.c_str());
  }
catch (tf2::TransformException &ex) {
  ROS_ERROR("%s", ex.what());
  }
// tf2_ross::Buffer.lookupTransform("to_frame", "from_frame", "how_recent", "how_long_to_wait);

geometry_msgs::PoseStamped current_pose, end_pose;
//current_pose.pose = //...;  unsure what to do here 

end_pose.pose.orientation.z += 0.10;
end_pose.pose.orientation.w = 0.707;
end_pose.pose.orientation.x = 0.0;
end_pose.pose.orientation.y = 0.707;
end_pose.pose.orientation.z = 0.0;

tf2::doTransform(current_pose, end_pose, tfStamped);



  ros::spin();

  return 0;
}

