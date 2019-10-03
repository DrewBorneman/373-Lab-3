// Include the header file for Trigger.
#include "std_srvs/Trigger.h"
#include "ros/ros.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "osrf_gear/Order.h"
#include "osrf_gear/GetMaterialLocations.h"
#include "osrf_gear/LogicalCameraImage.h"
#include "osrf_gear/Model.h"
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

	//create variables
	geometry_msgs::PoseStamped current_pose, end_pose;

void recieveOrder(const osrf_gear::Order::ConstPtr & order)
{

  ROS_INFO("Order recieved:%s\n", *order);
  order_vector.push_back(*order);
}

void logicalCameraCallback(const osrf_gear::LogicalCameraImage::ConstPtr & imageMsg)
{ 
  for(int i=0;i<imageMsg->models.size();i++){
    ROS_INFO("Model: %s\n",imageMsg->models[i]);
		if(imageMsg->models[i].type == ObjectType)
			current_pose.pose = imageMsg->models[i].pose;
  }
}

void startCompetition(ros::NodeHandle & n)
{

  // To declare the variable in this way where necessary in the code. 
  std_srvs::Trigger begin_comp;
  // Create the service client.
  ros::ServiceClient begin_client = n.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
  // Call the Service
  begin_client.call(begin_comp);
 if (!begin_comp.response.success) {  // If not successful, print out why.
    ROS_ERROR("Competition service returned failure: %s",begin_comp.response.message.c_str());
  } else {
    ROS_INFO("Competition started!");
  } 

}

tf2_ros::TransformListener tfListener(tfBuffer);

int main(int argc, char **argv)
{
	ros::AsyncSpinner spinner(2);
	spinner.start();
  ros::init(argc, argv, "ariac_challenge");
  ros::NodeHandle n;

  ros::Subscriber orderSub = n.subscribe("ariac/orders", 1000, recieveOrder);
  ros::Subscriber cameraSub = n.subscribe("ariac/logial_camera", 1000, logicalCameraCallback);
	geometry_msgs::TransformStamped tfStamped;
	moveit::planning_interface::MoveItErrorCode planningError;
  order_vector.clear();

  startCompetition(n);
	while(ros::ok()){
		ros::spinOnce();
		if(order_vector.size() > 0){
			//Retrieve the transformation

			try {
				tfStamped = tfBuffer.lookupTransform(move_group.getPlanningFrame().c_str(), "logical_camera_frame", ros::Time(0.0), ros::Duration(1.0));
				ROS_DEBUG("Transform to [%s] from [%s]", tfStamped.header.frame_id.c_str(), 		tfStamped.child_frame_id.c_str());
				}
			catch (tf2::TransformException &ex) {
				ROS_ERROR("%s", ex.what());
			}

			end_pose.pose.position.z += 0.10;
			end_pose.pose.orientation.w = 0.707;
			end_pose.pose.orientation.x = 0.0;
			end_pose.pose.orientation.y = 0.707;
			end_pose.pose.orientation.z = 0.0;

			tf2::doTransform(current_pose, end_pose, tfStamped);

			// Set the desired pose for the arm in the arm controller.
			move_group.setPoseTarget(end_pose);
			// Instantiate and create a plan.
			moveit::planning_interface::MoveGroupInterface::Plan movePlan;
			// Create a plan based on the settings (all default settings now) in the_plan.
			planningError = move_group.plan(movePlan);
			// Planning does not always succeed. Check the output.
			// In the event that the plan was created, execute it.
			if(planningError == moveit_msgs::MoveItErrorCodes::SUCCESS)
				move_group.execute(movePlan);
		}
	}
  return 0;
}

