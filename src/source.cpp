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
std::string CompetitionState;

	//create variables
	geometry_msgs::PoseStamped current_pose, end_pose;

void recieveOrder(const osrf_gear::Order::ConstPtr & order)
{

  ROS_INFO("Order recieved:%s\n", *order);
  order_vector.push_back(*order);
}

void logicalCameraCallback(const osrf_gear::LogicalCameraImage::ConstPtr & imageMsg)
{ 
    ROS_INFO("Model: %s\n",imageMsg->models[1]);
		if(imageMsg->models[1].type == ObjectType)//just work on the first item
			current_pose.pose.position = imageMsg->models[1].pose.position; // i think we need to add /ariac/{name} because that's the logical camera's output. I'm just not positive where to put it
}

  /// Called when a new message is received.
  void competitionCallback(const std_msgs::String::ConstPtr & msg) {
    if (msg->data == "done" && CompetitionState != "done")
    {
      ROS_INFO("Competition ended.");
    }
    CompetitionState = msg->data;
  }

void startCompetition(ros::NodeHandle & n)
{

  // Create the service client.
  ros::ServiceClient begin_client = n.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
  // Wait for the client to be ready
  if (!begin_client.exists()) {
    ROS_INFO("Waiting for the competition to be ready...");
    begin_client.waitForExistence();
    ROS_INFO("Competition is now ready.");
  }
 	ROS_INFO("Requesting competition start...");
  // To declare the variable in this way where necessary in the code.
  std_srvs::Trigger begin_comp;
  // Call the Service
  begin_client.call(begin_comp);
	if (!begin_comp.response.success) {  // If not successful, print out why.
    ROS_ERROR("Competition service returned failure: %s",begin_comp.response.message.c_str());
  } else {
    ROS_INFO("Competition started!");
  } 

}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "ariac_challenge_node");
	ros::NodeHandle n;

	ros::AsyncSpinner spinner(2);
	spinner.start();

	order_vector.clear();

	ros::Subscriber orderSub = n.subscribe("ariac/orders", 1000, recieveOrder);
	ros::Subscriber cameraSub = n.subscribe("ariac/logial_camera_over_agv1", 1000, logicalCameraCallback);
	// Subscribe to the '/ariac/competition_state' topic.
	ros::Subscriber competitionStateSubscriber = n.subscribe("/ariac/competition_state", 10, &competitionCallback);
	tf2_ros::Buffer tfBuffer;
	moveit::planning_interface::MoveGroupInterface move_group("manipulator");
	tf2_ros::TransformListener tfListener(tfBuffer);
	geometry_msgs::TransformStamped tfStamped;
	moveit::planning_interface::MoveItErrorCode planningError;

	startCompetition(n);
	
	while(ros::ok()){
		ros::spinOnce();
		if(order_vector.size() > 0){
			//Retrieve the transformation

			try {
				tfStamped = tfBuffer.lookupTransform("world", "logical_camera_frame", ros::Time(0.0), ros::Duration(1.0));
				ROS_DEBUG("Transform to [%s] from [%s]", tfStamped.header.frame_id.c_str(), 		tfStamped.child_frame_id.c_str());
				}
			catch (tf2::TransformException &ex) {
				ROS_ERROR("%s", ex.what());
			}
			//end_pose = current_pose;

		tf2::doTransform(current_pose, end_pose, tfStamped);

			end_pose.pose.position.z += 0.10;
			end_pose.pose.orientation.w = 0.707;
			end_pose.pose.orientation.x = 0.0;
			end_pose.pose.orientation.y = 0.707;
			end_pose.pose.orientation.z = 0.0;

			// Set the desired pose for the arm in the arm controller.
			move_group.setPoseTarget(end_pose);
			// Instantiate and create a plan.
			moveit::planning_interface::MoveGroupInterface::Plan movePlan;
			// Create a plan based on the settings (all default settings now) in movePlan.
			planningError = move_group.plan(movePlan);
			// Planning does not always succeed. Check the output.
			// In the event that the plan was created, execute it.
			if(planningError == moveit_msgs::MoveItErrorCodes::SUCCESS)
				move_group.execute(movePlan);
			
		}
	}
  return 0;
}

