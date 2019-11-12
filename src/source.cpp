// Include the header file for Trigger.
#include "std_srvs/Trigger.h"
#include "ros/ros.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "osrf_gear/Order.h"
#include "osrf_gear/GetMaterialLocations.h"
#include "osrf_gear/LogicalCameraImage.h"
#include "osrf_gear/Model.h"
#include "std_msgs/String.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "sensor_msgs/JointState.h"


#include <vector>
//includes from ur_kinematics
#include "ur_kinematics/ur_kin.h"
//Transformation header files
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/TransformStamped.h"

#define PI 3.14159265


std::vector<osrf_gear::Order> order_vector;
std::string ObjectType = "piston_rod_part";
std::string CompetitionState;
sensor_msgs::JointState joint_states;

	//create variables
	geometry_msgs::PoseStamped current_pose, end_pose;

void recieveOrder(const osrf_gear::Order::ConstPtr & order)
{

  ROS_INFO("Order recieved:%s\n", *order);
  order_vector.push_back(*order);
}

void jointCB(const sensor_msgs::JointState::ConstPtr & jointStateMsg){
	joint_states = *jointStateMsg;
}

void logicalCameraCallback(const osrf_gear::LogicalCameraImage::ConstPtr & imageMsg)
{ 
    //ROS_INFO("Model: %s\n",imageMsg->models[1]);
		//if(imageMsg->models[1].type == ObjectType)	we're assuming everything here is piston_rod_part
		current_pose.pose = imageMsg->models[0].pose; // i think we need to add /ariac/{name} because that's the logical camera's output. I'm just not positive where to put it
		//ROS_INFO("Part Pose: X:%f, Y:%f, Z:%f\n", current_pose.pose.position.x,current_pose.pose.position.y,current_pose.pose.position.z);
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

	ros::Subscriber cameraSub = n.subscribe("ariac/logical_camera", 1000, logicalCameraCallback);
	// Subscribe to the '/ariac/competition_state' topic.
	ros::Subscriber competitionStateSubscriber = n.subscribe("/ariac/competition_state", 10, &competitionCallback);
	ros::Publisher joint_trajectory_publisher = n.advertise<trajectory_msgs::JointTrajectory>("/ariac/arm/command", 10);
	ros::Subscriber joint_states_h = n.subscribe("ariac/joint_states", 10, jointCB);
	tf2_ros::Buffer tfBuffer;
	//moveit::planning_interface::MoveGroupInterface move_group("manipulator");
	tf2_ros::TransformListener tfListener(tfBuffer);
	geometry_msgs::TransformStamped tfStamped;
	//moveit::planning_interface::MoveItErrorCode planningError;
	double q[] = {3.14, -1.13, 1.51, 3.77, -1.51, 0};	//initial values for the joint angles
	double T[4][4];		//current pose
	double q_sols[8][6];
	std::string errStr;
	int num_sol;
	int best_num;
	int count = 0;
	double best_sol[6];
	trajectory_msgs::JointTrajectory joint_trajectory;
	startCompetition(n);
	
	while(ros::ok()){
		ros::spinOnce();
		if(order_vector.size() > 0){
			//Retrieve the transformation

			try {
				tfBuffer.canTransform("world","logical_camera_frame", ros::Time(), ros::Duration(5.0), &errStr);
				tfStamped = tfBuffer.lookupTransform("world", "logical_camera_frame", ros::Time(0.0), ros::Duration(1.0));
				ROS_DEBUG("Transform to [%s] from [%s]", tfStamped.header.frame_id.c_str(), 		tfStamped.child_frame_id.c_str());
				}
			catch (tf2::TransformException &ex) {
				ROS_ERROR("%s", ex.what());
			}
			//end_pose = current_pose;

			tf2::doTransform(current_pose, end_pose, tfStamped);

			end_pose.pose.orientation.w = 0.707;
			end_pose.pose.orientation.x = 0.0;
			end_pose.pose.orientation.y = 0.707;
			end_pose.pose.orientation.z = 0.0;

			//ROS_INFO("Part Pose: X:%f, Y:%f, Z:%f\n", end_pose.pose.position.x,end_pose.pose.position.y,end_pose.pose.position.z);

			double endPoseMatrix[4][4] = {{0.0, -1.0, 0.0, static_cast<double>(end_pose.pose.position.x)}, {0.0, 0.0, 1.0, static_cast<double>(end_pose.pose.position.y)}, {-1.0, 0.0, 0.0 , static_cast<double>(end_pose.pose.position.z+0.3)}, {0.0, 0.0, 0.0, 1.0}};
			
			//ur_kinematics::forward(&jointAngles_[0], &endPoseMatrix[0][0]);
			num_sol = ur_kinematics::inverse(&endPoseMatrix[0][0], &q_sols[0][0], 0.0);
			//ROS_INFO("no of solutions: %i\n",num_sol);
			//ROS_INFO("all solutions: ");
			best_num = -1;
			for(int i=0;i<num_sol;i++){
				//for(int j=0;j<6;j++)
					//ROS_INFO("%d, ", q_sols[i][j]);			
				//ROS_INFO("\n");
				if(q_sols[i][0]>(-PI/2)&&q_sols[i][0]<(PI/2)){	//base joint must be facing forward
					if(q_sols[i][1]>0&&q_sols[i][1]<(PI)){	//shoulder joint cannot be below the table/linear actuator
						if(q_sols[i][3]>(-PI/2)&&q_sols[i][3]<(PI/2)){	//first wrist joint cannot bend backwards		
							best_num = i;
							break;
						}
					}
				}			
			}
			if (best_num > -1){
				ROS_INFO("Best solution:%i\n", best_num);
				for (int i = 0; i < 6; i++)
				{
					best_sol[i] = q_sols[best_num][i];
				}
				joint_trajectory.header.seq = count++;
				joint_trajectory.header.stamp = ros::Time::now();
				joint_trajectory.header.frame_id = "/world";
				joint_trajectory.joint_names.clear();
				joint_trajectory.joint_names.push_back("linear_arm_actuator_joint");
				joint_trajectory.joint_names.push_back("shoulder_pan_joint");
				joint_trajectory.joint_names.push_back("shoulder_lift_joint");
				joint_trajectory.joint_names.push_back("elbow_joint");
				joint_trajectory.joint_names.push_back("wrist_1_joint");
				joint_trajectory.joint_names.push_back("wrist_2_joint");
				joint_trajectory.joint_names.push_back("wrist_3_joint");

				joint_trajectory.points.resize(2);
				joint_trajectory.points[0].positions.resize(joint_trajectory.joint_names.size());
				for (int indy = 0; indy < joint_trajectory.joint_names.size(); indy++) {
					for (int indz = 0; indz < joint_states.name.size(); indz++) {
						//if (joint_trajectory.joint_names[indy] == joint_states.name[indz]) {
							joint_trajectory.points[0].positions[indy] = joint_states.position[indz];
							break;
						//}
					}
				}
				
				joint_trajectory.points[0].time_from_start = ros::Duration(0.0);
				
				joint_trajectory.points[1].positions.resize(joint_trajectory.joint_names.size());
				joint_trajectory.points[1].positions[0] = joint_states.position[1];
				for (int indy = 0; indy < 6; indy++) {
					joint_trajectory.points[1].positions[indy + 1] = q_sols[best_num][indy];
				}
				joint_trajectory.points[1].time_from_start = ros::Duration(1.0);		
				
				joint_trajectory_publisher.publish(joint_trajectory);
				
			}
			else{
				//ROS_INFO("No solution exists\n");
			}
			
			// Set the desired pose for the arm in the arm controller.
			//move_group.setPoseTarget(end_pose);
			// Instantiate and create a plan.
			//moveit::planning_interface::MoveGroupInterface::Plan movePlan;
			// Create a plan based on the settings (all default settings now) in movePlan.
			//planningError = move_group.plan(movePlan);
			// Planning does not always succeed. Check the output.
			// In the event that the plan was created, execute it.
			//if(planningError == moveit_msgs::MoveItErrorCodes::SUCCESS)
			//	move_group.execute(movePlan);
			
		}
	}
  return 0;
}

