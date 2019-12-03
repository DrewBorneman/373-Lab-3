// Include the header file for Trigger.
#include "std_srvs/Trigger.h"
#include "ros/ros.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "osrf_gear/Order.h"
#include "osrf_gear/GetMaterialLocations.h"
#include "osrf_gear/LogicalCameraImage.h"
#include "osrf_gear/VacuumGripperState.h"
#include "osrf_gear/VacuumGripperControl.h"
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

#define SHOULDERPANJOINT 0
#define SHOULDERLIFTJOINT 1
#define ELBOWJOINT 2
#define WRIST1JOINT 3
#define WRIST2JOINT 4
#define WRIST3JOINT 5

#define PI 3.14159265

std::vector<osrf_gear::Order> order_vector;
std::string ObjectType = "piston_rod_part";
std::string CompetitionState;
sensor_msgs::JointState joint_states;
osrf_gear::VacuumGripperState gripper_state;
osrf_gear::Model camera_models[32];
int numberofparts = 12;
int partNumber = 0;
geometry_msgs::PoseStamped current_pose;

void recieveOrder(const osrf_gear::Order::ConstPtr & order)
{

  ROS_INFO("Order recieved!\n");
  order_vector.push_back(*order);
}

void jointCB(const sensor_msgs::JointState::ConstPtr & jointStateMsg){
	joint_states = *jointStateMsg;
}

void gripperCB(const osrf_gear::VacuumGripperState::ConstPtr & gripperStateMsg){
	gripper_state = *gripperStateMsg;
}

void logicalCameraCallback(const osrf_gear::LogicalCameraImage::ConstPtr & imageMsg)
{ 
    //ROS_INFO("Model: %s\n",imageMsg->models[1]);
		//if(imageMsg->models[1].type == ObjectType)	we're assuming everything here is piston_rod_part
		for(int i = 0; i<imageMsg->models.size();i++)
			camera_models[i] = imageMsg->models[i];
		//camera_models = imageMsg->models;
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

void move_arm(ros::Publisher joint_trajectory_publisher,double x_pos,double y_pos,double z_pos)
{
	double q[] = {3.14, -1.13, 1.51, 3.77, -1.51, 0};	//initial values for the joint angles
	double T[4][4];		//current pose
	double q_sols[8][6];
	double constraints[6][2] = {{(PI/2),(3*(PI/2))},{PI,(2*PI)},{0,(2*PI)},{0,(PI)},{((PI/2)-0.1),((PI/2)+0.1)},{0,(2*PI)}};
	int num_sol;
	int best_num;
	int count = 0;
	int trajectory_scores[8] = {0,0,0,0,0,0,0,0};
	trajectory_msgs::JointTrajectory joint_trajectory;
	
			ROS_INFO("Moving to x: %f,y:%f,z:%f",x_pos,y_pos,z_pos);


			double endPoseMatrix[4][4] = {{0.0, -1.0, 0.0, x_pos}, {0.0, 0.0, 1.0, y_pos}, {-1.0, 0.0, 0.0 , z_pos}, {0.0, 0.0, 0.0, 1.0}};
			
			//ur_kinematics::forward(&jointAngles_[0], &endPoseMatrix[0][0]);
			num_sol = ur_kinematics::inverse(&endPoseMatrix[0][0], &q_sols[0][0], 0.0);
			//ROS_INFO("no of solutions: %i\n",num_sol);
			//ROS_INFO("all solutions: ");
			best_num = -1;
			/*
			for(int i=0;i<num_sol;i++){
				ROS_INFO("Solution %i: ",i);
				for(int j=0;j<6;j++)
					ROS_INFO("%f",q_sols[i][j]);
			}*/

			//double test_sol[] = {PI,3*(PI/2),0,PI/2,0,0};			

			for(int i = 0;i<num_sol;i++){
				for(int j =0;j<6;j++){
					if(q_sols[i][j]>constraints[j][0] && q_sols[i][j]<constraints[j][1])
						trajectory_scores[i]++;		
				}	
			}
			for(int i =0; i<num_sol;i++){
				if(trajectory_scores[i]==6){
					best_num = i;
				}
			}

			/*for(int i=0;i<num_sol;i++){
				if(q_sols[i][SHOULDERPANJOINT]>(PI/2)&&q_sols[i][SHOULDERPANJOINT]<(3*(PI/2))){	//base joint must be facing forward
					if(q_sols[i][SHOULDERLIFTJOINT]>PI&&q_sols[i][SHOULDERLIFTJOINT]<(2*PI)){	//shoulder joint cannot be below the table/linear actuator
						if(q_sols[i][WRIST2JOINT]>((PI/2)-0.1) && q_sols[i][WRIST2JOINT]<((PI/2)+0.1)){	//second wrist joint cannot face upwards		
							best_num = i;
							break;
						}
					}
				}			
			}*/
			
			//double zero_sol[] = {(PI/2),(PI/2),0,0,0,0};

			ROS_INFO("Selecting part no.:%i\n",partNumber);

			if (best_num > -1){
				ROS_INFO("Best solution:%i\n", best_num);
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
						if (joint_trajectory.joint_names[indy] == joint_states.name[indz]) {
							joint_trajectory.points[0].positions[indy] = joint_states.position[indz];
							break;
						}
					}
				}
				
				joint_trajectory.points[0].time_from_start = ros::Duration(0.0);

				joint_trajectory.points[1].positions.resize(joint_trajectory.joint_names.size());

				for (int indy = 0; indy < 6; indy++) {
					joint_trajectory.points[1].positions[indy + 1] = q_sols[best_num][indy];
				}
				joint_trajectory.points[1].time_from_start = ros::Duration(1.0);		
				
				joint_trajectory_publisher.publish(joint_trajectory);
				
				}
	ros::Duration(2.5).sleep();
}

void move_actuator(ros::Publisher joint_trajectory_publisher, bool to_agv)
{
	trajectory_msgs::JointTrajectory joint_trajectory;
	int count = 0;

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
						if (joint_trajectory.joint_names[indy] == joint_states.name[indz]) {
							joint_trajectory.points[0].positions[indy] = joint_states.position[indz];
							break;
						}
					}
				}
				
				joint_trajectory.points[1].positions.resize(joint_trajectory.joint_names.size());

				joint_trajectory.points[0].time_from_start = ros::Duration(0.0);
				joint_trajectory.points[1].positions = joint_trajectory.points[0].positions;

				if(to_agv){
					ROS_INFO("Going to AGV");
					joint_trajectory.points[0].positions[0] = 2.1;
					joint_trajectory.points[1].positions[0] = 4.2;
				}else{
					ROS_INFO("Going to home position");
					joint_trajectory.points[0].positions[0] = 4.2;
					joint_trajectory.points[1].positions[0] = 2.1;
				}
				
				joint_trajectory.points[1].time_from_start = ros::Duration(2.0);		
				
				joint_trajectory_publisher.publish(joint_trajectory);
				
		

	ros::Duration(2.5).sleep();
}

geometry_msgs::PoseStamped get_transform(){

	geometry_msgs::PoseStamped end_pose;
	tf2_ros::Buffer tfBuffer;
	tf2_ros::TransformListener tfListener(tfBuffer);
	geometry_msgs::TransformStamped tfStamped;
	std::string errStr;
	
		try {
			tfBuffer.canTransform("world","logical_camera_frame", ros::Time(), ros::Duration(2.0), &errStr);
			tfStamped = tfBuffer.lookupTransform("base_link", "logical_camera_frame", ros::Time(0.0), ros::Duration(1.0));
			ROS_DEBUG("Transform to [%s] from [%s]", tfStamped.header.frame_id.c_str(), 		tfStamped.child_frame_id.c_str());
			}
		catch (tf2::TransformException &ex) {
			ROS_ERROR("%s", ex.what());
		}	
				
		tf2::doTransform(current_pose, end_pose, tfStamped);
		
		end_pose.pose.orientation.w = 0.707;
		end_pose.pose.orientation.x = 0.0;
		end_pose.pose.orientation.y = 0.707;
		end_pose.pose.orientation.z = 0.0;
		
		return end_pose;

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "ariac_challenge_node");
	ros::NodeHandle n;

	ros::AsyncSpinner spinner(2);
	spinner.start();
	int i = 0;
	order_vector.clear();

	ros::Subscriber orderSub = n.subscribe("ariac/orders", 1000, recieveOrder);

	ros::Subscriber cameraSub = n.subscribe("ariac/logical_camera", 1000, logicalCameraCallback);
	// Subscribe to the '/ariac/competition_state' topic.
	ros::Subscriber competitionStateSubscriber = n.subscribe("/ariac/competition_state", 10, &competitionCallback);
	ros::Publisher joint_trajectory_publisher = n.advertise<trajectory_msgs::JointTrajectory>("/ariac/arm/command", 10);
	//ros::Publisher gripper_publisher = n.advertise<osrf_gear::VacuumGripperControl>("/ariac/gripper/control", 10);
	ros::ServiceClient gripper_client = n.serviceClient<osrf_gear::VacuumGripperControl>("/ariac/gripper/control");
	ros::Subscriber gripper_subscriber = n.subscribe("ariac/gripper/state", 10, gripperCB);
	ros::Subscriber joint_states_h = n.subscribe("ariac/joint_states", 10, jointCB);
	geometry_msgs::PoseStamped end_pose;
	osrf_gear::VacuumGripperControl gripper_control;
	startCompetition(n);
	while(joint_states.name.size()<1 && ros::ok())
		ros::spinOnce();
	while(ros::ok()){
		ros::spinOnce();
		if(order_vector.size() > 0){
			//Retrieve the transformation
				if(partNumber<numberofparts){
					partNumber++;
				}else{
					partNumber=0;
				}

				if(camera_models[partNumber].type == ObjectType){

					current_pose.pose = camera_models[partNumber].pose;

					//move above the piece
					end_pose = get_transform();
					move_arm(joint_trajectory_publisher, end_pose.pose.position.x,end_pose.pose.position.y,end_pose.pose.position.z+0.2);
					//move down on the piece
					end_pose = get_transform();
					move_arm(joint_trajectory_publisher, end_pose.pose.position.x,end_pose.pose.position.y,end_pose.pose.position.z+0.02);
					//vacuum gripper
					//update the gripper state from the callback
					ros::Duration(0.1).sleep();
					//enable if not already grabbing
					if(gripper_state.enabled==false && gripper_state.attached==false){
						gripper_control.request.enable = true;
						gripper_client.call(gripper_control);
						i = 0;
						while(gripper_state.attached==false && i <= 5){
							ros::Duration(0.1).sleep();
							i++;
							if(i==5)
								ROS_ERROR("Couldn't pick up the part");	
						}	
					}else{
						ROS_INFO("Gripper currently in use");
					}
					ros::Duration(0.1).sleep();
					//pick up
					end_pose = get_transform();
					move_arm(joint_trajectory_publisher, end_pose.pose.position.x,end_pose.pose.position.y,end_pose.pose.position.z+0.2);
					ros::Duration(1).sleep();
					//take to AGV
					move_actuator(joint_trajectory_publisher, true);
					ros::Duration(1).sleep();
					//bring back
					move_actuator(joint_trajectory_publisher, false);
					ros::Duration(1).sleep();
					//move back down
					end_pose = get_transform();
					move_arm(joint_trajectory_publisher, end_pose.pose.position.x,end_pose.pose.position.y,end_pose.pose.position.z+0.02);
					//drop
					ros::Duration(0.5).sleep();
					gripper_control.request.enable = false;
						gripper_client.call(gripper_control);
					//move above the piece
					end_pose = get_transform();
					move_arm(joint_trajectory_publisher, end_pose.pose.position.x,end_pose.pose.position.y,end_pose.pose.position.z+0.2);
					
				}
			
		}
	}
  return 0;
}

