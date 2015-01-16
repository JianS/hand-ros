/*
 * allegroNode.cpp
 *
 *  Created on: Nov 14, 2012
 *  Authors: Alex ALSPACH, Seungsu KIM
 */
 
// CONTROL LOOP TEMPLATE 
// Using  timer callback  

// For the most basic torque control algorithm, 
// you need only add code where it says:
	// =============================== //
	// = COMPUTE control torque here = //
	// =============================== //	
// in the timer callback below. 
// READ, COMPUTE, and WRITE and PUBLISH
// are contrained within this callback.
 
#include <iostream>
#include <boost/thread/thread.hpp>
#include <boost/date_time.hpp>
#include <boost/thread/locks.hpp>

#include "ros/ros.h"
#include "ros/service.h"
#include "ros/service_server.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/String.h"

#include <stdio.h>
#include <iostream>
#include <string>
#include <math.h>
#include <Eigen/Dense>

//#include "BHand/BHand.h"
#include "controlAllegroHand.h"

// Topics
#define JOINT_STATE_TOPIC "/allegroHand/joint_states"
#define JOINT_CMD_TOPIC "/allegroHand/joint_cmd"
#define EXT_CMD_TOPIC "/allegroHand/external_cmd"

#define JOINT_DESIRED_TOPIC "/allegroHand/joint_desired_states"
#define JOINT_CURRENT_TOPIC "/allegroHand/joint_current_states"

using Eigen::MatrixXd;
using namespace std;

double desired_position[DOF_JOINTS]				= {0.0};
double current_position[DOF_JOINTS] 			= {0.0};
double previous_position[DOF_JOINTS]			= {0.0};
double current_position_filtered[DOF_JOINTS] 	= {0.0};
double previous_position_filtered[DOF_JOINTS]	= {0.0};

double desired_velocity[DOF_JOINTS]				= {0.0};
double current_velocity[DOF_JOINTS] 			= {0.0};
double previous_velocity[DOF_JOINTS] 			= {0.0};
double current_velocity_filtered[DOF_JOINTS] 	= {0.0};

double desired_torque[DOF_JOINTS] 				= {0.0};

// enumeration serves as the flag of loop number
static int nnnnn = 0;


std::string jointNames[DOF_JOINTS] 	= {    "joint_0.0",    "joint_1.0",    "joint_2.0",   "joint_3.0" , 
										   "joint_4.0",    "joint_5.0",    "joint_6.0",   "joint_7.0" , 
									  	   "joint_8.0",    "joint_9.0",    "joint_10.0",  "joint_11.0", 
										   "joint_12.0",   "joint_13.0",   "joint_14.0",  "joint_15.0" };

int frame = 0;

// Flags
int lEmergencyStop = 0;

boost::mutex *mutex;

// ROS Messages
ros::Publisher joint_state_pub;
ros::Publisher joint_desired_state_pub;
ros::Publisher joint_current_state_pub;

ros::Subscriber joint_cmd_sub;				// Handles external joint command (eg. sensor_msgs/JointState)
ros::Subscriber external_cmd_sub;			// Handles any other type of eternal command (eg. std_msgs/String)
sensor_msgs::JointState msgJoint_desired;	// Desired Position, Desired Velocity, Desired Torque
sensor_msgs::JointState msgJoint_current;	// Current Position, Current Velocity, NOTE: Current Torque Unavailable
sensor_msgs::JointState msgJoint;			// Collects most relavent information in one message: Current Position, Current Velocity, Control Torque
std::string  ext_cmd;

// ROS Time
ros::Time tstart;
ros::Time tnow;
double secs;
double dt;

// Initialize CAN device		
controlAllegroHand *canDevice;

// These functions are self-defined
MatrixXd FKResult(double, double, double, double);
MatrixXd SpaceJacobian(double, double, double, double);
MatrixXd TargetTrajectory(int);
MatrixXd TorqueFeedback(double, double, double, double, int, double, double, double);
MatrixXd TargetVelocity(int);

// Called when a desired joint position message is received
void SetjointCallback(const sensor_msgs::JointState& msg)
{
	mutex->lock();
	for(int i=0;i<DOF_JOINTS;i++) desired_position[i] = msg.position[i];
	mutex->unlock();	
}

// Called when an external (string) message is received
void extCmdCallback(const std_msgs::String::ConstPtr& msg)
{
	ROS_INFO("CTRL: Processing external command");
	ext_cmd = msg->data.c_str();

	// Compare the message received to an expected input
	if (ext_cmd.compare("comparisonString") == 0)
	{
		// Do something
	}    
	else
	{
		// Do something else
	}	
}


// In case of the Allegro Hand, this callback is processed every 0.003 seconds
void timerCallback(const ros::TimerEvent& event)
{
	// Matrix to receive the control torque in a idealized way
	MatrixXd forceResult(4,1);
	nnnnn++;
	
	// Calculate loop time;
	tnow = ros::Time::now();
	dt = 1e-9*(tnow - tstart).nsec;
	tstart = tnow;
		
	// save last joint position for velocity calc
	for(int i=0; i<DOF_JOINTS; i++) previous_position[i] = current_position[i];
		
		
		
		
/*  ================================= 
    =       CAN COMMUNICATION       = 
    ================================= */	
	canDevice->setTorque(desired_torque);		// WRITE joint torque
	lEmergencyStop = canDevice->update(); 		// Returns -1 in case of an error
	canDevice->getJointInfo(current_position);	// READ current joint positions

		
		
				
	// Stop program and shutdown node when Allegro Hand is switched off
	if( lEmergencyStop < 0 )
	{
		ROS_ERROR("\n\nAllegro Hand Node is Shutting Down! (Emergency Stop)\n");
		ros::shutdown();
	}


/*  ================================= 
	=       LOWPASS FILTERING       =   
	================================= */
	for(int i=0; i<DOF_JOINTS; i++)    
	{
		current_position_filtered[i] = (0.6*current_position_filtered[i]) + (0.198*previous_position[i]) + (0.198*current_position[i]);
		current_velocity[i] = (current_position_filtered[i] - previous_position_filtered[i]) / dt;
		current_velocity_filtered[i] = (0.6*current_velocity_filtered[i]) + (0.198*previous_velocity[i]) + (0.198*current_velocity[i]);
	}	





    if(frame>100) // give the low pass filters 100 iterations (0.03s) to build up good data.
	{

/*  ================================= 
	=  COMPUTE CONTROL TORQUE HERE  =   		// current_position_filtered --> desired_torque //
	================================= */	
	forceResult = TorqueFeedback(current_position_filtered[4], current_position_filtered[5], current_position_filtered[6], current_position_filtered[7], nnnnn-320, 660.0/3,  660.0/3, 660.0/3);
	desired_torque[4] = forceResult(0, 0);
	desired_torque[5] = forceResult(1, 0);
	desired_torque[6] = forceResult(2, 0);
	desired_torque[7] = forceResult(3, 0);
	}





	// PUBLISH current position, velocity and effort (torque)
	msgJoint.header.stamp 		= tnow;	
	for(int i=0; i<DOF_JOINTS; i++)
	{
		msgJoint.position[i] 	= current_position_filtered[i];
		msgJoint.velocity[i] 	= current_velocity_filtered[i];
		msgJoint.effort[i] 		= desired_torque[i];
	}
	joint_state_pub.publish(msgJoint);
		
		
	frame++;

} // end timerCallback








int main(int argc, char** argv)
{
	using namespace std;
	
	ros::init(argc, argv, "allegro_hand_core");
	ros::Time::init();
	
	ros::NodeHandle nh;

	// Setup timer callback (ALLEGRO_CONTROL_TIME_INTERVAL = 0.003)
	ros::Timer timer = nh.createTimer(ros::Duration(0.003), timerCallback);

	mutex = new boost::mutex();

	// Publisher and Subscribers
	joint_state_pub = nh.advertise<sensor_msgs::JointState>(JOINT_STATE_TOPIC, 3);
	joint_cmd_sub = nh.subscribe(JOINT_CMD_TOPIC, 3, SetjointCallback);
	external_cmd_sub = nh.subscribe(EXT_CMD_TOPIC, 1, extCmdCallback);
	
	// Create arrays 16 long for each of the four joint state components
	msgJoint.position.resize(DOF_JOINTS);
	msgJoint.velocity.resize(DOF_JOINTS);
	msgJoint.effort.resize(DOF_JOINTS);
	msgJoint.name.resize(DOF_JOINTS);

	// Joint names (for use with joint_state_publisher GUI - matches URDF)
	for(int i=0; i<DOF_JOINTS; i++)	msgJoint.name[i] = jointNames[i];	

	
	// Get Allegro Hand information from parameter server
	// This information is found in the Hand-specific "zero.yaml" file from the allegro_hand_description package	
	string robot_name, whichHand, manufacturer, origin, serial;
	double version;
	ros::param::get("~hand_info/robot_name",robot_name);
	ros::param::get("~hand_info/which_hand",whichHand);
	ros::param::get("~hand_info/manufacturer",manufacturer);
	ros::param::get("~hand_info/origin",origin);
	ros::param::get("~hand_info/serial",serial);
	ros::param::get("~hand_info/version",version);
	

	// Initialize CAN device
	canDevice = new controlAllegroHand();
	canDevice->init();
	usleep(3000);
	
	// Dump Allegro Hand information to the terminal	
	cout << endl << endl << robot_name << " v" << version << endl << serial << " (" << whichHand << ")" << endl << manufacturer << endl << origin << endl << endl;

	// Initialize torque at zero
	for(int i=0; i<16; i++) desired_torque[i] = 0.0;

	// Start ROS time
	tstart = ros::Time::now();

	// Starts control loop, message pub/subs and all other callbacks
	ros::spin();			

	// Clean shutdown: shutdown node, shutdown can, be polite.
	nh.shutdown();
	delete canDevice;
	printf("\nAllegro Hand Node has been shut down. Bye!\n\n");
	return 0;
	
}



// Self-defined functions
MatrixXd FKResult( double theta1, double theta2, double theta3, double theta4 )
{
  double L1 = .0157;
  double L2 = .054;
  double L3 = .0383998;
  
  MatrixXd fix(4, 1);
  fix << 0.0068490,
  0,
  0.0528512,
  1;
  
  MatrixXd fkResult(3,1);
  MatrixXd fkResultTemp(4,1);
  MatrixXd fk(4,4);
  fk << cos(theta1)*cos(theta2+theta3+theta4), -sin(theta1), cos(theta1)*sin(theta2+theta3+theta4), cos(theta1)*(L2*sin(theta2)+L3*sin(theta2+theta3)),
  cos(theta2+theta3+theta4)*sin(theta1), cos(theta1), sin(theta1)*sin(theta2+theta3+theta4), sin(theta1)*(L2*sin(theta2)+L3*sin(theta2+theta3)),
  -sin(theta2+theta3+theta4), 0, cos(theta2+theta3+theta4), L1+L2*cos(theta2)+L3*cos(theta2+theta3),
  0,0,0,1;
  
  fkResultTemp = fk*fix;
  
  fkResult << fkResultTemp(0,0),
  fkResultTemp(1,0),
  fkResultTemp(2,0);
  
  return fkResult;
}

MatrixXd SpaceJacobian (double theta1, double theta2, double theta3, double theta4)
{
  double L1 = 0.0157;
  double L2 = 0.054;
  double L3 = 0.0383998;
  
  MatrixXd jacobian(6, 4);
  jacobian << 0,-sin(theta1),-sin(theta1),-sin(theta1),
  0,cos(theta1),cos(theta1),cos(theta1),
  1,0,0,0,
  0,-L1*cos(theta1),-cos(theta1)*(L1 + L2*cos(theta2)),-cos(theta1)*(L1+L2*cos(theta2)+L3*cos(theta2+theta3)),
  0,-L1*sin(theta1), -(L1+L2*cos(theta2))*sin(theta1),-(L1+L2*cos(theta2)+L3*cos(theta2+theta3))*sin(theta1),
  0,0, L2*sin(theta2), L2*sin(theta2) + L3*sin(theta2+theta3);
  
  return jacobian;
  
}


MatrixXd TargetTrajectory(int n)
{
 /* -0.1350591202115246, 0.41364274215009617, 0.41364274215009617, 0.41364274215009617
  
  -0.04700224165990417, 0.709232855336447, 1.685279017370342, -0.1451493775332897*/
 int number;
 MatrixXd point1(3,1) ;
 MatrixXd point2(3,1);
 MatrixXd pointTarget(3,1);
 double targetTime = 2.0; // follow the trajecctory in targetTime secs
 double dt;
 dt = 330; // The control loop works at frequency of 330 Hz
 number = floor(targetTime*dt);
 
 
 point1 << 0.101261,
 -0.01376,
 -0.01376;
 point2 << 0.0979962,
 -0.00460944,
 -0.00999974;

 if(n<1){pointTarget = point1;return pointTarget;}
 else if(n>number){pointTarget = point2; return pointTarget;}
 else {pointTarget = point1 + (point2 - point1)*n/number; return pointTarget;}
  
}

MatrixXd TargetVelocity(int n)
{
	dt = 330;
	MatrixXd pointPrevious(3,1);
	MatrixXd pointForward(3,1);
	MatrixXd goalVelocity(3,1);
	
	pointPrevious = TargetTrajectory(n-1);
	pointForward = TargetTrajectory(n+1);
	
	goalVelocity = (pointForward - pointPrevious)*2*dt;  // I'm not sure if this arithmetic operation is right in syntax
	
	return(goalVelocity);
	
}

MatrixXd TorqueFeedback(double theta1, double theta2, double theta3, double theta4, int n, double kpx, double kpy, double kpz)
{
  MatrixXd torqueCalculated(4, 1);
  MatrixXd jacobianTranspose(4, 6);
  MatrixXd coordinateMeasured(3, 1); 
  MatrixXd coordinateGoal(3,1);
  MatrixXd externalForce(6,1);
  int delay = 300;
  // the measured coordinates
  coordinateMeasured = FKResult(theta1, theta2, theta3, theta4);
  coordinateGoal = TargetTrajectory(n - delay);
  externalForce << 0,
  0,
  0,
  kpx*(coordinateGoal(0,0) - coordinateMeasured(0,0)),
  kpy*(coordinateGoal(1,0) - coordinateMeasured(1,0)),
  kpz*(coordinateGoal(2,0) - coordinateMeasured(2,0));
  
  //calculate the torque 
  jacobianTranspose = (SpaceJacobian(theta1, theta2, theta3, theta4)).transpose();
  torqueCalculated = jacobianTranspose*externalForce;
  
  return torqueCalculated;
  
}


