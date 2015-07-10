#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include <inttypes.h>


#include "ros/ros.h"
#include "nodes/LegEncoders.h"
#include "nodes/LegCommand.h"
#include "nodes/LegPosition.h"

#include <sstream>

#define GEAR_RATIO 51
#define SATURATION_POS 10*GEAR_RATIO

/*

	Front Left Leg ROS node that reads encoder position in counts from FL_encoders_feedback topic and based on the desired position read from the FL_Position topic, runs a PID controller to calculate the PWM commands. The calculated command is published to the FL_command_dispatch topic.

*/

// Global variables
int16_t acc_error_hip, prev_error_hip, acc_error_knee, prev_error_knee; // Acceleration and previous error values
double moveVelocity_hip, moveVelocity_knee; // The maximum velocity of movement
int32_t desiredPosHip, desiredPosKnee;  // Holds the desired positions
bool sendMsg = false; // Flag set high when we want to transmit a message via topic
nodes::LegCommand command_msg;  // Command message for topic

int32_t tenc[4];
int msg_count = 0;
// PID error initialization function
void InitErrors()
{
	acc_error_hip = prev_error_hip = acc_error_knee = prev_error_knee = 0;
}

// The PID controller function  for knee and hip
// Input : P,I,D gains, saturation , reference point(desired) and current point (Enc_value)
// Output : PWM duty cycle
int8_t PID_controller_hip (double P_Gain, double I_Gain, double D_Gain, double saturation, int32_t Ref_point, int32_t Enc_value) 
{
	int8_t output = 0;
	int32_t error = 0;
	double Pterm, Dterm, Iterm, PIDsum;
	
	// Error calculation
	error = Ref_point - Enc_value;
	Pterm = error * P_Gain;
	Dterm = (error-prev_error_hip) * D_Gain;
	acc_error_hip += error;
	Iterm = acc_error_hip * I_Gain;
	
	// P I D output calculation
	PIDsum = Pterm + Iterm + Dterm;
	
	//Saturation Filter
	
	if(PIDsum > saturation)
	{
		PIDsum = saturation;
	}
	else if(PIDsum < - saturation)
	{
		PIDsum = -saturation;
	}
	
	//Update error
	prev_error_hip = error;
	
	output = (int8_t)((PIDsum/saturation)*100);
	
	return output;
}

int8_t PID_controller_knee (double P_Gain, double I_Gain, double D_Gain, double saturation, int32_t Ref_point, int32_t Enc_value) 
{
	int8_t output = 0;
	int32_t error = 0;
	double Pterm, Dterm, Iterm, PIDsum;
	
	// Error calculation
	error = Ref_point - Enc_value;
	Pterm = error * P_Gain;
	Dterm = (error-prev_error_knee) * D_Gain;
	acc_error_knee += error;
	Iterm = acc_error_knee * I_Gain;
	
	// P I D output calculation
	PIDsum = Pterm + Iterm + Dterm;
	
	//Saturation Filter
	
	if(PIDsum > saturation)
	{
		PIDsum = saturation;
	}
	else if(PIDsum < - saturation)
	{
		PIDsum = -saturation;
	}
	
	//Update error
	prev_error_knee = error;
	
	output = (int8_t)((PIDsum/saturation)*100);
	
	return output;
}

// Callback function for reception of Encoder value message from topic
void encoderCallback(const nodes::LegEncoders::ConstPtr& msg)
{
 // Calculate the PWM duty cycle from the PID function and raise flag to send command message
 //ROS_INFO("Desired pos: %d\n", desiredPosHip);
 command_msg.pwm_duty_hip = PID_controller_hip (0.05, 0.0, 0.0, moveVelocity_hip, desiredPosHip, (int32_t)msg->encoder_hip);
 //command_msg.pwm_duty_knee = PID_controller_knee (0.05, 0.0, 0.0, moveVelocity_knee, desiredPosKnee, (int32_t)msg->encoder_knee);
 //ROS_INFO("pwm_duty_hip callback command: %d\n", command_msg.pwm_duty_hip);
 tenc[0] = (int32_t)msg->encoder_hip;
 //ROS_INFO("Encoder hip callback: %d\n", msg->encoder_hip);
 msg_count++;
 
 sendMsg = true;
}

// Callback function for reception of desired position message from topic
void positionCallback(const nodes::LegPosition::ConstPtr& msg)
{
 // Extract the value and set it to the corresponding variable
 //desiredPosHip = (int32_t)msg->position_hip;
 //desiredPosHip = 1000;
 desiredPosKnee = (int32_t)msg->position_knee;
 
}

// Main Function
int main(int argc, char **argv)
{
  int i;
  nodes::LegEncoders encoders_msg;
  moveVelocity_hip = moveVelocity_knee = SATURATION_POS;
  desiredPosHip = desiredPosKnee = 1000;
  
  
  
  InitErrors();

  // Initialize ROS node
  ros::init(argc, argv, "FL_leg_control");
  ros::NodeHandle n;
  // Initialize the publisher for command data post
  ros::Publisher motor_command_pub = n.advertise<nodes::LegCommand>("FL_command_dispatch", 1000);
  // Initialize the subscribers for Encoder data reception and desired position reception
  ros::Subscriber motor_interface_sub = n.subscribe<nodes::LegEncoders>("FL_encoders_feedback", 1000, encoderCallback);
  //ros::Subscriber motor_position_sub = n.subscribe("FL_Position", 1000, positionCallback);
  ros::Rate loop_rate(120);
  
  ROS_INFO("Staring communication with FL leg interface node.");
  while (ros::ok())
  {		
  	
	    // If a PWM command has been calculated publish it to the topic
	    if(sendMsg)
	    {
			sendMsg = false;
			
			motor_command_pub.publish(command_msg);
	    }
	    if(msg_count == 50)
            {
  		msg_count = 0;
		ROS_INFO("publish pwm: %d", command_msg.pwm_duty_hip);
  		ROS_INFO("ENC : %d\n",tenc[0]);
            }
	    ros::spinOnce();

	    loop_rate.sleep();
  }


  return 0;
}
