#include "ros/ros.h"
#include "std_msgs/String.h"

#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include <inttypes.h>


#include "nodes/LegEncoders.h"
#include "nodes/LegCommand.h"
#include "nodes/LegPosition.h"
 
/**
  * This tutorial demonstrates simple receipt of messages over the ROS system.
*/


int16_t pwm_periodA, pwm_periodB;

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}
 
void commandCallback(const nodes::LegCommand::ConstPtr& msg)
     {
		 //pwmHip1 = (int8_t) msg->pwm_duty_hip;
		 pwm_periodA = ((int16_t)msg->pwm_duty_hip * 10);
		 //ROS_INFO("PWM value callback: %d", msg->pwm_duty_hip);
		 ROS_INFO("PWM value callback: %d", pwm_periodA);
     }
	
     void encoderCallback(const nodes::LegEncoders::ConstPtr& msg)
     {
	 // Calculate the PWM duty cycle from the PID function and raise flag to send command message
	 //ROS_INFO("Desired pos: %d\n", desiredPosHip);
	 //command_msg.pwm_duty_hip = PID_controller_hip (0.05, 0.0, 0.0, moveVelocity_hip, desiredPosHip, (int32_t)msg->encoder_hip);
	 //command_msg.pwm_duty_knee = PID_controller_knee (0.05, 0.0, 0.0, moveVelocity_knee, desiredPosKnee, (int32_t)msg->encoder_knee);
	 //ROS_INFO("pwm_duty_hip callback command: %d\n", command_msg.pwm_duty_hip);
	 
	 ROS_INFO("Encoder hip callback: %d\n", msg->encoder_hip);
	 
      }

int main(int argc, char **argv)
{
   /**
    * The ros::init() function needs to see argc and argv so that it can perform
    * any ROS arguments and name remapping that were provided at the command line.
    * For programmatic remappings you can use a different version of init() which takes
    * remappings directly, but for most command-line programs, passing argc and argv is
    * the easiest way to do it.  The third argument to init() is the name of the node.
    *
    * You must call one of the versions of ros::init() before using any other
    * part of the ROS system.
    */
    ros::init(argc, argv, "listener");
  
   /**
    * NodeHandle is the main access point to communications with the ROS system.
    * The first NodeHandle constructed will fully initialize this node, and the last
    * NodeHandle destructed will close down the node.
    */
  ros::NodeHandle n;
  
   /**
    * The subscribe() call is how you tell ROS that you want to receive messages
    * on a given topic.  This invokes a call to the ROS
    * master node, which keeps a registry of who is publishing and who
    * is subscribing.  Messages are passed to a callback function, here
    * called chatterCallback.  subscribe() returns a Subscriber object that you
    * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
    * object go out of scope, this callback will automatically be unsubscribed from
    * this topic.
    *
    * The second parameter to the subscribe() function is the size of the message
    * queue.  If messages are arriving faster than they are being processed, this
    * is the number of messages that will be buffered up before beginning to throw
    * away the oldest ones.
    */
   ros::Subscriber motor_interface_sub = n.subscribe<nodes::LegEncoders>("FL_encoders_feedback", 1000, encoderCallback);
   //ros::Subscriber motor_command_sub = n.subscribe("FL_command_dispatch", 1000, commandCallback);
   ros::Subscriber motor_command_sub = n.subscribe<nodes::LegCommand>("FL_command_dispatch", 1000,  commandCallback);
  
   /**
    * ros::spin() will enter a loop, pumping callbacks.  With this version, all
    * callbacks will be called from within this thread (the main one).  ros::spin()
    * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
    */
   ros::spin();
  
   return 0;
  }
