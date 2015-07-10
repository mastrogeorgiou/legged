#include "ros/ros.h"

#include <nodes/led_get.h>
#include <nodes/led_set.h>
#include <nodes/motor_set_position.h>
#include <nodes/robot_set_time_step.h>
#include <nodes/sensor_set.h>
#include <nodes/robot_device_list.h>
#include <nodes/robot_set_keyboard.h>
#include <nodes/robot_get_key.h>

#include <std_msgs/String.h>

#include <signal.h>
#include <stdio.h>

#define TIME_STEP 32

static int controllerCount;
static std::vector<std::string> controllerList;
static std::string controllerName;
static double lposition = 0;
static double rposition = 0;


ros::ServiceClient leftWheelClient;
nodes::motor_set_position leftWheelSrv;

ros::ServiceClient rightWheelClient;
nodes::motor_set_position rightWheelSrv;

ros::ServiceClient timeStepClient;
nodes::robot_set_time_step timeStepSrv;

ros::ServiceClient setKeyboardClient;
nodes::robot_set_keyboard setKeyboardSrv;


// catch names of the controllers availables on ROS network
void controllerNameCallback(const std_msgs::String::ConstPtr& name)
{
  controllerCount++;
  controllerList.push_back(name->data);
  ROS_INFO("controller #%d : %s", controllerCount, controllerList.back().c_str());
}

void quit(int sig)
{
  setKeyboardSrv.request.period = 0;
  setKeyboardClient.call(setKeyboardSrv);
  timeStepSrv.request.step = 0;
  timeStepClient.call(timeStepSrv);
  ROS_INFO("user stopped the node keyboard_teleop.");
  ros::shutdown();
  exit(0);
}

void keyCallback(int key)
{
  int send = 0;

  switch(key)
  {
    case 314:
      lposition += -0.2;
      rposition += 0.2;
      send = 1;
      break;
    case 316:
      lposition += 0.2;
      rposition += -0.2;
      send = 1;
      break;
    case 315:
      lposition += 0.2;
      rposition += 0.2;
      send = 1;
      break;
    case 317:
      lposition += -0.2;
      rposition += -0.2;
      send = 1;
      break;
    case 312:
      ROS_INFO("END");
      quit(-1);
      break;
    default :
      send = 0;
      break;
  }

  leftWheelSrv.request.position = lposition;
  rightWheelSrv.request.position = rposition;
  if (send)
  {
    if (!leftWheelClient.call(leftWheelSrv) || !rightWheelClient.call(rightWheelSrv) || leftWheelSrv.response.success != 1 || rightWheelSrv.response.success != 1)
      ROS_ERROR("failed to send new position commands to the robot");
  }
  return;
}


int main(int argc, char **argv)
{
  int wantedController = 0;
  
  if (argc != 1)
  {
    ROS_INFO("keyboard_teleop doesn't take any arguments");
    return 1;
  }
  
  // create a node named 'keyboard_teleop' on ROS network
  ros::init(argc, argv, "keyboard_teleop",ros::init_options::AnonymousName);  
  ros::NodeHandle n;
  
  signal(SIGINT,quit);
  
  // subscribe to the topic model_name to get the list of availables controllers
  ros::Subscriber nameSub = n.subscribe("model_name", 100, controllerNameCallback);
  while (controllerCount == 0 || controllerCount < nameSub.getNumPublishers())
  {
    ros::spinOnce();
    ros::spinOnce();
    ros::spinOnce();
  }
  ros::spinOnce();
  
  // if there is more than one controller available, let the user choose
  if (controllerCount == 1)
    controllerName = controllerList[0];
  else
  {
    std::cout << "choose the # of the controller you want to use :\n";
    std::cin >> wantedController;
    if (1 <= wantedController && wantedController <= controllerCount)
      controllerName = controllerList[wantedController-1];
    else
    {
      ROS_ERROR("invalid number for controller choice");
      return 1;
    }
  }
  // leave topic once it's not necessary anymore
  nameSub.shutdown();
  
  leftWheelClient = n.serviceClient<nodes::motor_set_position>(controllerName+"/left_wheel/set_position");
  rightWheelClient = n.serviceClient<nodes::motor_set_position>(controllerName+"/right_wheel/set_position");
  timeStepClient = n.serviceClient<nodes::robot_set_time_step>(controllerName+"/Robot/time_step");
  timeStepSrv.request.step = TIME_STEP;
  
  setKeyboardClient = n.serviceClient<nodes::robot_set_keyboard>(controllerName+"/Robot/set_keyboard");
  setKeyboardSrv.request.period = TIME_STEP;
  if (setKeyboardClient.call(setKeyboardSrv) && setKeyboardSrv.response.success == 1)
  {
    ROS_INFO("keyboard enabled");
    ROS_INFO("use the arrows in Webots window to move the robot.");
    ROS_INFO("press the End key to stop the node.");
    ros::ServiceClient getKeyClient = n.serviceClient<nodes::robot_get_key>(controllerName+"/Robot/get_key");
    nodes::robot_get_key getKeySrv;
    
    // main loop
    while (ros::ok())
    {
      getKeyClient.call(getKeySrv);
      if (getKeySrv.response.key > 0)
        keyCallback(getKeySrv.response.key);
      if (!timeStepClient.call(timeStepSrv) || timeStepSrv.response.success != 1)
        ROS_ERROR("failed to call service time_step for next step");
    }
  }
  else
    ROS_ERROR("could not enable keyboard, success = %d", setKeyboardSrv.response.success);
  
  setKeyboardSrv.request.period = 0;
  if (!setKeyboardClient.call(setKeyboardSrv) || setKeyboardSrv.response.success != 0)
    ROS_ERROR("could not disable keyboard, success = %d", setKeyboardSrv.response.success);
  timeStepSrv.request.step = 0;
  timeStepClient.call(timeStepSrv);
  ros::shutdown();
  return(0);
}
