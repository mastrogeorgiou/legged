#include "ros/ros.h"
#include <signal.h>

#include <std_msgs/String.h>
#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/Float32MultiArray.h>

#include "nodes/camera_save_image.h"
#include "nodes/robot_set_time_step.h"
#include "nodes/sensor_set.h"
#include "nodes/robot_device_list.h"
#include "nodes/camera_get_info.h"
#include "nodes/motor_set_position.h"
#include "nodes/motor_get_target_position.h"

#define TIME_STEP 32;


static int controllerCount;
static std::vector<std::string> controllerList;
static std::vector<unsigned char> imageColor;
static std::vector<float> imageRangeFinder;

ros::ServiceClient timeStepClient;
nodes::robot_set_time_step timeStepSrv;

// catch names of the controllers availables on ROS network
void controllerNameCallback(const std_msgs::String::ConstPtr& name)
{
  controllerCount++;
  controllerList.push_back(name->data);
  ROS_INFO("controller #%d : %s",controllerCount,controllerList.back().c_str());
}


// get color image from the camera
void cameraColorCallback(const std_msgs::UInt8MultiArray::ConstPtr& values)
{
  int i = 0;
  imageColor.resize(values->data.size());
  for (std::vector<unsigned char>::const_iterator it = values->data.begin(); it != values->data.end(); ++it)
  {
    imageColor[i] = *it;
    i++;
  }
}

// get range image from the camera
void cameraRangeFinderCallback(const std_msgs::Float32MultiArray::ConstPtr& values)
{
  int i = 0;
  imageRangeFinder.resize(values->data.size());
  for (std::vector<float>::const_iterator it = values->data.begin(); it != values->data.end(); ++it)
  {
    imageRangeFinder[i] = *it;
    i++;
  }
}

void quit(int sig)
{
  timeStepSrv.request.step = 0;
  timeStepClient.call(timeStepSrv);
  ROS_INFO("user stopped the node catch_the_bird.");
  ros::shutdown();
  exit(0);
}


int main(int argc, char **argv)
{
  int wantedController = 0;
  std::string controllerName, motorName, cameraName;
  std::vector<std::string> deviceList;
  int width,height;
  float i,step;
  bool birdCatched = false;
  
  if (argc != 1)
  {
    ROS_INFO("usage: $ example_catch_bird");
    return 1;
  }
  
  // create a node named 'catch_the_bird' on ROS network
  ros::init(argc, argv, "catch_the_bird",ros::init_options::AnonymousName);  
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
      controllerName=controllerList[wantedController - 1];
    else
    {
      ROS_ERROR("invalid number for  controller choice");
      return 1;
    }
  }
  // leave topic once it's not necessary anymore
  nameSub.shutdown();
  
  // call device_list service to get the list of the devices available on the controller and print it
  // the device_list_srv object contains 2 members request and response. Their fields are described in the corresponding .srv file
  ros::ServiceClient deviceListClient = n.serviceClient<nodes::robot_device_list>(controllerName+"/Robot/device_list");
  nodes::robot_device_list deviceListSrv;
  
  if (deviceListClient.call(deviceListSrv))
      deviceList = deviceListSrv.response.list;
  else
    ROS_ERROR("Failed to call service device_list");
  motorName = deviceList[0];
  cameraName = deviceList[1];
  
  ros::ServiceClient cameraGetInfoClient = n.serviceClient<nodes::camera_get_info>(controllerName+'/'+cameraName+"/get_info");
  nodes::camera_get_info cameraGetInfoSrv;
  if (cameraGetInfoClient.call(cameraGetInfoSrv))
  {
    width = cameraGetInfoSrv.response.width;
    height = cameraGetInfoSrv.response.height;
    ROS_INFO("camera is of size %d x %d", width, height);
  }
  else
    ROS_ERROR("Failed to call service camera_get_info");
  
  // enable the camera
  ros::ServiceClient setCameraClient = n.serviceClient<nodes::sensor_set>(controllerName+'/'+cameraName+"/set_sensor");
  nodes::sensor_set setCameraSrv;
  ros::Subscriber subCameraColor;
  ros::Subscriber subCameraRangeFinder;
  
  setCameraSrv.request.period = 2 * TIME_STEP;
  if (setCameraClient.call(setCameraSrv) && setCameraSrv.response.success == 1)
  {
    ROS_INFO("camera enabled with sampling period %d",setCameraSrv.request.period);
    subCameraColor = n.subscribe(controllerName+'/'+cameraName+"/64_color", 1, cameraColorCallback);
    subCameraRangeFinder = n.subscribe(controllerName+'/'+cameraName+"/64_range_finder",1,cameraRangeFinderCallback);
    
    // wait for  the topics to be initialized
    while (subCameraColor.getNumPublishers() == 0);
    while (subCameraRangeFinder.getNumPublishers() == 0);
  }
  else
    ROS_ERROR("failed to call service set_sensor for  %s",cameraName.c_str());

  ros::ServiceClient cameraSaveImageClient = n.serviceClient<nodes::camera_save_image>(controllerName+'/'+cameraName+"/save_image");
  nodes::camera_save_image cameraSaveImageSrv;
  cameraSaveImageSrv.request.filename = "bird_catched.png";
  cameraSaveImageSrv.request.quality = 100;
  
  // enable motor
  ros::ServiceClient motorSetPositionClient = n.serviceClient<nodes::motor_set_position>(controllerName+'/'+motorName+"/set_position");
  nodes::motor_set_position motorSetPositionSrv;
  motorSetPositionSrv.request.position = 0;
  i = 0.2;
  step = 0.025;
  
  ros::ServiceClient motorGetTargetPositionClient = n.serviceClient<nodes::motor_get_target_position>(controllerName+'/'+motorName+"/get_target_position");
  nodes::motor_get_target_position motorGetTargetPositionSrv;
  
  // enable time_step
  timeStepClient = n.serviceClient<nodes::robot_set_time_step>(controllerName+"/Robot/time_step");
  timeStepSrv.request.step=TIME_STEP;
  
  // main loop
  while (!birdCatched && ros::ok())
  {
    motorSetPositionSrv.request.position = i;
    motorSetPositionClient.call(motorSetPositionSrv);
    if (!timeStepClient.call(timeStepSrv) || timeStepSrv.response.success != 1) {
      ROS_ERROR("failed to call next step with time_step service");
      exit(1);
    }
    motorGetTargetPositionClient.call(motorGetTargetPositionSrv);
    if (i >= 3.14)
      step = -0.025;
    if (i <= -3.14)
      step = 0.025;
    i += step;
    ros::spinOnce();
    while (imageRangeFinder.size() < (width * height))
      ros::spinOnce();
    // check if it sees the bird and take a picture of the bird
    if (imageRangeFinder[12 + (width * height / 4)] < 0.5)
    {
      birdCatched = true;
      if (cameraSaveImageClient.call(cameraSaveImageSrv) && cameraSaveImageSrv.response.success == 1)
        ROS_INFO("What a beautifull bird we found here !");
      else
        ROS_INFO("failed to call service camera_save_image to take a picture of the bird");
    }
  }
  timeStepSrv.request.step = 0;
  timeStepClient.call(timeStepSrv);
  n.shutdown();
}
