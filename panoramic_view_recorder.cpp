#include "ros/ros.h"
#include <signal.h>

#include <nodes/supervisor_set_label.h>
#include <nodes/supervisor_start_movie.h>
#include <nodes/supervisor_stop_movie.h>
#include <nodes/robot_set_time_step.h>
#include <nodes/supervisor_get_root.h>
#include <nodes/node_get_field.h>
#include <nodes/field_get_node.h>
#include <nodes/field_get_vec3f.h>
#include <nodes/field_set_vec3f.h>
#include <nodes/field_get_rotation.h>
#include <nodes/field_set_rotation.h>
#include <nodes/field_get_type_name.h>

#include <std_msgs/String.h>

#define TIME_STEP 32

static int controllerCount;
static std::vector<std::string> controllerList;

ros::ServiceClient timeStepClient;
nodes::robot_set_time_step timeStepSrv;

// catch names of the controllers available on ROS network
void controllerNameCallback(const std_msgs::String::ConstPtr& name)
{
  controllerCount++;
  controllerList.push_back(name->data);
  ROS_INFO("controller #%d : %s", controllerCount, controllerList.back().c_str());
}

void quit(int sig)
{
  timeStepSrv.request.step = 0;
  timeStepClient.call(timeStepSrv);
  ROS_INFO("user stopped the node panoramic_view_recorder.");
  ros::shutdown();
  exit(0);
}

int main(int argc, char **argv)
{
  int wantedController = 0;
  std::string controllerName;
  
  if (argc != 1)
  {
    ROS_INFO("usage: $ panoramic_view_recorder");
    return 1;
  }
  
  // create a node named 'panoramic_view_recorder' on ROS network
  ros::init(argc, argv, "panoramic_view_recorder", ros::init_options::AnonymousName);
  ros::NodeHandle n;
  
  signal(SIGINT, quit);
  
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
      controllerName = controllerList[wantedController - 1];
    else
    {
      ROS_ERROR("invalid number for controller choice");
      return 1;
    }
  }
  // leave topic once it's not necessary anymore
  nameSub.shutdown();
  
  timeStepClient = n.serviceClient<nodes::robot_set_time_step>(controllerName+"/Robot/time_step");
  timeStepSrv.request.step = TIME_STEP;
  
  ROS_INFO("Welcome to the panoramic_view_recorder example");
  ROS_INFO("This node will connect to a supervisor in Webots and access world to record a panoramic movie around a nao robot");
  
  // get world node
  ros::ServiceClient getWorldClient = n.serviceClient<nodes::supervisor_get_root>(controllerName+"/supervisor/get_root");
  nodes::supervisor_get_root getWorldSrv;
  getWorldClient.call(getWorldSrv);
  
  // get children list
  ros::ServiceClient getWorldChildrenClient = n.serviceClient<nodes::node_get_field>(controllerName+"/supervisor/node/get_field");
  nodes::node_get_field getWorldChildrenSrv;
  getWorldChildrenSrv.request.node = getWorldSrv.response.node;
  getWorldChildrenSrv.request.fieldName = "children";
  getWorldChildrenClient.call(getWorldChildrenSrv);
  
  // get spotlight node
  ros::ServiceClient getSpotlightClient = n.serviceClient<nodes::field_get_node>(controllerName+"/supervisor/field/get_node");
  nodes::field_get_node getSpotlightSrv;
  getSpotlightSrv.request.field = getWorldChildrenSrv.response.field;
  getSpotlightSrv.request.index = 4;
  getSpotlightClient.call(getSpotlightSrv);
  
  // get spotlight position and orientation
  ros::ServiceClient getSpotlightPositionFieldClient = n.serviceClient<nodes::node_get_field>(controllerName+"/supervisor/node/get_field");
  nodes::node_get_field getSpotlightPositionFieldSrv;
  getSpotlightPositionFieldSrv.request.node = getSpotlightSrv.response.node;
  getSpotlightPositionFieldSrv.request.fieldName = "location";
  getSpotlightPositionFieldClient.call(getSpotlightPositionFieldSrv);
  
  ros::ServiceClient getPositionClient = n.serviceClient<nodes::field_get_vec3f>(controllerName+"/supervisor/field/get_vec3f");
  nodes::field_get_vec3f getPositionSrv;
  getPositionSrv.request.field = getSpotlightPositionFieldSrv.response.field;
  getPositionClient.call(getPositionSrv);
  std::vector<double> spotPosition = getPositionSrv.response.value;
  
  ros::ServiceClient getSpotlightOrientationFieldClient = n.serviceClient<nodes::node_get_field>(controllerName+"/supervisor/node/get_field");
  nodes::node_get_field getSpotlightOrientationFieldSrv;
  getSpotlightOrientationFieldSrv.request.node = getSpotlightSrv.response.node;
  getSpotlightOrientationFieldSrv.request.fieldName = "direction";
  getSpotlightOrientationFieldClient.call(getSpotlightOrientationFieldSrv);
  
  ros::ServiceClient getOrientationClient = n.serviceClient<nodes::field_get_vec3f>(controllerName+"/supervisor/field/get_vec3f");
  nodes::field_get_vec3f getOrientationSrv;
  getOrientationSrv.request.field = getSpotlightOrientationFieldSrv.response.field;
  getOrientationClient.call(getOrientationSrv);
  std::vector<double> spotOrientation = getOrientationSrv.response.value;
    
  ROS_INFO("initial position of spotlight is %f %f %f and its orientation is %f %f %f", spotPosition[0], spotPosition[1], spotPosition[2], spotOrientation[0], spotOrientation[1], spotOrientation[2]);
  
  // initialize services to update spotlight position and orientation
  ros::ServiceClient setPositionClient = n.serviceClient<nodes::field_set_vec3f>(controllerName+"/supervisor/field/set_vec3f");
  nodes::field_set_vec3f setPositionSrv;
  setPositionSrv.request.field = getSpotlightPositionFieldSrv.response.field;
  setPositionSrv.request.value = spotPosition;
  
  ros::ServiceClient setOrientationClient = n.serviceClient<nodes::field_set_vec3f>(controllerName+"/supervisor/field/set_vec3f");
  nodes::field_set_vec3f setOrientationSrv;
  setOrientationSrv.request.field = getSpotlightOrientationFieldSrv.response.field;
  setOrientationSrv.request.value = spotOrientation;
  
  timeStepClient.call(timeStepSrv);
  
  // get POV(Point Of View) node
  ros::ServiceClient getPovClient = n.serviceClient<nodes::field_get_node>(controllerName+"/supervisor/field/get_node");
  nodes::field_get_node getPovSrv;
  getPovSrv.request.field = getWorldChildrenSrv.response.field;
  getPovSrv.request.index = 1;
  getPovClient.call(getPovSrv);
  
  // get POV position and orientation
  ros::ServiceClient getPovPositionFieldClient = n.serviceClient<nodes::node_get_field>(controllerName+"/supervisor/node/get_field");
  nodes::node_get_field getPovPositionFieldSrv;
  getPovPositionFieldSrv.request.node = getPovSrv.response.node;
  getPovPositionFieldSrv.request.fieldName = "position";
  getPovPositionFieldClient.call(getPovPositionFieldSrv);
  
  nodes::field_get_vec3f getPovPositionSrv;
  getPovPositionSrv.request.field = getPovPositionFieldSrv.response.field;
  getPositionClient.call(getPovPositionSrv);
  std::vector<double> povPosition = getPovPositionSrv.response.value;
  
  ros::ServiceClient getPovOrientationFieldClient = n.serviceClient<nodes::node_get_field>(controllerName+"/supervisor/node/get_field");
  nodes::node_get_field getPovOrientationFieldSrv;
  getPovOrientationFieldSrv.request.node = getPovSrv.response.node;
  getPovOrientationFieldSrv.request.fieldName = "orientation";
  getPovOrientationFieldClient.call(getPovOrientationFieldSrv);
  
  ros::ServiceClient getPovOrientationClient = n.serviceClient<nodes::field_get_rotation>(controllerName+"/supervisor/field/get_rotation");
  nodes::field_get_rotation getPovOrientationSrv;
  getPovOrientationSrv.request.field = getPovOrientationFieldSrv.response.field;
  getPovOrientationClient.call(getPovOrientationSrv);
  std::vector<double> povOrientation = getPovOrientationSrv.response.value;
  
  ROS_INFO("initial position of POV is %f %f %f and its orientation is %f %f %f %f", povPosition[0], povPosition[1], povPosition[2], povOrientation[0], povOrientation[1], povOrientation[2], povOrientation[3]);
  
  timeStepClient.call(timeStepSrv);
  
  // initialize services to update POV position and orientation
  nodes::field_set_vec3f setPovPositionSrv;
  setPovPositionSrv.request.field = getPovPositionFieldSrv.response.field;
  setPovPositionSrv.request.value = povPosition;
  
  ros::ServiceClient setPovOrientationClient = n.serviceClient<nodes::field_set_rotation>(controllerName+"/supervisor/field/set_orientation");
  nodes::field_set_rotation setPovOrientationSrv;
  setPovOrientationSrv.request.field = getPovOrientationFieldSrv.response.field;
  setPovOrientationSrv.request.value = povOrientation;
  
  //  set label to 'Recording'
  ros::ServiceClient supervisorSetLabelClient;
  nodes::supervisor_set_label supervisorSetLabelSrv;
  supervisorSetLabelClient = n.serviceClient<nodes::supervisor_set_label>(controllerName+"/supervisor/set_label");
  
  supervisorSetLabelSrv.request.id = 1;
  supervisorSetLabelSrv.request.label = "Recording";
  supervisorSetLabelSrv.request.xpos = 0.02;
  supervisorSetLabelSrv.request.ypos = 0.1;
  supervisorSetLabelSrv.request.size = 0.2;
  supervisorSetLabelSrv.request.color = 0XFF0000;
  supervisorSetLabelSrv.request.transparency = 0;
  if (!supervisorSetLabelClient.call(supervisorSetLabelSrv) || supervisorSetLabelSrv.response.success != 1)
    ROS_ERROR("failed to call service set_label");
  
  // start_movie
  ros::ServiceClient supervisorStartMovieClient;
  nodes::supervisor_start_movie supervisorStartMovieSrv;
  supervisorStartMovieClient = n.serviceClient<nodes::supervisor_start_movie>(controllerName+"/supervisor/start_movie");
  
  supervisorStartMovieSrv.request.filename = "supervisor_movie.mp4";
  supervisorStartMovieSrv.request.width = 480;
  supervisorStartMovieSrv.request.height = 360;
  supervisorStartMovieSrv.request.codec = 1337;
  supervisorStartMovieSrv.request.quality = 100;
  supervisorStartMovieSrv.request.acceleration = 1;
  supervisorStartMovieSrv.request.caption = false;
  if (supervisorStartMovieClient.call(supervisorStartMovieSrv) && supervisorStartMovieSrv.response.success == 1)
    ROS_INFO("movie started recording");
  else
    ROS_ERROR("failed to call service start_movie");
  
  while (setPovOrientationSrv.request.value[3] < 6.28) //we start at angle 0 and end at angle 2*pi
  {
    setPovOrientationSrv.request.value[3] += 0.05;
    if (setPovOrientationClient.call(setPovOrientationSrv) && setPovOrientationSrv.response.success == 1)
    {
      setPovPositionSrv.request.value[0] = povPosition[2] * sin(setPovOrientationSrv.request.value[3]);
      setPovPositionSrv.request.value[2] = povPosition[2] * cos(setPovOrientationSrv.request.value[3]);
      if (setPositionClient.call(setPovPositionSrv) && setPovPositionSrv.response.success == 1)
      {
        // we adjust the position and orientation of the spotlight to follow the camera's POV
        setOrientationSrv.request.value[0] = spotOrientation[2] * sin(setPovOrientationSrv.request.value[3]);
        setOrientationSrv.request.value[2] = spotOrientation[2] * cos(setPovOrientationSrv.request.value[3]);
        if (setOrientationClient.call(setOrientationSrv) && setOrientationSrv.response.success == 1)
        {
          setPositionSrv.request.value[0] = spotPosition[2] * sin(setPovOrientationSrv.request.value[3]);
          setPositionSrv.request.value[2] = spotPosition[2] * cos(setPovOrientationSrv.request.value[3]);
          if (setPositionClient.call(setPositionSrv) && setPositionSrv.response.success == 1)
          {
            if (!timeStepClient.call(timeStepSrv) || timeStepSrv.response.success != 1)
              ROS_ERROR("couldn't update robot step");
          }
          else
            ROS_ERROR("failed to send new spotlight position");
        }
        else
          ROS_ERROR("failed to send new spotlight orientation");
      }
      else
        ROS_ERROR("failed to send new POV position");
    }
    else
      ROS_ERROR("failed to send new POV orientation");
  }
  
  // stop_movie
  ros::ServiceClient supervisorStopMovieClient;
  nodes::supervisor_stop_movie supervisorStopMovieSrv;
  supervisorStopMovieClient = n.serviceClient<nodes::supervisor_stop_movie>(controllerName+"/supervisor/stop_movie");
  
  if (supervisorStopMovieClient.call(supervisorStopMovieSrv) && supervisorStopMovieSrv.response.success == 1)
    ROS_INFO("movie stopped recording");
  else
    ROS_ERROR("failed to call service stop_movie");
  
  supervisorSetLabelSrv.request.label = "";
  supervisorSetLabelClient.call(supervisorSetLabelSrv);
  
  timeStepSrv.request.step = 0;
  timeStepClient.call(timeStepSrv);
}
