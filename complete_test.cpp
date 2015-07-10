#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/UInt8MultiArray.h>
#include <cstdlib>
#include <signal.h>

#include <nodes/camera_get_info.h>
#include <nodes/camera_save_image.h>
#include <nodes/connector_get_presence.h>
#include <nodes/connector_lock.h>
#include <nodes/connector_set_presence.h>
#include <nodes/differential_wheels_enable_control.h>
#include <nodes/differential_wheels_get_speed.h>
#include <nodes/display_draw_line.h>
#include <nodes/display_draw_oval.h>
#include <nodes/display_draw_pixel.h>
#include <nodes/display_draw_polygon.h>
#include <nodes/display_draw_rectangle.h>
#include <nodes/display_draw_text.h>
#include <nodes/display_fill_oval.h>
#include <nodes/display_fill_polygon.h>
#include <nodes/display_fill_rectangle.h>
#include <nodes/display_get_info.h>
#include <nodes/display_image_copy.h>
#include <nodes/display_image_delete.h>
#include <nodes/display_image_load.h>
#include <nodes/display_image_new.h>
#include <nodes/display_image_paste.h>
#include <nodes/display_image_save.h>
#include <nodes/display_set_alpha.h>
#include <nodes/display_set_color.h>
#include <nodes/display_set_opacity.h>
#include <nodes/emitter_get_buffer_size.h>
#include <nodes/emitter_get_channel.h>
#include <nodes/emitter_get_range.h>
#include <nodes/emitter_send.h>
#include <nodes/emitter_set_channel.h>
#include <nodes/emitter_set_range.h>
#include <nodes/field_get_bool.h>
#include <nodes/field_get_color.h>
#include <nodes/field_get_count.h>
#include <nodes/field_get_float.h>
#include <nodes/field_get_int32.h>
#include <nodes/field_get_node.h>
#include <nodes/field_get_rotation.h>
#include <nodes/field_get_string.h>
#include <nodes/field_get_type.h>
#include <nodes/field_get_type_name.h>
#include <nodes/field_get_vec2f.h>
#include <nodes/field_get_vec3f.h>
#include <nodes/field_import_node.h>
#include <nodes/field_set_bool.h>
#include <nodes/field_set_color.h>
#include <nodes/field_set_float.h>
#include <nodes/field_set_int32.h>
#include <nodes/field_set_rotation.h>
#include <nodes/field_set_string.h>
#include <nodes/field_set_vec2f.h>
#include <nodes/field_set_vec3f.h>
#include <nodes/led_get.h>
#include <nodes/led_set.h>
#include <nodes/motor_get_max_position.h>
#include <nodes/motor_get_min_position.h>
#include <nodes/motor_get_target_position.h>
#include <nodes/motor_set_acceleration.h>
#include <nodes/motor_set_available_force.h>
#include <nodes/motor_set_available_torque.h>
#include <nodes/motor_set_control_p.h>
#include <nodes/motor_set_force.h>
#include <nodes/motor_set_position.h>
#include <nodes/motor_set_torque.h>
#include <nodes/motor_set_velocity.h>
#include <nodes/node_get_center_of_mass.h>
#include <nodes/node_get_number_of_contact_points.h>
#include <nodes/node_get_contact_point.h>
#include <nodes/node_get_field.h>
#include <nodes/node_get_orientation.h>
#include <nodes/node_get_position.h>
#include <nodes/node_get_static_balance.h>
#include <nodes/node_get_type.h>
#include <nodes/node_get_type_name.h>
#include <nodes/node_reset_physics.h>
#include <nodes/pen_set_ink_color.h>
#include <nodes/pen_write.h>
#include <nodes/receiver_get_channel.h>
#include <nodes/receiver_get_data_size.h>
#include <nodes/receiver_get_emitter_direction.h>
#include <nodes/receiver_get_queue_length.h>
#include <nodes/receiver_get_signal_strength.h>
#include <nodes/receiver_next_packet.h>
#include <nodes/receiver_set_channel.h>
#include <nodes/robot_device_list.h>
#include <nodes/robot_get_basic_time_step.h>
#include <nodes/robot_get_controller_arguments.h>
#include <nodes/robot_get_controller_name.h>
#include <nodes/robot_get_data.h>
#include <nodes/robot_get_key.h>
#include <nodes/robot_get_mode.h>
#include <nodes/robot_get_model.h>
#include <nodes/robot_get_number_of_devices.h>
#include <nodes/robot_get_project_path.h>
#include <nodes/robot_get_synchronization.h>
#include <nodes/robot_get_time.h>
#include <nodes/robot_get_type.h>
#include <nodes/robot_set_data.h>
#include <nodes/robot_set_keyboard.h>
#include <nodes/robot_set_mode.h>
#include <nodes/robot_set_time_step.h>
#include <nodes/sensor_get_type.h>
#include <nodes/sensor_sampling_period.h>
#include <nodes/sensor_set.h>
#include <nodes/supervisor_export_image.h>
#include <nodes/supervisor_get_from_def.h>
#include <nodes/supervisor_get_root.h>
#include <nodes/supervisor_get_self.h>
#include <nodes/supervisor_set_label.h>
#include <nodes/supervisor_simulation_reset_physics.h>
#include <nodes/supervisor_simulation_quit.h>
#include <nodes/supervisor_simulation_revert.h>
#include <nodes/supervisor_start_movie.h>
#include <nodes/supervisor_stop_movie.h>
#include <nodes/supervisor_get_movie_status.h>

#define TIME_STEP 32

using namespace std;

static int model_count;
static vector<string> model_list;
vector<unsigned char> imageColor;
vector<float> imageRangeFinder;
double accelerometerValues[3] = {0, 0, 0};
double compassValues[3] = {0, 0, 0};
double GPSValues[3] = {0, 0, 0};
double GyroValues[3] = {0, 0, 0};
double inertialUnitValues[3] = {0, 0, 0};
double touchSensorValues[3] = {0, 0, 0};

ros::ServiceClient time_step_client;
nodes::robot_set_time_step time_step_srv;

void modelNameCallback(const std_msgs::String::ConstPtr& name)
{
  model_count++;
  model_list.push_back(name->data);
  ROS_INFO("model #%d : %s", model_count, model_list.back().c_str());
}

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

void accelerometerCallback(const std_msgs::Float64MultiArray::ConstPtr& values)
{
  int i = 0;
  for (std::vector<double>::const_iterator it = values->data.begin(); it != values->data.end(); ++it)
  {
    accelerometerValues[i] = *it;
    i++;
  }
  ROS_INFO("accelerometer values are x=%f y=%f z=%f", accelerometerValues[0], accelerometerValues[1], accelerometerValues[2]);
}

void battery_sensorCallback(const std_msgs::Float64::ConstPtr& value)
{
  ROS_INFO("battery level is %f",value->data);
}

void compassCallback(const std_msgs::Float64MultiArray::ConstPtr& values)
{
  int i = 0;
  for (std::vector<double>::const_iterator it = values->data.begin(); it != values->data.end(); ++it)
  {
    compassValues[i] = *it;
    i++;
  }
  ROS_INFO("compass values are x=%f y=%f z=%f", compassValues[0], compassValues[1], compassValues[2]);
}

void distance_sensorCallback(const std_msgs::Float64::ConstPtr& value)
{
  ROS_INFO("distance from object is %f",value->data);
}

void GPSCallback(const std_msgs::Float64MultiArray::ConstPtr& values)
{
  int i = 0;
  for (std::vector<double>::const_iterator it = values->data.begin(); it != values->data.end(); ++it)
  {
    GPSValues[i] = *it;
    i++;
  }
  ROS_INFO("GPS values are x=%f y=%f z=%f", GPSValues[0], GPSValues[1], GPSValues[2]);
}

void gyroCallback(const std_msgs::Float64MultiArray::ConstPtr& values)
{
  int i = 0;
  for (std::vector<double>::const_iterator it = values->data.begin(); it != values->data.end(); ++it)
  {
    GyroValues[i] = *it;
    i++;
  }
  ROS_INFO("Gyro values are x=%f y=%f z=%f", GyroValues[0], GyroValues[1], GyroValues[2]);
}

void inertialUnitCallback(const std_msgs::Float64MultiArray::ConstPtr& values)
{
  int i = 0;
  for (std::vector<double>::const_iterator it = values->data.begin(); it != values->data.end(); ++it)
  {
    inertialUnitValues[i] = *it;
    i++;
  }
  ROS_INFO("inertial unit values are x=%f y=%f z=%f",inertialUnitValues[0],inertialUnitValues[1],inertialUnitValues[2]);
}

void light_sensorCallback(const std_msgs::Float64::ConstPtr& value)
{
  ROS_INFO("light intensity is %f", value->data);
}

void motorSensorCallback(const std_msgs::Float64::ConstPtr& value)
{
  ROS_INFO("motor sensor sent value %f", value->data);
}

void positionSensorCallback(const std_msgs::Float64::ConstPtr& value)
{
  ROS_INFO("position sensor sent value %f", value->data);
}

void touchSensorCallback(const std_msgs::Float64::ConstPtr& value)
{
  ROS_INFO("touch sensor sent value %f", value->data);
}

void touchSensor3DCallback(const std_msgs::Float64MultiArray::ConstPtr& values)
{
  int i = 0;
  for (std::vector<double>::const_iterator it = values->data.begin(); it != values->data.end(); ++it)
  {
    touchSensorValues[i] = *it;
    i++;
  }
  ROS_INFO("touch sensor values are x = %f, y = %f and z = %f", touchSensorValues[0], touchSensorValues[1], touchSensorValues[2]);
}

void receiverCallback(const std_msgs::String::ConstPtr& value)
{
  void *message;
  message = const_cast<char *>(value->data.c_str());
  ROS_INFO("received a message");
}

void quit(int sig)
{
  time_step_srv.request.step = 0;
  time_step_client.call(time_step_srv);
  ROS_INFO("user stopped the node complete_test.");
  ros::shutdown();
  exit(0);
}

int main(int argc, char **argv)
{
  int wanted_model = 0;
  string model_name;
  
  if (argc != 1)  {
    ROS_INFO("usage: $ complete_test");
    return 1;
  }
  
  ros::init(argc, argv, "complete_test", ros::init_options::AnonymousName);
  ros::NodeHandle n;
  
  signal(SIGINT,quit);
  
  ros::Subscriber name_sub = n.subscribe("model_name", 100, modelNameCallback);
  while (model_count == 0 || model_count < name_sub.getNumPublishers()) {
    ros::spinOnce();
    ros::spinOnce();
    ros::spinOnce();
  }
  ros::spinOnce();
  
  if (model_count == 1)
    model_name = model_list[0];
  else {
    std::cout << "choose the # of the model you want to use :\n";
    std::cin >> wanted_model;
    if (1 <= wanted_model && wanted_model <= model_count)
      model_name = model_list[wanted_model-1];
    else  {
      ROS_ERROR("invalid number for model choice");
      return 1;
    }
  }
  name_sub.shutdown();
  
  ////////////////////////
  // ROBOT METHODS TEST //
  ////////////////////////
  
  bool synchronization = false;
  int number_of_devices = 0;
  int mode = 0;
  int type = 0;
  double basic_time_step = 0.0;
  double time = 0.0;
  string controller_args;
  string controller_name;
  string model;
  string project_path;
  string data;
  vector<string> device_list;

  time_step_client = n.serviceClient<nodes::robot_set_time_step>(model_name+"/Robot/time_step");
  time_step_srv.request.step = TIME_STEP;
  
  if (time_step_client.call(time_step_srv) && time_step_srv.response.success == 1)
    ROS_INFO("time_step service works");
  else
    ROS_ERROR("Failed to call service time_step to update robot's time step");

  ros::ServiceClient get_number_of_devices_client = n.serviceClient<nodes::robot_get_number_of_devices>(model_name+"/Robot/get_number_of_devices");
  nodes::robot_get_number_of_devices get_number_of_devices_srv;
  
  if (get_number_of_devices_client.call(get_number_of_devices_srv))
  {
    number_of_devices = get_number_of_devices_srv.response.number;
    ROS_INFO("%s has %d devices", model_name.c_str(), number_of_devices);
  }
  else
    ROS_ERROR("Failed to call service get_number_of_devices");

  ros::ServiceClient device_list_client = n.serviceClient<nodes::robot_device_list>(model_name+"/Robot/device_list");
  nodes::robot_device_list device_list_srv;
  
  if (device_list_client.call(device_list_srv))
  {
      device_list = device_list_srv.response.list;
      for (unsigned int i = 0; i < device_list.size(); i++)
        ROS_INFO("device [%d]: %s", i, device_list[i].c_str());
  }
  else
    ROS_ERROR("Failed to call service device_list");

  ros::ServiceClient get_basic_time_step_client = n.serviceClient<nodes::robot_get_basic_time_step>(model_name+"/Robot/get_basic_time_step");
  nodes::robot_get_basic_time_step get_basic_time_step_srv;
  
  if (get_basic_time_step_client.call(get_basic_time_step_srv))
  {
    basic_time_step = get_basic_time_step_srv.response.step;
    ROS_INFO("%s has a basic time step of %f", model_name.c_str(), basic_time_step);
  }
  else
    ROS_ERROR("Failed to call service get_basic_time_step");

  ros::ServiceClient get_controller_arguments_client = n.serviceClient<nodes::robot_get_controller_arguments>(model_name+"/Robot/get_controller_arguments");
  nodes::robot_get_controller_arguments get_controller_arguments_srv;
  
  if (get_controller_arguments_client.call(get_controller_arguments_srv))
  {
    controller_args = get_controller_arguments_srv.response.arguments;
    ROS_INFO("controller arguments of %s are %s", model_name.c_str(), controller_args.c_str());
  }
  else
    ROS_ERROR("Failed to call service get_controller_arguments");
    
  ros::ServiceClient get_controller_name_client = n.serviceClient<nodes::robot_get_controller_name>(model_name+"/Robot/get_controller_name");
  nodes::robot_get_controller_name get_controller_name_srv;
  
  if (get_controller_name_client.call(get_controller_name_srv))
  {
    controller_name = get_controller_name_srv.response.name;
    ROS_INFO("controller name of %s is %s", model_name.c_str(), controller_name.c_str());
  }
  else
    ROS_ERROR("Failed to call service get_controller_name");
    
  ros::ServiceClient robot_get_data_client = n.serviceClient<nodes::robot_get_data>(model_name+"/Robot/get_data");
  nodes::robot_get_data robot_get_data_srv;
  
  if (robot_get_data_client.call(robot_get_data_srv))
  {
    data = robot_get_data_srv.response.data;
    ROS_INFO("data of %s is %s", model_name.c_str(), data.c_str());
  }
  else
    ROS_ERROR("Failed to call service robot_get_data");

  ros::ServiceClient get_mode_client = n.serviceClient<nodes::robot_get_mode>(model_name+"/Robot/get_mode");
  nodes::robot_get_mode get_mode_srv;
  
  if (get_mode_client.call(get_mode_srv))
  {
    mode = get_mode_srv.response.mode;
    ROS_INFO("mode of %s is %d", model_name.c_str(), mode);
  }
  else
    ROS_ERROR("Failed to call service get_mode");

  ros::ServiceClient get_model_client = n.serviceClient<nodes::robot_get_model>(model_name+"/Robot/get_model");
  nodes::robot_get_model get_model_srv;
  
  if (get_model_client.call(get_model_srv))
  {
    model = get_model_srv.response.model;
    ROS_INFO("model of %s is %s", model_name.c_str(), model.c_str());
  }
  else
    ROS_ERROR("Failed to call service get_model");

  ros::ServiceClient get_project_path_client = n.serviceClient<nodes::robot_get_project_path>(model_name+"/Robot/get_project_path");
  nodes::robot_get_project_path get_project_path_srv;
  
  if (get_project_path_client.call(get_project_path_srv))
  {
    project_path = get_project_path_srv.response.path;
    ROS_INFO("project path of %s is %s", model_name.c_str(), project_path.c_str());
  }
  else
    ROS_ERROR("Failed to call service get_project_path");

  ros::ServiceClient get_synchronization_client = n.serviceClient<nodes::robot_get_synchronization>(model_name+"/Robot/get_synchronization");
  nodes::robot_get_synchronization get_synchronization_srv;
  
  if (get_synchronization_client.call(get_synchronization_srv))
  {
    synchronization = get_synchronization_srv.response.synchronization;
    if (synchronization)
      ROS_INFO("%s is sync", model_name.c_str());
  else
      ROS_INFO("%s isn't sync", model_name.c_str());
  }
  else
    ROS_ERROR("Failed to call service get_synchronization");

  ros::ServiceClient get_time_client = n.serviceClient<nodes::robot_get_time>(model_name+"/Robot/get_time");
  nodes::robot_get_time get_time_srv;
  
  if (get_time_client.call(get_time_srv))
  {
    time = get_time_srv.response.time;
    ROS_INFO("time for %s is %f", model_name.c_str(), time);
  }
  else
    ROS_ERROR("Failed to call service get_time");

  ros::ServiceClient get_type_client = n.serviceClient<nodes::robot_get_type>(model_name+"/Robot/get_type");
  nodes::robot_get_type get_type_srv;
  
  if (get_type_client.call(get_type_srv))
  {
    type = get_type_srv.response.type;
    ROS_INFO("type of %s is %d", model_name.c_str(), type);
  }
  else
    ROS_ERROR("Failed to call service get_type");
    
  ros::ServiceClient robot_set_data_client = n.serviceClient<nodes::robot_set_data>(model_name+"/Robot/set_data");
  nodes::robot_set_data robot_set_data_srv;
  
  robot_set_data_srv.request.data = "OVERWRITTEN";
  if (robot_set_data_client.call(robot_set_data_srv))
  {
    if (robot_set_data_srv.response.success == 1)
      ROS_INFO("data of %s has been set to %s", model_name.c_str(), data.c_str());
  }
  else
    ROS_ERROR("Failed to call service robot_set_data");

  ros::ServiceClient set_mode_client = n.serviceClient<nodes::robot_set_mode>(model_name+"/Robot/set_mode");
  nodes::robot_set_mode set_mode_srv;
  
  set_mode_srv.request.mode = mode;
  if (set_mode_client.call(set_mode_srv))
  {
    if (set_mode_srv.response.success == 1)
      ROS_INFO("mode of %s has been set to %d", model_name.c_str(), mode);
  }
  else
    ROS_ERROR("Failed to call service set_mode");

  ros::ServiceClient set_keyboard_client = n.serviceClient<nodes::robot_set_keyboard>(model_name+"/Robot/set_keyboard");
  nodes::robot_set_keyboard set_keyboard_srv;

  set_keyboard_srv.request.period = 32;
  if (set_keyboard_client.call(set_keyboard_srv) && set_keyboard_srv.response.success == 1) {
      ROS_INFO("keyboard of %s has been enable",model_name.c_str());
      ros::ServiceClient get_key_client = n.serviceClient<nodes::robot_get_key>(model_name+"/Robot/get_key");
      nodes::robot_get_key get_key_srv;
      if (get_key_client.call(get_key_srv))
      {
        ROS_INFO("got key %d from %s", get_key_srv.response.key, model_name.c_str());
        set_keyboard_srv.request.period = 0;
        if (set_keyboard_client.call(set_keyboard_srv) && set_keyboard_srv.response.success == 1)
          ROS_INFO("keyboard of %s has been disable", model_name.c_str());
        else
          ROS_ERROR("Failed to call service set_keyboard");
      }
      else
        ROS_ERROR("Failed to call service get_key");
  }
  else
    ROS_ERROR("Failed to call service set_keyboard");
 
  /////////////////////////
  // CAMERA METHODS TEST //
  /////////////////////////

  ros::ServiceClient get_info_client = n.serviceClient<nodes::camera_get_info>(model_name+"/camera/get_info");
  nodes::camera_get_info get_info_srv;
  
  if (get_info_client.call(get_info_srv))
    ROS_INFO("camera of %s has a width of %d, a height of %d, a field of view of %f, a near range of %f and a max range of %f", model_name.c_str(), get_info_srv.response.width, get_info_srv.response.height, get_info_srv.response.Fov, get_info_srv.response.nearRange, get_info_srv.response.maxRange);
  else
    ROS_ERROR("Failed to call service get_info");

  ros::ServiceClient camera_get_type_client = n.serviceClient<nodes::sensor_get_type>(model_name+"/camera/get_type");
  nodes::sensor_get_type camera_get_type_srv;
  
  if (camera_get_type_client.call(camera_get_type_srv))
    ROS_INFO("camera of %s is of type %d", model_name.c_str(), camera_get_type_srv.response.type);
  else
    ROS_ERROR("Failed to call service get_type");

  ros::ServiceClient set_camera_client;
  nodes::sensor_set camera_srv;
  ros::Subscriber sub_camera_color;
  ros::Subscriber sub_camera_range_finder;
  
  set_camera_client = n.serviceClient<nodes::sensor_set>(model_name+"/camera/set_sensor");
  camera_srv.request.period = TIME_STEP;
  if (set_camera_client.call(camera_srv) && camera_srv.response.success == 1)
  {
    ROS_INFO("camera enabled");
    if (camera_get_type_srv.response.type == 99 || camera_get_type_srv.response.type == 98)
    {
      sub_camera_color = n.subscribe(model_name+"/camera/32_color", 1, cameraColorCallback);
      ROS_INFO("topic for camera color initialized");
    }
    if (camera_get_type_srv.response.type == 114 || camera_get_type_srv.response.type == 98)
    {
      sub_camera_range_finder = n.subscribe(model_name+"/camera/32_range_finder", 1, cameraRangeFinderCallback);
      ROS_INFO("topic for camera range finder initialized");
    }
    while (sub_camera_color.getNumPublishers() == 0);
    ROS_INFO("topic for camera color connected");
    while (sub_camera_range_finder.getNumPublishers() == 0);
    ROS_INFO("topic for camera range finder connected");
  }
  else
  {
    if (camera_srv.response.success == 2)
      ROS_ERROR("Sampling period is not valid");
    ROS_ERROR("Failed to enable camera");
    return 1;
  }

  time_step_client.call(time_step_srv);

  ros::ServiceClient save_image_client = n.serviceClient<nodes::camera_save_image>(model_name+"/camera/save_image");
  nodes::camera_save_image save_image_srv;
  save_image_srv.request.filename = "test_image_camera.png";
  save_image_srv.request.quality = 100;
  
  if (save_image_client.call(save_image_srv) && save_image_srv.response.success == 1)
    ROS_INFO("Image saved");
  else
    ROS_ERROR("Failed to call service save_image");
  
  time_step_client.call(time_step_srv);

  sub_camera_color.shutdown();
  sub_camera_range_finder.shutdown();
  ROS_INFO("camera disabled");
 
  ////////////////////////////////
  // ACCELEROMETER METHODS TEST //
  ////////////////////////////////

  ros::ServiceClient set_accelerometer_client;
  nodes::sensor_set accelerometer_srv;
  ros::Subscriber sub_accelerometer_32;
  ros::Subscriber sub_accelerometer_64;
    
  set_accelerometer_client = n.serviceClient<nodes::sensor_set>(model_name+"/accelerometer/set_sensor");
  
  accelerometer_srv.request.period = 64;
  if (set_accelerometer_client.call(accelerometer_srv) && accelerometer_srv.response.success == 1)
  {
    sub_accelerometer_64 = n.subscribe(model_name+"/accelerometer/64", 1, accelerometerCallback);
    while (sub_accelerometer_64.getNumPublishers() == 0);
  }
  else
  {
    if (accelerometer_srv.response.success == 2)
      ROS_ERROR("Sampling period is not valid");
    ROS_ERROR("Failed to enable accelerometer");
    return 1;
  }
  
  ros::ServiceClient sampling_period_accelerometer_client;
  nodes::sensor_sampling_period sampling_period_accelerometer_srv;
  
  sampling_period_accelerometer_client = n.serviceClient<nodes::sensor_sampling_period>(model_name+"/accelerometer/sampling_period");
  sampling_period_accelerometer_client.call(sampling_period_accelerometer_srv);
  ROS_INFO("accelerometer is enable with a sampling period of %d",sampling_period_accelerometer_srv.response.period);
  
  accelerometer_srv.request.period = 32;
  if (set_accelerometer_client.call(accelerometer_srv) && accelerometer_srv.response.success == 1)
  {
    sub_accelerometer_32 = n.subscribe(model_name+"/accelerometer/32", 1, accelerometerCallback);
    while (sub_accelerometer_32.getNumPublishers() == 0);
  }
  else
  {
    if (accelerometer_srv.response.success == 2)
      ROS_ERROR("Sampling period is not valid");
    ROS_ERROR("Failed to enable accelerometer");
    return 1;
  }
  
  sampling_period_accelerometer_client.call(sampling_period_accelerometer_srv);
  ROS_INFO("accelerometer is enable with a sampling period of %d", sampling_period_accelerometer_srv.response.period);

  sub_accelerometer_32.shutdown();
  sub_accelerometer_64.shutdown();
  
  // wait for webots to detect shutdown of topics and to disable the sensor afterwards
  time_step_client.call(time_step_srv);
  time_step_client.call(time_step_srv);
  time_step_client.call(time_step_srv);
  
  sampling_period_accelerometer_client.call(sampling_period_accelerometer_srv);
  ROS_INFO("accelerometer is disabled (sampling period is %d)", sampling_period_accelerometer_srv.response.period);
  
  time_step_client.call(time_step_srv);
  time_step_client.call(time_step_srv);
  
  /////////////////////////////////
  // BATTERY SENSOR METHODS TEST //
  /////////////////////////////////

  ros::ServiceClient set_battery_sensor_client;
  nodes::sensor_set battery_sensor_srv;
  ros::Subscriber sub_battery_sensor_32;
  set_battery_sensor_client = n.serviceClient<nodes::sensor_set>(model_name+"/battery_sensor/set_sensor");
  
  ros::ServiceClient sampling_period_battery_sensor_client;
  nodes::sensor_sampling_period sampling_period_battery_sensor_srv;
  sampling_period_battery_sensor_client = n.serviceClient<nodes::sensor_sampling_period>(model_name+"/battery_sensor/sampling_period");
  
  battery_sensor_srv.request.period = 32;
  if (set_battery_sensor_client.call(battery_sensor_srv) && battery_sensor_srv.response.success == 1)
  {
    ROS_INFO("battery_sensor enabled");
    sub_battery_sensor_32 = n.subscribe(model_name+"/battery_sensor/32", 1, battery_sensorCallback);
    while (sub_battery_sensor_32.getNumPublishers() == 0);
  }
  else
  {
    if (battery_sensor_srv.response.success == 2)
      ROS_ERROR("Sampling period is not valid");
    ROS_ERROR("Failed to enable battery_sensor");
    return 1;
  }
  
  time_step_client.call(time_step_srv);
  
  sampling_period_battery_sensor_client.call(sampling_period_battery_sensor_srv);
  ROS_INFO("battery_sensor is enable with a sampling period of %d", sampling_period_battery_sensor_srv.response.period);
  
  time_step_client.call(time_step_srv);
  
  sub_battery_sensor_32.shutdown();

  time_step_client.call(time_step_srv);
  time_step_client.call(time_step_srv);
  time_step_client.call(time_step_srv);
  
  sampling_period_battery_sensor_client.call(sampling_period_battery_sensor_srv);
  ROS_INFO("battery_sensor is disabled (sampling period is %d)", sampling_period_battery_sensor_srv.response.period);
  
  //////////////////////////
  // COMPASS METHODS TEST //
  //////////////////////////

  ros::ServiceClient set_compass_client;
  nodes::sensor_set compass_srv;
  ros::Subscriber sub_compass_32;
  set_compass_client = n.serviceClient<nodes::sensor_set>(model_name+"/compass/set_sensor");
  
  ros::ServiceClient sampling_period_compass_client;
  nodes::sensor_sampling_period sampling_period_compass_srv;
  sampling_period_compass_client = n.serviceClient<nodes::sensor_sampling_period>(model_name+"/compass/sampling_period");
  
  compass_srv.request.period = 32;
  if (set_compass_client.call(compass_srv) && compass_srv.response.success == 1)
  {
    ROS_INFO("compass enabled");
    sub_compass_32 = n.subscribe(model_name+"/compass/32", 1, compassCallback);
    while (sub_compass_32.getNumPublishers() == 0);
  }
  else
  {
    if (compass_srv.response.success == 2)
      ROS_ERROR("Sampling period is not valid");
    ROS_ERROR("Failed to enable compass");
    return 1;
  }
  
  time_step_client.call(time_step_srv);
  
  sampling_period_compass_client.call(sampling_period_compass_srv);
  ROS_INFO("compass is enable with a sampling period of %d", sampling_period_compass_srv.response.period);
  
  time_step_client.call(time_step_srv);

  sub_compass_32.shutdown();

  time_step_client.call(time_step_srv);
  time_step_client.call(time_step_srv);
  time_step_client.call(time_step_srv);
  
  sampling_period_compass_client.call(sampling_period_compass_srv);
  ROS_INFO("compass is disabled (sampling period is %d)", sampling_period_compass_srv.response.period);
  
  ////////////////////////////
  // CONNECTOR METHODS TEST //
  ////////////////////////////

  ros::ServiceClient connector_set_presence_client;
  nodes::connector_set_presence connector_srv;
  connector_set_presence_client = n.serviceClient<nodes::connector_set_presence>(model_name+"/connector/set_presence");
  
  connector_srv.request.period = 32;
  if (connector_set_presence_client.call(connector_srv) && connector_srv.response.success == 1)
    ROS_INFO("connector's presence sensor enabled");
  else
  {
    if (connector_srv.response.success == 2)
      ROS_ERROR("Sampling period is not valid");
    ROS_ERROR("Failed to enable connector's presence sensor");
    return 1;
  }
  
  ros::ServiceClient connector_get_presence_client;
  nodes::connector_get_presence connector_presence_srv;
  connector_get_presence_client = n.serviceClient<nodes::connector_get_presence>(model_name+"/connector/get_presence");
  
  connector_get_presence_client.call(connector_presence_srv);
  ROS_INFO("connector's presence is %d", connector_presence_srv.response.presence);
  
  time_step_client.call(time_step_srv);
  time_step_client.call(time_step_srv);
  time_step_client.call(time_step_srv);

  connector_srv.request.period = 0;
  if (connector_set_presence_client.call(connector_srv) && connector_srv.response.success == 0)
    ROS_INFO("connector's presence sensor disabled");
  else
  {
    if (connector_srv.response.success == 2)
      ROS_ERROR("Sampling period is not valid");
    ROS_ERROR("Failed to disable connector's presence sensor");
    return 1;
  }
  
  ros::ServiceClient connector_lock_client;
  nodes::connector_lock connector_lock_srv;
  connector_lock_client = n.serviceClient<nodes::connector_lock>(model_name+"/connector/lock");
  
  connector_lock_srv.request.lock = 1;
  if (connector_lock_client.call(connector_lock_srv) && connector_lock_srv.response.success == 1)
    ROS_INFO("connector has been locked");
  else
    ROS_INFO("Failed to lock connector");
  
  //////////////////////////
  // DISPLAY METHODS TEST //
  //////////////////////////

  ros::ServiceClient display_get_info_client;
  nodes::display_get_info display_get_info_srv;
  display_get_info_client = n.serviceClient<nodes::display_get_info>(model_name+"/display/get_info");
  
  display_get_info_client.call(display_get_info_srv);
  ROS_INFO("display's width is %d and its height is %d", display_get_info_srv.response.width, display_get_info_srv.response.height);

  ros::ServiceClient display_set_color_client;
  nodes::display_set_color display_set_color_srv;
  display_set_color_client = n.serviceClient<nodes::display_set_color>(model_name+"/display/set_color");
  
  display_set_color_srv.request.color = 0xFF0000;
  if (display_set_color_client.call(display_set_color_srv) && display_set_color_srv.response.success == 1)
    ROS_INFO("display's color has been updated");
  else
    ROS_ERROR("Failed to call service display_set_color. success = %d", display_set_color_srv.response.success);

  ros::ServiceClient display_set_alpha_client;
  nodes::display_set_alpha display_set_alpha_srv;
  display_set_alpha_client = n.serviceClient<nodes::display_set_alpha>(model_name+"/display/set_alpha");
  
  display_set_alpha_srv.request.alpha = 1.0;
  if (display_set_alpha_client.call(display_set_alpha_srv) && display_set_alpha_srv.response.success == 1)
    ROS_INFO("display's alpha has been updated");
  else
    ROS_ERROR("Failed to call service display_set_alpha. success = %d", display_set_alpha_srv.response.success);

  ros::ServiceClient display_set_opacity_client;
  nodes::display_set_opacity display_set_opacity_srv;
  display_set_opacity_client = n.serviceClient<nodes::display_set_opacity>(model_name+"/display/set_opacity");
  
  display_set_opacity_srv.request.opacity = 1.0;
  if (display_set_opacity_client.call(display_set_opacity_srv) && display_set_opacity_srv.response.success == 1)
    ROS_INFO("display's opacity has been updated");
  else
    ROS_ERROR("Failed to call service display_set_opacity. success = %d", display_set_opacity_srv.response.success);

  ros::ServiceClient display_draw_pixel_client;
  nodes::display_draw_pixel display_draw_pixel_srv;
  display_draw_pixel_client = n.serviceClient<nodes::display_draw_pixel>(model_name+"/display/draw_pixel");
  
  display_draw_pixel_srv.request.x1 = 10;
  display_draw_pixel_srv.request.y1 = 10;
  if (display_draw_pixel_client.call(display_draw_pixel_srv) && display_draw_pixel_srv.response.success == 1)
    ROS_INFO("pixel drawn at x =32 and y = 32 on the display");
  else
    ROS_ERROR("Failed to call service display_draw_pixel. success = %d", display_draw_pixel_srv.response.success);

  ros::ServiceClient display_draw_line_client;
  nodes::display_draw_line display_draw_line_srv;
  display_draw_line_client = n.serviceClient<nodes::display_draw_line>(model_name+"/display/draw_line");
  
  display_draw_line_srv.request.x1 = 32;
  display_draw_line_srv.request.x2 = 63;
  display_draw_line_srv.request.y1 = 32;
  display_draw_line_srv.request.y2 = 42;
  if (display_draw_line_client.call(display_draw_line_srv) && display_draw_line_srv.response.success == 1)
    ROS_INFO("line drawn at x =32 and y = 32 on the display");
  else
    ROS_ERROR("Failed to call service display_draw_line. success = %d", display_draw_line_srv.response.success);

  ros::ServiceClient display_draw_rectangle_client;
  nodes::display_draw_rectangle display_draw_rectangle_srv;
  display_draw_rectangle_client = n.serviceClient<nodes::display_draw_rectangle>(model_name+"/display/draw_rectangle");
  
  display_draw_rectangle_srv.request.x = 2;
  display_draw_rectangle_srv.request.y = 32;
  display_draw_rectangle_srv.request.width = 10;
  display_draw_rectangle_srv.request.height = 5;
  if (display_draw_rectangle_client.call(display_draw_rectangle_srv) && display_draw_rectangle_srv.response.success == 1)
    ROS_INFO("rectangle drawn at x =32 and y = 32 with width = 10 and height = 5 on the display");
  else
    ROS_ERROR("Failed to call service display_draw_rectangle. success = %d", display_draw_rectangle_srv.response.success);

  ros::ServiceClient display_draw_oval_client;
  nodes::display_draw_oval display_draw_oval_srv;
  display_draw_oval_client = n.serviceClient<nodes::display_draw_oval>(model_name+"/display/draw_oval");
  
  display_draw_oval_srv.request.cx = 32;
  display_draw_oval_srv.request.cy = 6;
  display_draw_oval_srv.request.a = 10;
  display_draw_oval_srv.request.b = 5;
  
  if (display_draw_oval_client.call(display_draw_oval_srv) && display_draw_oval_srv.response.success == 1)
    ROS_INFO("oval drawn at x =32 and y = 6 and axes a = 10 and b = 5 on the display");
  else
    ROS_ERROR("Failed to call service display_draw_oval. success = %d", display_draw_oval_srv.response.success);

  ros::ServiceClient display_draw_polygon_client;
  nodes::display_draw_polygon display_draw_polygon_srv;
  display_draw_polygon_client = n.serviceClient<nodes::display_draw_polygon>(model_name+"/display/draw_polygon");
  
  display_draw_polygon_srv.request.x.push_back(55);
  display_draw_polygon_srv.request.y.push_back(55);
  display_draw_polygon_srv.request.x.push_back(50);
  display_draw_polygon_srv.request.y.push_back(50);
  display_draw_polygon_srv.request.x.push_back(45);
  display_draw_polygon_srv.request.y.push_back(45);
  display_draw_polygon_srv.request.x.push_back(45);
  display_draw_polygon_srv.request.y.push_back(55);
  display_draw_polygon_srv.request.x.push_back(40);
  display_draw_polygon_srv.request.y.push_back(50);
  display_draw_polygon_srv.request.size = 5;
  if (display_draw_polygon_client.call(display_draw_polygon_srv) && display_draw_polygon_srv.response.success == 1)
    ROS_INFO("polygon drawn on the display");
  else
    ROS_ERROR("Failed to call service display_draw_polygon. success = %d",display_draw_polygon_srv.response.success);

  ros::ServiceClient display_draw_text_client;
  nodes::display_draw_text display_draw_text_srv;
  display_draw_text_client = n.serviceClient<nodes::display_draw_text>(model_name+"/display/draw_text");
  
  display_draw_text_srv.request.x = 10;
  display_draw_text_srv.request.y = 52;
  display_draw_text_srv.request.text = "hello world";
  if (display_draw_text_client.call(display_draw_text_srv) && display_draw_text_srv.response.success == 1)
    ROS_INFO("Hello World written at x =10 and y = 52 on the display");
  else
    ROS_ERROR("Failed to call service display_draw_text. success = %d", display_draw_text_srv.response.success);

  ros::ServiceClient display_fill_rectangle_client;
  nodes::display_fill_rectangle display_fill_rectangle_srv;
  display_fill_rectangle_client = n.serviceClient<nodes::display_fill_rectangle>(model_name+"/display/fill_rectangle");
  
  display_fill_rectangle_srv.request.x = 2;
  display_fill_rectangle_srv.request.y = 32;
  display_fill_rectangle_srv.request.width = 10;
  display_fill_rectangle_srv.request.height = 5;
  if (display_fill_rectangle_client.call(display_fill_rectangle_srv) && display_fill_rectangle_srv.response.success == 1)
    ROS_INFO("rectangle filled at x =32 and y = 32 with width = 10 and height = 5 on the display");
  else
    ROS_ERROR("Failed to call service display_fill_rectangle. success = %d", display_fill_rectangle_srv.response.success);

  ros::ServiceClient display_fill_oval_client;
  nodes::display_fill_oval display_fill_oval_srv;
  display_fill_oval_client = n.serviceClient<nodes::display_fill_oval>(model_name+"/display/fill_oval");
  
  display_fill_oval_srv.request.cx = 32;
  display_fill_oval_srv.request.cy = 6;
  display_fill_oval_srv.request.a = 10;
  display_fill_oval_srv.request.b = 5;
  
  if (display_fill_oval_client.call(display_fill_oval_srv) && display_fill_oval_srv.response.success == 1)
    ROS_INFO("oval filled at x =32 and y = 6 and axes a = 10 and b = 5 on the display");
  else
    ROS_ERROR("Failed to call service display_fill_oval. success = %d", display_fill_oval_srv.response.success);

  ros::ServiceClient display_fill_polygon_client;
  nodes::display_fill_polygon display_fill_polygon_srv;
  display_fill_polygon_client = n.serviceClient<nodes::display_fill_polygon>(model_name+"/display/fill_polygon");
  
  display_fill_polygon_srv.request.x.push_back(55);
  display_fill_polygon_srv.request.y.push_back(55);
  display_fill_polygon_srv.request.x.push_back(50);
  display_fill_polygon_srv.request.y.push_back(50);
  display_fill_polygon_srv.request.x.push_back(45);
  display_fill_polygon_srv.request.y.push_back(45);
  display_fill_polygon_srv.request.x.push_back(45);
  display_fill_polygon_srv.request.y.push_back(55);
  display_fill_polygon_srv.request.x.push_back(40);
  display_fill_polygon_srv.request.y.push_back(50);
  display_fill_polygon_srv.request.size = 5;
  if (display_fill_polygon_client.call(display_fill_polygon_srv) && display_fill_polygon_srv.response.success == 1)
    ROS_INFO("polygon filled on the display");
  else
    ROS_ERROR("Failed to call service display_fill_polygon. success = %d", display_fill_polygon_srv.response.success);

  time_step_client.call(time_step_srv);
  time_step_client.call(time_step_srv);
  time_step_client.call(time_step_srv);

  ros::ServiceClient display_image_new_client;
  nodes::display_image_new display_image_new_srv;
  display_image_new_client = n.serviceClient<nodes::display_image_new>(model_name+"/display/image_new");
  
  display_image_new_srv.request.format = 3;
  display_image_new_srv.request.width = 10;
  display_image_new_srv.request.height = 5;
  display_image_new_srv.request.data.push_back(1);
  display_image_new_srv.request.data.push_back(2);
  display_image_new_srv.request.data.push_back(5);
  display_image_new_srv.request.data.push_back(3);
  display_image_new_srv.request.data.push_back(4);
  display_image_new_client.call(display_image_new_srv);
  ROS_INFO("new image created");
  uint64_t new_image = display_image_new_srv.response.ir;

  ros::ServiceClient display_image_copy_client;
  nodes::display_image_copy display_image_copy_srv;
  display_image_copy_client = n.serviceClient<nodes::display_image_copy>(model_name+"/display/image_copy");
  
  display_image_copy_srv.request.x = 0;
  display_image_copy_srv.request.y = 32;
  display_image_copy_srv.request.width = 64;
  display_image_copy_srv.request.height = 32;
  display_image_copy_client.call(display_image_copy_srv);
  ROS_INFO("image copied");
  uint64_t copy_image = display_image_copy_srv.response.ir;

  ros::ServiceClient display_image_paste_client;
  nodes::display_image_paste display_image_paste_srv;
  display_image_paste_client = n.serviceClient<nodes::display_image_paste>(model_name+"/display/image_paste");
  
  display_image_paste_srv.request.ir = copy_image;
  display_image_paste_srv.request.x = 0;
  display_image_paste_srv.request.y = 0;
  if (display_image_paste_client.call(display_image_paste_srv) && display_image_paste_srv.response.success == 1)
    ROS_INFO("image successfully copy/paste");
  else
    ROS_ERROR("Failed to call service display_image_paste to paste image");
  
  time_step_client.call(time_step_srv);
  time_step_client.call(time_step_srv);
  time_step_client.call(time_step_srv);

  ros::ServiceClient display_image_save_client;
  nodes::display_image_save display_image_save_srv;
  display_image_save_client = n.serviceClient<nodes::display_image_save>(model_name+"/display/image_save");
  
  display_image_save_srv.request.ir = copy_image;
  display_image_save_srv.request.filename = "copy_image.png";
  if (display_image_save_client.call(display_image_save_srv) && display_image_save_srv.response.success == 1)
    ROS_INFO("image successfully saved");
  else
    ROS_ERROR("Failed to call service display_image_save to save image");

  ros::ServiceClient display_image_load_client;
  nodes::display_image_load display_image_load_srv;
  display_image_load_client = n.serviceClient<nodes::display_image_load>(model_name+"/display/image_load");
  
  display_image_load_srv.request.filename = "test_image_camera.png";
  display_image_load_client.call(display_image_load_srv);
  ROS_INFO("image successfully loaded to clipboard");
  uint64_t loaded_image = display_image_load_srv.response.ir;
  
  display_image_paste_srv.request.ir = loaded_image;
  if (display_image_paste_client.call(display_image_paste_srv) && display_image_paste_srv.response.success == 1)
    ROS_INFO("image successfully load and paste");
  else
    ROS_ERROR("Failed to call service display_image_paste to paste image");   

  ros::ServiceClient display_image_delete_client;
  nodes::display_image_delete display_image_delete_srv;
  display_image_delete_client = n.serviceClient<nodes::display_image_delete>(model_name+"/display/image_delete");
  
  display_image_delete_srv.request.ir = loaded_image;
  if (display_image_delete_client.call(display_image_delete_srv) && display_image_delete_srv.response.success == 1)
    ROS_INFO("loaded image has been deleted");
  else
    ROS_ERROR("Failed to call service display_image_delete");
  
  display_image_delete_srv.request.ir = copy_image;
  if (display_image_delete_client.call(display_image_delete_srv) && display_image_delete_srv.response.success == 1)
    ROS_INFO("copy image has been deleted");
  else
    ROS_ERROR("Failed to call service display_image_delete");

  display_image_delete_srv.request.ir = new_image;
  if (display_image_delete_client.call(display_image_delete_srv) && display_image_delete_srv.response.success == 1)
    ROS_INFO("new image has been deleted");
  else
    ROS_ERROR("Failed to call service display_image_delete");
  
  //////////////////////////////////
  // DISTANCE SENSOR METHODS TEST //
  //////////////////////////////////

  ros::ServiceClient set_distance_sensor_client;
  nodes::sensor_set distance_sensor_srv;
  ros::Subscriber sub_distance_sensor_32;
  set_distance_sensor_client = n.serviceClient<nodes::sensor_set>(model_name+"/distance_sensor/set_sensor");
  
  ros::ServiceClient sampling_period_distance_sensor_client;
  nodes::sensor_sampling_period sampling_period_distance_sensor_srv;
  sampling_period_distance_sensor_client = n.serviceClient<nodes::sensor_sampling_period>(model_name+"/distance_sensor/sampling_period");
  
  distance_sensor_srv.request.period = 32;
  if (set_distance_sensor_client.call(distance_sensor_srv) && distance_sensor_srv.response.success == 1)
  {
    ROS_INFO("distance_sensor enabled");
    sub_distance_sensor_32 = n.subscribe(model_name+"/distance_sensor/32", 1, distance_sensorCallback);
    while (sub_distance_sensor_32.getNumPublishers() == 0);
  }
  else
  {
    if (distance_sensor_srv.response.success == 2)
      ROS_ERROR("Sampling period is not valid");
    ROS_ERROR("Failed to enable distance_sensor");
    return 1;
  }
  
  time_step_client.call(time_step_srv);
  
  sampling_period_distance_sensor_client.call(sampling_period_distance_sensor_srv);
  ROS_INFO("distance_sensor is enable with a sampling period of %d", sampling_period_distance_sensor_srv.response.period);
  
  time_step_client.call(time_step_srv);

  sub_distance_sensor_32.shutdown();

  time_step_client.call(time_step_srv);
  time_step_client.call(time_step_srv);
  time_step_client.call(time_step_srv);
  
  sampling_period_distance_sensor_client.call(sampling_period_distance_sensor_srv);
  ROS_INFO("distance_sensor is disabled (sampling period is %d)", sampling_period_distance_sensor_srv.response.period);
  
  //////////////////////////
    // EMITTER METHODS TEST //
    //////////////////////////

  ros::ServiceClient emitter_send_client;
  nodes::emitter_send emitter_send_srv;
  emitter_send_client = n.serviceClient<nodes::emitter_send>(model_name+"/emitter/send");
  
  emitter_send_srv.request.size = 3;
  emitter_send_srv.request.data.push_back('a');
  emitter_send_srv.request.data.push_back('b');
  emitter_send_srv.request.data.push_back('c');
  
  if (emitter_send_client.call(emitter_send_srv) && emitter_send_srv.response.success == 1)
    ROS_INFO("emitter has sent his 3 datas");
  else
    ROS_ERROR("Failed to call service emitter_send to send datas");

  ros::ServiceClient emitter_get_buffer_size_client;
  nodes::emitter_get_buffer_size emitter_get_buffer_size_srv;
  emitter_get_buffer_size_client = n.serviceClient<nodes::emitter_get_buffer_size>(model_name+"/emitter/get_buffer_size");
  
  emitter_get_buffer_size_client.call(emitter_get_buffer_size_srv);
  ROS_INFO("emitter's buffer is of size %d", emitter_get_buffer_size_srv.response.bufferSize);

  ros::ServiceClient emitter_set_channel_client;
  nodes::emitter_set_channel emitter_set_channel_srv;
  emitter_set_channel_client = n.serviceClient<nodes::emitter_set_channel>(model_name+"/emitter/set_channel");
  
  emitter_set_channel_srv.request.channel = 1;
  
  if (emitter_set_channel_client.call(emitter_set_channel_srv) && emitter_set_channel_srv.response.success == 1)
    ROS_INFO("emitter has update the channel");
  else
    ROS_ERROR("Failed to call service emitter_set_channel to update channel");

  ros::ServiceClient emitter_set_range_client;
  nodes::emitter_set_range emitter_set_range_srv;
  emitter_set_range_client = n.serviceClient<nodes::emitter_set_range>(model_name+"/emitter/set_range");
  
  emitter_set_range_srv.request.range = 50;
  
  if (emitter_set_range_client.call(emitter_set_range_srv) && emitter_set_range_srv.response.success == 1)
    ROS_INFO("emitter has update the range");
  else
    ROS_ERROR("Failed to call service emitter_set_range to update range");

  ros::ServiceClient emitter_get_channel_client;
  nodes::emitter_get_channel emitter_get_channel_srv;
  emitter_get_channel_client = n.serviceClient<nodes::emitter_get_channel>(model_name+"/emitter/get_channel");
  
  emitter_get_channel_client.call(emitter_get_channel_srv);
  ROS_INFO("emitter uses channel %d", emitter_get_channel_srv.response.channel);

  ros::ServiceClient emitter_get_range_client;
  nodes::emitter_get_range emitter_get_range_srv;
  emitter_get_range_client = n.serviceClient<nodes::emitter_get_range>(model_name+"/emitter/get_range");
  
  emitter_get_range_client.call(emitter_get_range_srv);
  ROS_INFO("emitter has a range of %f", emitter_get_range_srv.response.range);
  
  //////////////////////
  // GPS METHODS TEST //
  //////////////////////

  ros::ServiceClient set_GPS_client;
  nodes::sensor_set GPS_srv;
  ros::Subscriber sub_GPS_32;
  set_GPS_client = n.serviceClient<nodes::sensor_set>(model_name+"/gps/set_sensor");
  
  ros::ServiceClient sampling_period_GPS_client;
  nodes::sensor_sampling_period sampling_period_GPS_srv;
  sampling_period_GPS_client = n.serviceClient<nodes::sensor_sampling_period>(model_name+"/gps/sampling_period");
  
  GPS_srv.request.period = 32;
  if (set_GPS_client.call(GPS_srv) && GPS_srv.response.success == 1)
  {
    ROS_INFO("GPS enabled");
    sub_GPS_32 = n.subscribe(model_name+"/gps/32", 1, GPSCallback);
    while (sub_GPS_32.getNumPublishers() == 0);
  }
  else
  {
    if (GPS_srv.response.success == 2)
      ROS_ERROR("Sampling period is not valid");
    ROS_ERROR("Failed to enable GPS");
    return 1;
  }
  
  time_step_client.call(time_step_srv);
  
  sampling_period_GPS_client.call(sampling_period_GPS_srv);
  ROS_INFO("GPS is enable with a sampling period of %d", sampling_period_GPS_srv.response.period);
  
  time_step_client.call(time_step_srv);

  sub_GPS_32.shutdown();

  time_step_client.call(time_step_srv);
  time_step_client.call(time_step_srv);
  time_step_client.call(time_step_srv);
  
  sampling_period_GPS_client.call(sampling_period_GPS_srv);
  ROS_INFO("GPS is disabled (sampling period is %d)", sampling_period_GPS_srv.response.period);

  ///////////////////////
  // GYRO METHODS TEST //
  ///////////////////////

  ros::ServiceClient set_gyro_client;
  nodes::sensor_set gyro_srv;
  ros::Subscriber sub_gyro_32;
  set_gyro_client = n.serviceClient<nodes::sensor_set>(model_name+"/gyro/set_sensor");
  
  ros::ServiceClient sampling_period_gyro_client;
  nodes::sensor_sampling_period sampling_period_gyro_srv;
  sampling_period_gyro_client = n.serviceClient<nodes::sensor_sampling_period>(model_name+"/gyro/sampling_period");
  
  gyro_srv.request.period = 32;
  if (set_gyro_client.call(gyro_srv) && gyro_srv.response.success == 1)
  {
    ROS_INFO("gyro enabled");
    sub_gyro_32 = n.subscribe(model_name+"/gyro/32", 1, gyroCallback);
    while (sub_gyro_32.getNumPublishers() == 0);
  }
  else
  {
    if (gyro_srv.response.success == 2)
      ROS_ERROR("Sampling period is not valid");
    ROS_ERROR("Failed to enable gyro");
    return 1;
  }
  
  time_step_client.call(time_step_srv);
  
  sampling_period_gyro_client.call(sampling_period_gyro_srv);
  ROS_INFO("gyro is enable with a sampling period of %d", sampling_period_gyro_srv.response.period);
  
  time_step_client.call(time_step_srv);

  sub_gyro_32.shutdown();

  time_step_client.call(time_step_srv);
  time_step_client.call(time_step_srv);
  time_step_client.call(time_step_srv);
  
  sampling_period_gyro_client.call(sampling_period_gyro_srv);
  ROS_INFO("gyro is disabled (sampling period is %d)", sampling_period_gyro_srv.response.period);
  
  ////////////////////////////////
  // INERTIAL_UNIT METHODS TEST //
  ////////////////////////////////

  ros::ServiceClient set_inertial_unit_client;
  nodes::sensor_set inertial_unit_srv;
  ros::Subscriber sub_inertial_unit_32;
  set_inertial_unit_client = n.serviceClient<nodes::sensor_set>(model_name+"/inertial_unit/set_sensor");
  
  ros::ServiceClient sampling_period_inertial_unit_client;
  nodes::sensor_sampling_period sampling_period_inertial_unit_srv;
  sampling_period_inertial_unit_client = n.serviceClient<nodes::sensor_sampling_period>(model_name+"/inertial_unit/sampling_period");
  
  inertial_unit_srv.request.period = 32;
  if (set_inertial_unit_client.call(inertial_unit_srv) && inertial_unit_srv.response.success == 1)
  {
    ROS_INFO("inertial_unit enabled");
    sub_inertial_unit_32 = n.subscribe(model_name+"/inertial_unit/32", 1, inertialUnitCallback);
    while (sub_inertial_unit_32.getNumPublishers() == 0);
  }
  else
  {
    if (inertial_unit_srv.response.success == 2)
      ROS_ERROR("Sampling period is not valid");
    ROS_ERROR("Failed to enable inertial_unit");
    return 1;
  }
  
  time_step_client.call(time_step_srv);
  
  sampling_period_inertial_unit_client.call(sampling_period_inertial_unit_srv);
  ROS_INFO("inertial_unit is enable with a sampling period of %d", sampling_period_inertial_unit_srv.response.period);
  
  time_step_client.call(time_step_srv);

  sub_inertial_unit_32.shutdown();

  time_step_client.call(time_step_srv);
  time_step_client.call(time_step_srv);
  time_step_client.call(time_step_srv);
  
  sampling_period_inertial_unit_client.call(sampling_period_inertial_unit_srv);
  ROS_INFO("inertial_unit is disabled (sampling period is %d)", sampling_period_inertial_unit_srv.response.period);

  //////////////////////
  // LED METHODS TEST //
  //////////////////////
  
  ros::ServiceClient set_led_client;
  nodes::led_set set_led_srv;
  set_led_client = n.serviceClient<nodes::led_set>(model_name+"/led/set_led");
  
  set_led_srv.request.value = 1;
  if (set_led_client.call(set_led_srv) && set_led_srv.response.success == 1)
    ROS_INFO("LED set to 1");
  else
    ROS_ERROR("Failed to call service set_led");
  
  ros::ServiceClient get_led_client;
  nodes::led_get get_led_srv;
  get_led_client = n.serviceClient<nodes::led_get>(model_name+"/led/get_led");
  
  get_led_client.call(get_led_srv);
  ROS_INFO("LED value is %d", get_led_srv.response.value);
  
  set_led_srv.request.value = 0;
  if (set_led_client.call(set_led_srv) && set_led_srv.response.success == 1)
    ROS_INFO("LED set to 0");
  else
    ROS_ERROR("Failed to call service set_led");
  
  ///////////////////////////////
  // LIGHT SENSOR METHODS TEST //
  ///////////////////////////////

  ros::ServiceClient set_light_sensor_client;
  nodes::sensor_set light_sensor_srv;
  ros::Subscriber sub_light_sensor_32;
  set_light_sensor_client = n.serviceClient<nodes::sensor_set>(model_name+"/light_sensor/set_sensor");
  
  ros::ServiceClient sampling_period_light_sensor_client;
  nodes::sensor_sampling_period sampling_period_light_sensor_srv;
  sampling_period_light_sensor_client = n.serviceClient<nodes::sensor_sampling_period>(model_name+"/light_sensor/sampling_period");
  
  light_sensor_srv.request.period = 32;
  if (set_light_sensor_client.call(light_sensor_srv) && light_sensor_srv.response.success == 1)
  {
    ROS_INFO("light_sensor enabled");
    sub_light_sensor_32 = n.subscribe(model_name+"/light_sensor/32", 1, light_sensorCallback);
    while (sub_light_sensor_32.getNumPublishers() == 0);
  }
  else
  {
    if (light_sensor_srv.response.success == 2)
      ROS_ERROR("Sampling period is not valid");
    ROS_ERROR("Failed to enable light_sensor");
    return 1;
  }
  
  time_step_client.call(time_step_srv);
  
  sampling_period_light_sensor_client.call(sampling_period_light_sensor_srv);
  ROS_INFO("light_sensor is enable with a sampling period of %d", sampling_period_light_sensor_srv.response.period);
  
  time_step_client.call(time_step_srv);

  sub_light_sensor_32.shutdown();

  time_step_client.call(time_step_srv);
  time_step_client.call(time_step_srv);
  time_step_client.call(time_step_srv);
  
  sampling_period_light_sensor_client.call(sampling_period_light_sensor_srv);
  ROS_INFO("light_sensor is disabled (sampling period is %d)", sampling_period_light_sensor_srv.response.period);
  
  ////////////////////////
  // MOTOR METHODS TEST //
  ////////////////////////

  ros::ServiceClient motor_get_type_client;
  nodes::robot_get_type motor_get_type_srv;
  motor_get_type_client = n.serviceClient<nodes::robot_get_type>(model_name+"/rotational_motor/get_type");
  motor_get_type_client.call(motor_get_type_srv);
  ROS_INFO("rotational_motor is of type %d", motor_get_type_srv.response.type);
  
  ros::ServiceClient linear_motor_get_type_client;
  nodes::robot_get_type linear_motor_get_type_srv;
  linear_motor_get_type_client = n.serviceClient<nodes::robot_get_type>(model_name+"/linear_motor/get_type");
  linear_motor_get_type_client.call(linear_motor_get_type_srv);
  ROS_INFO("linear_motor is of type %d", linear_motor_get_type_srv.response.type);

  ros::ServiceClient set_acceleration_client;
  nodes::motor_set_acceleration set_acceleration_srv;
  set_acceleration_client = n.serviceClient<nodes::motor_set_acceleration>(model_name+"/rotational_motor/set_acceleration");
  
  set_acceleration_srv.request.acceleration = 0.5;
  if (set_acceleration_client.call(set_acceleration_srv) && set_acceleration_srv.response.success == 1)
    ROS_INFO("acceleration set to 0.5");
  else
    ROS_ERROR("Failed to call service set_acceleration on motor");
  
  time_step_client.call(time_step_srv);

  ros::ServiceClient set_velocity_client;
  nodes::motor_set_velocity set_velocity_srv;
  set_velocity_client = n.serviceClient<nodes::motor_set_velocity>(model_name+"/rotational_motor/set_velocity");
  
  set_velocity_srv.request.velocity = 0.9;
  if (set_velocity_client.call(set_velocity_srv) && set_velocity_srv.response.success == 1)
    ROS_INFO("velocity set to 0.9");
  else
    ROS_ERROR("Failed to call service set_velocity on motor");
  
  time_step_client.call(time_step_srv);

  ros::ServiceClient set_force_client;
  nodes::motor_set_force set_force_srv;
  set_force_client = n.serviceClient<nodes::motor_set_force>(model_name+"/linear_motor/set_force");
  
  set_force_srv.request.force = 0.2;
  if (set_force_client.call(set_force_srv) && set_force_srv.response.success == 1)
    ROS_INFO("force set to 0.2");
  else
    ROS_ERROR("Failed to call service set_force on motor");
  
  time_step_client.call(time_step_srv);

  ros::ServiceClient set_torque_client;
  nodes::motor_set_torque set_torque_srv;
  set_torque_client = n.serviceClient<nodes::motor_set_torque>(model_name+"/rotational_motor/set_torque");
  
  set_torque_srv.request.torque = 0.5;
  if (set_torque_client.call(set_torque_srv) && set_torque_srv.response.success == 1)
    ROS_INFO("torque set to 0.5");
  else
    ROS_ERROR("Failed to call service set_torque on motor");
  
  time_step_client.call(time_step_srv);

  ros::ServiceClient set_available_force_client;
  nodes::motor_set_available_force set_available_force_srv;
  set_available_force_client = n.serviceClient<nodes::motor_set_available_force>(model_name+"/linear_motor/set_available_force");
  
  set_available_force_srv.request.force = 0.8;
  if (set_available_force_client.call(set_available_force_srv) && set_available_force_srv.response.success == 1)
    ROS_INFO("available_force set to 0.8");
  else
    ROS_ERROR("Failed to call service set_available_force on motor");
  
  time_step_client.call(time_step_srv);
  
  ros::ServiceClient set_available_torque_client;
  nodes::motor_set_available_torque set_available_torque_srv;
  set_available_torque_client = n.serviceClient<nodes::motor_set_available_torque>(model_name+"/rotational_motor/set_available_torque");
  
  set_available_torque_srv.request.torque = 0.8;
  if (set_available_torque_client.call(set_available_torque_srv) && set_available_torque_srv.response.success == 1)
    ROS_INFO("available_torque set to 0.8");
  else
    ROS_ERROR("Failed to call service set_available_torque on motor");
  
  time_step_client.call(time_step_srv);

  ros::ServiceClient set_controlp_client;
  nodes::motor_set_control_p set_controlp_srv;
  set_controlp_client = n.serviceClient<nodes::motor_set_control_p>(model_name+"/rotational_motor/set_controlp");
  
  set_controlp_srv.request.controlp = 1;
  if (set_controlp_client.call(set_controlp_srv) && set_controlp_srv.response.success == 1)
    ROS_INFO("controlp set to 1");
  else
    ROS_ERROR("Failed to call service set_controlp on motor");
  
  time_step_client.call(time_step_srv);
  time_step_client.call(time_step_srv);
  
  ros::ServiceClient set_linear_controlp_client;
  nodes::motor_set_control_p set_linear_controlp_srv;
  set_linear_controlp_client = n.serviceClient<nodes::motor_set_control_p>(model_name+"/linear_motor/set_controlp");
  
  set_linear_controlp_srv.request.controlp = 1;
  if (set_linear_controlp_client.call(set_linear_controlp_srv) && set_linear_controlp_srv.response.success == 1)
    ROS_INFO("controlp set to 1 for linear_motor");
  else
    ROS_ERROR("Failed to call service set_controlp on linear_motor");
  
  time_step_client.call(time_step_srv);
  time_step_client.call(time_step_srv);

  ros::ServiceClient set_position_client;
  nodes::motor_set_position set_position_srv;
  set_position_client = n.serviceClient<nodes::motor_set_position>(model_name+"/rotational_motor/set_position");
  
  set_position_srv.request.position = 1.5;
  if (set_position_client.call(set_position_srv) && set_position_srv.response.success == 1)
    ROS_INFO("position set to 1.5");
  else
    ROS_ERROR("Failed to call service set_position on motor");
  
  time_step_client.call(time_step_srv);
  time_step_client.call(time_step_srv);
  
  ros::ServiceClient set_linear_position_client;
  nodes::motor_set_position set_linear_position_srv;
  set_linear_position_client = n.serviceClient<nodes::motor_set_position>(model_name+"/linear_motor/set_position");
  
  set_linear_position_srv.request.position = 0.3;
  if (set_linear_position_client.call(set_linear_position_srv) && set_linear_position_srv.response.success == 1)
    ROS_INFO("position set to 0.3 for linear_motor");
  else
    ROS_ERROR("Failed to call service set_position on linear_motor");

  ros::ServiceClient get_target_position_client;
  nodes::motor_get_target_position get_target_position_srv;
  get_target_position_client = n.serviceClient<nodes::motor_get_target_position>(model_name+"/rotational_motor/get_target_position");
  
  get_target_position_client.call(get_target_position_srv);
  ROS_INFO("target position for rotational_motor is %f", get_target_position_srv.response.position);
  
  time_step_client.call(time_step_srv);

  ros::ServiceClient get_min_position_client;
  nodes::motor_get_min_position get_min_position_srv;
  get_min_position_client = n.serviceClient<nodes::motor_get_min_position>(model_name+"/rotational_motor/get_min_position");
  
  get_min_position_client.call(get_min_position_srv);
  ROS_INFO("min position for rotational_motor is %f", get_min_position_srv.response.position);
  
  time_step_client.call(time_step_srv);

  ros::ServiceClient get_max_position_client;
  nodes::motor_get_max_position get_max_position_srv;
  get_max_position_client = n.serviceClient<nodes::motor_get_max_position>(model_name+"/rotational_motor/get_max_position");
  
  get_max_position_client.call(get_max_position_srv);
  ROS_INFO("max position for rotational_motor is %f", get_max_position_srv.response.position);
  
  time_step_client.call(time_step_srv);
  time_step_client.call(time_step_srv);
  
  //////////////////////
  // PEN METHODS TEST //
  //////////////////////

  ros::ServiceClient set_ink_color_client;
  nodes::pen_set_ink_color set_ink_color_srv;
  set_ink_color_client = n.serviceClient<nodes::pen_set_ink_color>(model_name+"/pen/set_ink_color");
  
  set_ink_color_srv.request.color = 0x00FF08;
  if (set_ink_color_client.call(set_ink_color_srv) && set_ink_color_srv.response.success == 1)
    ROS_INFO("ink color set to turquoise");
  else
    ROS_ERROR("Failed to call service set_ink_color");
  
  ros::ServiceClient pen_write_client;
  nodes::pen_write pen_write_srv;
  pen_write_client = n.serviceClient<nodes::pen_write>(model_name+"/pen/write");
  
  pen_write_srv.request.write = true;
  if (pen_write_client.call(pen_write_srv) && pen_write_srv.response.success == 1)
    ROS_INFO("pen is now writing");
  else
    ROS_ERROR("Failed to call service pen_write");
  
  time_step_client.call(time_step_srv);
  time_step_client.call(time_step_srv);
  
  //////////////////////////////////
  // POSITION SENSOR METHODS TEST //
  //////////////////////////////////

  ros::ServiceClient set_position_sensor_client;
  nodes::sensor_set position_sensor_srv;
  ros::Subscriber sub_position_sensor_32;
  set_position_sensor_client = n.serviceClient<nodes::sensor_set>(model_name+"/position_sensor/set_sensor");
  
  ros::ServiceClient sampling_period_position_sensor_client;
  nodes::sensor_sampling_period sampling_period_position_sensor_srv;
  sampling_period_position_sensor_client = n.serviceClient<nodes::sensor_sampling_period>(model_name+"/position_sensor/sampling_period");
  
  position_sensor_srv.request.period = 32;
  if (set_position_sensor_client.call(position_sensor_srv) && position_sensor_srv.response.success == 1)
  {
    ROS_INFO("position_sensor enabled");
    sub_position_sensor_32 = n.subscribe(model_name+"/position_sensor/32", 1, positionSensorCallback);
    while (sub_position_sensor_32.getNumPublishers() == 0);
  }
  else
  {
    if (position_sensor_srv.response.success == 2)
      ROS_ERROR("Sampling period is not valid");
    ROS_ERROR("Failed to enable position_sensor");
    return 1;
  }
  
  time_step_client.call(time_step_srv);
  
  sampling_period_position_sensor_client.call(sampling_period_position_sensor_srv);
  ROS_INFO("position_sensor is enable with a sampling period of %d", sampling_period_position_sensor_srv.response.period);
  
  time_step_client.call(time_step_srv);
  
  ros::ServiceClient position_sensor_get_type_client;
  nodes::sensor_get_type position_sensor_get_type_srv;
  position_sensor_get_type_client = n.serviceClient<nodes::sensor_get_type>(model_name+"/position_sensor/get_type");
  
  position_sensor_get_type_client.call(position_sensor_get_type_srv);
  ROS_INFO("position_sensor is of type %d", position_sensor_get_type_srv.response.type);
  
  time_step_client.call(time_step_srv);

  sub_position_sensor_32.shutdown();

  time_step_client.call(time_step_srv);
  time_step_client.call(time_step_srv);
  time_step_client.call(time_step_srv);
  
  sampling_period_position_sensor_client.call(sampling_period_position_sensor_srv);
  ROS_INFO("position_sensor is disabled (sampling period is %d)", sampling_period_position_sensor_srv.response.period);
  
  ///////////////////////////
  // RECEIVER METHODS TEST //
  ///////////////////////////

  ros::ServiceClient set_receiver_client;
  nodes::sensor_set receiver_srv;
  ros::Subscriber sub_receiver_32;
  set_receiver_client = n.serviceClient<nodes::sensor_set>(model_name+"/receiver/set_sensor");
  
  ros::ServiceClient sampling_period_receiver_client;
  nodes::sensor_sampling_period sampling_period_receiver_srv;
  sampling_period_receiver_client = n.serviceClient<nodes::sensor_sampling_period>(model_name+"/receiver/sampling_period");
  
  receiver_srv.request.period = 32;
  if (set_receiver_client.call(receiver_srv) && receiver_srv.response.success == 1)
  {
    ROS_INFO("receiver enabled");
    sub_receiver_32 = n.subscribe(model_name+"/receiver/32", 1, receiverCallback);
    while (sub_receiver_32.getNumPublishers() == 0);
  }
  else
  {
    if (receiver_srv.response.success == 2)
      ROS_ERROR("Sampling period is not valid");
    ROS_ERROR("Failed to enable receiver");
    return 1;
  }
  
  time_step_client.call(time_step_srv);
  
  sampling_period_receiver_client.call(sampling_period_receiver_srv);
  ROS_INFO("receiver is enable with a sampling period of %d", sampling_period_receiver_srv.response.period);
  
  time_step_client.call(time_step_srv);
  time_step_client.call(time_step_srv);
  
  // test receiver_get_data_size
  // An error message will probably appear since no data has been sent to the receiver.
  ros::ServiceClient receiver_get_data_size_client;
  nodes::receiver_get_data_size receiver_get_data_size_srv;
  receiver_get_data_size_client = n.serviceClient<nodes::receiver_get_data_size>(model_name+"/receiver/get_data_size");
  
  receiver_get_data_size_client.call(receiver_get_data_size_srv);
  if (receiver_get_data_size_srv.response.size != -1)
    ROS_INFO("emitter's buffer is of size %d", receiver_get_data_size_srv.response.size);
  else
    ROS_ERROR("illegal call to Receiver::getDataSize()");
  
  time_step_client.call(time_step_srv);
  time_step_client.call(time_step_srv);

  ros::ServiceClient receiver_set_channel_client;
  nodes::receiver_set_channel receiver_set_channel_srv;
  receiver_set_channel_client = n.serviceClient<nodes::receiver_set_channel>(model_name+"/receiver/set_channel");
  
  receiver_set_channel_srv.request.channel = 1;
  
  if (receiver_set_channel_client.call(receiver_set_channel_srv) && receiver_set_channel_srv.response.success == 1)
    ROS_INFO("receiver has update the channel");
  else
    ROS_ERROR("Failed to call service receiver_set_channel to update channel");
  
  time_step_client.call(time_step_srv);
  
  // test receiver_get_channel
  ros::ServiceClient receiver_get_channel_client;
  nodes::receiver_get_channel receiver_get_channel_srv;
  receiver_get_channel_client = n.serviceClient<nodes::receiver_get_channel>(model_name+"/receiver/get_channel");
  
  receiver_get_channel_client.call(receiver_get_channel_srv);
  ROS_INFO("receiver uses channel %d", receiver_get_channel_srv.response.channel);
  
  time_step_client.call(time_step_srv);
  
  // test receiver_get_queue_length
  ros::ServiceClient receiver_get_queue_length_client;
  nodes::receiver_get_queue_length receiver_get_queue_length_srv;
  receiver_get_queue_length_client = n.serviceClient<nodes::receiver_get_queue_length>(model_name+"/receiver/get_queue_length");
  
  receiver_get_queue_length_client.call(receiver_get_queue_length_srv);
  ROS_INFO("length of receiver queue is %d", receiver_get_queue_length_srv.response.length);
  
  time_step_client.call(time_step_srv);
  
  // test receiver_get_signal_strength
    // An error message will probably appear since no signal has been sent to the receiver.
  ros::ServiceClient receiver_get_signal_strength_client;
  nodes::receiver_get_signal_strength receiver_get_signal_strength_srv;
  receiver_get_signal_strength_client = n.serviceClient<nodes::receiver_get_signal_strength>(model_name+"/receiver/get_signal_strength");
  
  receiver_get_signal_strength_client.call(receiver_get_signal_strength_srv);
  if (receiver_get_signal_strength_srv.response.strength != -1)
    ROS_INFO("strength of the signal is %d", receiver_get_signal_strength_srv.response.strength);
  else
    ROS_ERROR("illegal call to Receiver::get_signal_strength()");
  
  time_step_client.call(time_step_srv);
  
  // test receiver_get_emitter_direction
  // An error message will probably appear since no signal has been sent to the receiver
  ros::ServiceClient receiver_get_emitter_direction_client;
  nodes::receiver_get_emitter_direction receiver_get_emitter_direction_srv;
  receiver_get_emitter_direction_client = n.serviceClient<nodes::receiver_get_emitter_direction>(model_name+"/receiver/get_emitter_direction");
  
  receiver_get_emitter_direction_client.call(receiver_get_emitter_direction_srv);
  if (receiver_get_emitter_direction_srv.response.direction[0] != 0 || receiver_get_emitter_direction_srv.response.direction[1] != 0 || receiver_get_emitter_direction_srv.response.direction[2] !=0)
    ROS_INFO("signal from emitter comes from direction {%f. %f, %f}", receiver_get_emitter_direction_srv.response.direction[0], receiver_get_emitter_direction_srv.response.direction[1], receiver_get_emitter_direction_srv.response.direction[2]);
  else
    ROS_ERROR("illegal call to Receiver::getEmitterDirection()");
  
  time_step_client.call(time_step_srv);
  
  // test receiver_next_packet
  // An error message will probably appear since there is no packet to read
  ros::ServiceClient receiver_next_packet_client;
  nodes::receiver_next_packet receiver_next_packet_srv;
  receiver_next_packet_client = n.serviceClient<nodes::receiver_next_packet>(model_name+"/receiver/next_packet");
  
  if (receiver_next_packet_client.call(receiver_next_packet_srv) && receiver_next_packet_srv.response.success == 1)
    ROS_INFO("next packet is ready to be read");
  else if (receiver_next_packet_srv.response.success == -1)
    ROS_ERROR("Illegeal call to Receiver::next_packet()");
  else
    ROS_ERROR("Failed to call service receiver_next_packet");
  
  time_step_client.call(time_step_srv);

  sub_receiver_32.shutdown();
  
  time_step_client.call(time_step_srv);
  time_step_client.call(time_step_srv);
  time_step_client.call(time_step_srv);
  
  sampling_period_receiver_client.call(sampling_period_receiver_srv);
  ROS_INFO("receiver is disabled (sampling period is %d)", sampling_period_receiver_srv.response.period);
  
  ///////////////////////////////
  // TOUCH SENSOR METHODS TEST //
  ///////////////////////////////

  ros::ServiceClient set_touch_sensor_client;
  nodes::sensor_set touch_sensor_srv;
  ros::Subscriber sub_touch_sensor_32;
  set_touch_sensor_client = n.serviceClient<nodes::sensor_set>(model_name+"/touch_sensor/set_sensor");
  
  ros::ServiceClient sampling_period_touch_sensor_client;
  nodes::sensor_sampling_period sampling_period_touch_sensor_srv;
  sampling_period_touch_sensor_client = n.serviceClient<nodes::sensor_sampling_period>(model_name+"/touch_sensor/sampling_period");
  
  ros::ServiceClient touch_sensor_get_type_client;
  nodes::sensor_get_type touch_sensor_get_type_srv;
  touch_sensor_get_type_client = n.serviceClient<nodes::sensor_get_type>(model_name+"/touch_sensor/get_type");
  
  touch_sensor_get_type_client.call(touch_sensor_get_type_srv);
  ROS_INFO("touch_sensor is of type %d", touch_sensor_get_type_srv.response.type);
  
  time_step_client.call(time_step_srv);
  
  touch_sensor_srv.request.period = 32;
  if (set_touch_sensor_client.call(touch_sensor_srv) && touch_sensor_srv.response.success == 1) {
    ROS_INFO("touch_sensor enabled");
    if (touch_sensor_get_type_srv.response.type == 0)
      sub_touch_sensor_32 = n.subscribe(model_name+"/touch_sensor/32_bumper", 1, touchSensorCallback);
    else if (touch_sensor_get_type_srv.response.type == 1)
      sub_touch_sensor_32 = n.subscribe(model_name+"/touch_sensor/32_force", 1, touchSensorCallback);
    else
      sub_touch_sensor_32 = n.subscribe(model_name+"/touch_sensor/32_force3D", 1, touchSensor3DCallback);
    while (sub_touch_sensor_32.getNumPublishers() == 0);
  }
  else  {
    if (touch_sensor_srv.response.success == 2)
      ROS_ERROR("Sampling period is not valid");
    ROS_ERROR("Failed to enable touch_sensor");
    return 1;
  }
  
  time_step_client.call(time_step_srv);
  
  sampling_period_touch_sensor_client.call(sampling_period_touch_sensor_srv);
  ROS_INFO("touch_sensor is enable with a sampling period of %d", sampling_period_touch_sensor_srv.response.period);
  
  time_step_client.call(time_step_srv);
  
  time_step_client.call(time_step_srv);
  
  sub_touch_sensor_32.shutdown();

  time_step_client.call(time_step_srv);
  time_step_client.call(time_step_srv);
  time_step_client.call(time_step_srv);
  
  sampling_period_touch_sensor_client.call(sampling_period_touch_sensor_srv);
  ROS_INFO("touch_sensor is disabled (sampling period is %d)", sampling_period_touch_sensor_srv.response.period);
  
  /////////////////////////////
  // SUPERVISOR METHODS TEST //
  /////////////////////////////

  ros::ServiceClient supervisor_simulation_reset_physics_client;
  nodes::supervisor_simulation_reset_physics supervisor_simulation_reset_physics_srv;
  supervisor_simulation_reset_physics_client = n.serviceClient<nodes::supervisor_simulation_reset_physics>(model_name+"/supervisor/simulation_reset_physics");
  
  if (supervisor_simulation_reset_physics_client.call(supervisor_simulation_reset_physics_srv) && supervisor_simulation_reset_physics_srv.response.success == 1)
    ROS_INFO("simulation has reset_physics successfully");
  else
    ROS_ERROR("Failed to call service simulation_reset_physics");
  
  ros::ServiceClient supervisor_export_image_client;
  nodes::supervisor_export_image supervisor_export_image_srv;
  supervisor_export_image_client = n.serviceClient<nodes::supervisor_export_image>(model_name+"/supervisor/export_image");
  
  supervisor_export_image_srv.request.filename = "main_window_test.jpg";
  supervisor_export_image_srv.request.quality = 100;
  if (supervisor_export_image_client.call(supervisor_export_image_srv) && supervisor_export_image_srv.response.success == 1)
    ROS_INFO("image from main window saved successfully");
  else
    ROS_ERROR("Failed to call service export_image");
  
  ros::ServiceClient supervisor_set_label_client;
  nodes::supervisor_set_label supervisor_set_label_srv;
  supervisor_set_label_client = n.serviceClient<nodes::supervisor_set_label>(model_name+"/supervisor/set_label");
  
  supervisor_set_label_srv.request.id = 1;
  supervisor_set_label_srv.request.label = "This is a label";
  supervisor_set_label_srv.request.xpos = 0.02;
  supervisor_set_label_srv.request.ypos = 0.2;
  supervisor_set_label_srv.request.size = 0.1;
  supervisor_set_label_srv.request.color = 0XFF0000;
  supervisor_set_label_srv.request.transparency = 0;
  if (supervisor_set_label_client.call(supervisor_set_label_srv) && supervisor_set_label_srv.response.success == 1)
    ROS_INFO("label set successfully");
  else
    ROS_ERROR("failed to call service set_label");
  
  time_step_client.call(time_step_srv);
  
  ros::ServiceClient supervisor_get_root_client;
  nodes::supervisor_get_root supervisor_get_root_srv;
  supervisor_get_root_client = n.serviceClient<nodes::supervisor_get_root>(model_name+"/supervisor/get_root");
  
  supervisor_get_root_client.call(supervisor_get_root_srv);
  ROS_INFO("got root node : %ld",supervisor_get_root_srv.response.node);
  uint64_t root_node = supervisor_get_root_srv.response.node;
  
  ros::ServiceClient supervisor_get_self_client;
  nodes::supervisor_get_self supervisor_get_self_srv;
  supervisor_get_self_client = n.serviceClient<nodes::supervisor_get_self>(model_name+"/supervisor/get_self");
  
  supervisor_get_self_client.call(supervisor_get_self_srv);
  ROS_INFO("got self node : %ld", supervisor_get_self_srv.response.node);
  uint64_t self_node = supervisor_get_self_srv.response.node;

  ros::ServiceClient supervisor_get_from_def_client;
  nodes::supervisor_get_from_def supervisor_get_from_def_srv;
  supervisor_get_from_def_client = n.serviceClient<nodes::supervisor_get_from_def>(model_name+"/supervisor/get_from_def");
  
  supervisor_get_from_def_srv.request.name = "TEST";
  supervisor_get_from_def_client.call(supervisor_get_from_def_srv);
  uint64_t from_def_node;
  if (supervisor_get_from_def_srv.response.node != 0)
  {
    ROS_INFO("got from_def node : %ld", supervisor_get_from_def_srv.response.node);
    from_def_node = supervisor_get_from_def_srv.response.node;  
  }
  else
    ROS_ERROR("could not get node from def");
  
  ros::ServiceClient supervisor_node_get_type_client;
  nodes::node_get_type supervisor_node_get_type_srv;
  supervisor_node_get_type_client = n.serviceClient<nodes::node_get_type>(model_name+"/supervisor/node/get_type");
  
  supervisor_node_get_type_srv.request.node = from_def_node;
  supervisor_node_get_type_client.call(supervisor_node_get_type_srv);
  ROS_INFO("got type of from_def_node : %d", supervisor_node_get_type_srv.response.type);
  
  ros::ServiceClient supervisor_node_get_type_name_client;
  nodes::node_get_type_name supervisor_node_get_type_name_srv;
  supervisor_node_get_type_name_client = n.serviceClient<nodes::node_get_type_name>(model_name+"/supervisor/node/get_type_name");
  
  supervisor_node_get_type_name_srv.request.node = from_def_node;
  supervisor_node_get_type_name_client.call(supervisor_node_get_type_name_srv);
  ROS_INFO("got type name of from_def_node : %s", supervisor_node_get_type_name_srv.response.name.c_str());
  
  supervisor_node_get_type_name_srv.request.node = root_node;
  supervisor_node_get_type_name_client.call(supervisor_node_get_type_name_srv);
  ROS_INFO("got type name of root_node : %s", supervisor_node_get_type_name_srv.response.name.c_str());

  supervisor_node_get_type_name_srv.request.node = self_node;
  supervisor_node_get_type_name_client.call(supervisor_node_get_type_name_srv);
  ROS_INFO("got type name of self_node : %s", supervisor_node_get_type_name_srv.response.name.c_str());
  
  ros::ServiceClient supervisor_node_get_position_client;
  nodes::node_get_position supervisor_node_get_position_srv;
  supervisor_node_get_position_client = n.serviceClient<nodes::node_get_position>(model_name+"/supervisor/node/get_position");
  
  supervisor_node_get_position_srv.request.node = from_def_node;
  supervisor_node_get_position_client.call(supervisor_node_get_position_srv);
  ROS_INFO("from_def node got position : x = %f y = %f z = %f", supervisor_node_get_position_srv.response.position[0], supervisor_node_get_position_srv.response.position[1], supervisor_node_get_position_srv.response.position[2]);
  
  ros::ServiceClient supervisor_node_get_orientation_client;
  nodes::node_get_orientation supervisor_node_get_orientation_srv;
  supervisor_node_get_orientation_client = n.serviceClient<nodes::node_get_orientation>(model_name+"/supervisor/node/get_orientation");
  
  supervisor_node_get_orientation_srv.request.node = from_def_node;
  supervisor_node_get_orientation_client.call(supervisor_node_get_orientation_srv);
    ROS_INFO("from_def orientation matrix is :\n%f %f %f\n%f %f %f\n%f %f %f", supervisor_node_get_orientation_srv.response.orientation[0], supervisor_node_get_orientation_srv.response.orientation[1], supervisor_node_get_orientation_srv.response.orientation[2], supervisor_node_get_orientation_srv.response.orientation[3], supervisor_node_get_orientation_srv.response.orientation[4], supervisor_node_get_orientation_srv.response.orientation[5], supervisor_node_get_orientation_srv.response.orientation[6], supervisor_node_get_orientation_srv.response.orientation[7], supervisor_node_get_orientation_srv.response.orientation[8]);
  
  ros::ServiceClient supervisor_node_get_center_of_mass_client;
  nodes::node_get_center_of_mass supervisor_node_get_center_of_mass_srv;
  supervisor_node_get_center_of_mass_client = n.serviceClient<nodes::node_get_center_of_mass>(model_name+"/supervisor/node/get_center_of_mass");
  
  supervisor_node_get_center_of_mass_srv.request.node = from_def_node;
  supervisor_node_get_center_of_mass_client.call(supervisor_node_get_center_of_mass_srv);
  ROS_INFO("from_def node's center of mass coordinates are : x = %f y = %f z = %f", supervisor_node_get_center_of_mass_srv.response.centerOfMass[0], supervisor_node_get_center_of_mass_srv.response.centerOfMass[1], supervisor_node_get_center_of_mass_srv.response.centerOfMass[2]);

  time_step_client.call(time_step_srv);

  ros::ServiceClient supervisor_node_get_number_of_contact_points_client;
  nodes::node_get_number_of_contact_points supervisor_node_get_number_of_contact_points_srv;
  supervisor_node_get_number_of_contact_points_client = n.serviceClient<nodes::node_get_number_of_contact_points>(model_name+"/supervisor/node/get_number_of_contact_points");
  
  supervisor_node_get_number_of_contact_points_srv.request.node = from_def_node;
  supervisor_node_get_number_of_contact_points_client.call(supervisor_node_get_number_of_contact_points_srv);
  ROS_INFO("from_def node got %d contact points",supervisor_node_get_number_of_contact_points_srv.response.numberOfContactPoints);
  
  time_step_client.call(time_step_srv);
  
  ros::ServiceClient supervisor_node_get_contact_point_client;
  nodes::node_get_contact_point supervisor_node_get_contact_point_srv;
  supervisor_node_get_contact_point_client = n.serviceClient<nodes::node_get_contact_point>(model_name+"/supervisor/node/get_contact_point");
  
  supervisor_node_get_contact_point_srv.request.node = from_def_node;
  supervisor_node_get_contact_point_srv.request.index = 0;
  supervisor_node_get_contact_point_client.call(supervisor_node_get_contact_point_srv);
  ROS_INFO("from_def_node first contact point is at x = %f, y = %f z = %f", supervisor_node_get_contact_point_srv.response.point[0], supervisor_node_get_contact_point_srv.response.point[1], supervisor_node_get_contact_point_srv.response.point[2]);
  
  // test get_static_balance
  // if the node isn't a top Solid webots will throw a warning but still return true to ros
  ros::ServiceClient supervisor_node_get_static_balance_client;
  nodes::node_get_static_balance supervisor_node_get_static_balance_srv;
  supervisor_node_get_static_balance_client = n.serviceClient<nodes::node_get_static_balance>(model_name+"/supervisor/node/get_static_balance");
  
  supervisor_node_get_static_balance_srv.request.node = from_def_node;
  supervisor_node_get_static_balance_client.call(supervisor_node_get_static_balance_srv);
  ROS_INFO("from_def node balance is %d",supervisor_node_get_static_balance_srv.response.balance);
  
  time_step_client.call(time_step_srv);
  
  // test reset_physics
  // if the node isn't a top Solid webots will throw a warning but still return true to ros
  ros::ServiceClient supervisor_node_reset_physics_client = n.serviceClient<nodes::node_reset_physics>(model_name+"/supervisor/node/reset_physics");
  nodes::node_reset_physics supervisor_node_reset_physics_srv;
  
  supervisor_node_reset_physics_srv.request.node = from_def_node;
  if (supervisor_node_reset_physics_client.call(supervisor_node_reset_physics_srv) && supervisor_node_reset_physics_srv.response.success == 1)
    ROS_INFO("node physics has been reset successfully");
  else
    ROS_ERROR("Failed to call service node_reset_physics");
  
  time_step_client.call(time_step_srv);
  
  ros::ServiceClient supervisor_node_get_field_client;
  nodes::node_get_field supervisor_node_get_field_srv;
  supervisor_node_get_field_client = n.serviceClient<nodes::node_get_field>(model_name+"/supervisor/node/get_field");
  
  supervisor_node_get_field_srv.request.node = root_node;
  supervisor_node_get_field_srv.request.fieldName = "children";
  supervisor_node_get_field_client.call(supervisor_node_get_field_srv);
  uint64_t field = supervisor_node_get_field_srv.response.field;
  
  time_step_client.call(time_step_srv);
  
  ros::ServiceClient supervisor_field_get_type_client;
  nodes::field_get_type supervisor_field_get_type_srv;
  supervisor_field_get_type_client = n.serviceClient<nodes::field_get_type>(model_name+"/supervisor/field/get_type");
  
  supervisor_field_get_type_srv.request.field = field;
  supervisor_field_get_type_client.call(supervisor_field_get_type_srv);
  ROS_INFO("wolrd's children field is of type %d", supervisor_field_get_type_srv.response.type);
  
  time_step_client.call(time_step_srv);
  
  ros::ServiceClient supervisor_field_get_type_name_client;
  nodes::field_get_type_name supervisor_field_get_type_name_srv;
  supervisor_field_get_type_name_client = n.serviceClient<nodes::field_get_type_name>(model_name+"/supervisor/field/get_type_name");
  
  supervisor_field_get_type_name_srv.request.field = field;
  supervisor_field_get_type_name_client.call(supervisor_field_get_type_name_srv);
  ROS_INFO("also known as %s", supervisor_field_get_type_name_srv.response.name.c_str());
  
  time_step_client.call(time_step_srv);
  
  ros::ServiceClient supervisor_field_get_count_client;
  nodes::field_get_count supervisor_field_get_count_srv;
  supervisor_field_get_count_client = n.serviceClient<nodes::field_get_count>(model_name+"/supervisor/field/get_count");
  
  supervisor_field_get_count_srv.request.field = field;
  supervisor_field_get_count_client.call(supervisor_field_get_count_srv);
  if (supervisor_field_get_count_srv.response.count != -1)
    ROS_INFO("there is %d nodes in this field", supervisor_field_get_count_srv.response.count);
  else
    ROS_ERROR("Illegal call to Field::getCount() argument must be multiple fields");
  
  time_step_client.call(time_step_srv);
  
  supervisor_node_get_field_srv.request.node = from_def_node;
  supervisor_node_get_field_srv.request.fieldName = "name";
  supervisor_node_get_field_client.call(supervisor_node_get_field_srv);
  field = supervisor_node_get_field_srv.response.field;
  
  ros::ServiceClient supervisor_field_set_string_client;
  nodes::field_set_string supervisor_field_set_string_srv;
  supervisor_field_set_string_client = n.serviceClient<nodes::field_set_string>(model_name+"/supervisor/field/set_string");
  
  supervisor_field_set_string_srv.request.field = field;
  supervisor_field_set_string_srv.request.value = "solid_test";
  if (supervisor_field_set_string_client.call(supervisor_field_set_string_srv) && supervisor_field_set_string_srv.response.success == 1)
    ROS_INFO("field's string updated to: 'solid_test'");
  else
    ROS_ERROR("Failed to call service field_set_string");
  
  time_step_client.call(time_step_srv);
  
  ros::ServiceClient supervisor_field_get_string_client;
  nodes::field_get_string supervisor_field_get_string_srv;
  supervisor_field_get_string_client = n.serviceClient<nodes::field_get_string>(model_name+"/supervisor/field/get_string");
  
  supervisor_field_get_string_srv.request.field = field;
  supervisor_field_get_string_client.call(supervisor_field_get_string_srv);
  ROS_INFO("field contains \"%s\"", supervisor_field_get_string_srv.response.value.c_str());
  
  time_step_client.call(time_step_srv);
  
  supervisor_node_get_field_srv.request.node = root_node;
  supervisor_node_get_field_srv.request.fieldName = "children";
  supervisor_node_get_field_client.call(supervisor_node_get_field_srv);
  field = supervisor_node_get_field_srv.response.field;
  
  ros::ServiceClient supervisor_field_get_node_client;
  nodes::field_get_node supervisor_field_get_node_srv;
  supervisor_field_get_node_client = n.serviceClient<nodes::field_get_node>(model_name+"/supervisor/field/get_node");
  
  supervisor_field_get_node_srv.request.field = field;
  supervisor_field_get_node_srv.request.index = 7;
  supervisor_field_get_node_client.call(supervisor_field_get_node_srv);
  
  supervisor_node_get_type_name_srv.request.node = supervisor_field_get_node_srv.response.node;
  supervisor_node_get_type_name_client.call(supervisor_node_get_type_name_srv);
  ROS_INFO("node got from field_get_node is of type %s", supervisor_node_get_type_name_srv.response.name.c_str());

  time_step_client.call(time_step_srv);
  
    
  ros::ServiceClient supervisor_start_movie_client;
  nodes::supervisor_start_movie supervisor_start_movie_srv;
  supervisor_start_movie_client = n.serviceClient<nodes::supervisor_start_movie>(model_name+"/supervisor/start_movie");
  
  supervisor_start_movie_srv.request.filename = "movie_test.mp4";
  supervisor_start_movie_srv.request.width = 480;
  supervisor_start_movie_srv.request.height = 360;
  supervisor_start_movie_srv.request.codec = 1337;
  supervisor_start_movie_srv.request.quality = 100;
  supervisor_start_movie_srv.request.acceleration = 1;
  supervisor_start_movie_srv.request.caption = false;
  if (supervisor_start_movie_client.call(supervisor_start_movie_srv) && supervisor_start_movie_srv.response.success == 1)
      ROS_INFO("movie started successfully");
  else
      ROS_ERROR("failed to call service start_movie");
  
  time_step_client.call(time_step_srv);
  time_step_client.call(time_step_srv);
  time_step_client.call(time_step_srv);
  time_step_client.call(time_step_srv);
  time_step_client.call(time_step_srv);
  time_step_client.call(time_step_srv);
  time_step_client.call(time_step_srv);
  time_step_client.call(time_step_srv);
  time_step_client.call(time_step_srv);
  time_step_client.call(time_step_srv);
  time_step_client.call(time_step_srv);
  time_step_client.call(time_step_srv);
  time_step_client.call(time_step_srv);
  time_step_client.call(time_step_srv);
  time_step_client.call(time_step_srv);
  time_step_client.call(time_step_srv);
  time_step_client.call(time_step_srv);
  time_step_client.call(time_step_srv);
  time_step_client.call(time_step_srv);
  time_step_client.call(time_step_srv);
  time_step_client.call(time_step_srv);
  time_step_client.call(time_step_srv);
  time_step_client.call(time_step_srv);
  time_step_client.call(time_step_srv);
  time_step_client.call(time_step_srv);
  
  ros::ServiceClient supervisor_get_movie_status_client;
  nodes::supervisor_get_movie_status supervisor_get_movie_status_srv;
  supervisor_get_movie_status_client = n.serviceClient<nodes::supervisor_get_movie_status>(model_name+"/supervisor/get_movie_status");
  
  if (supervisor_get_movie_status_client.call(supervisor_get_movie_status_srv) && supervisor_get_movie_status_srv.response.status == 1)
      ROS_INFO("movie status received correctly");
  else
      ROS_ERROR("failed to call service get_movie_status");
  
  ros::ServiceClient supervisor_stop_movie_client;
  nodes::supervisor_stop_movie supervisor_stop_movie_srv;
  supervisor_stop_movie_client = n.serviceClient<nodes::supervisor_stop_movie>(model_name+"/supervisor/stop_movie");
  
  if (supervisor_stop_movie_client.call(supervisor_stop_movie_srv) && supervisor_stop_movie_srv.response.success == 1)
      ROS_INFO("movie stopped successfully");
  else
      ROS_ERROR("failed to call service stop_movie");
  
    supervisor_set_label_srv.request.label = "";
  supervisor_set_label_client.call(supervisor_set_label_srv);

    
  // test field_import_node
    // this test is disabled as it needs a webots object file but it will works if you export the cone_test solid from the world before
    //  ros::ServiceClient supervisor_field_import_node_client;
    //  nodes::field_import_node supervisor_field_import_node_srv;
    //  supervisor_field_import_node_client = n.serviceClient<nodes::field_import_node>(model_name+"/supervisor/field/import_node");
    //  
    //  supervisor_field_import_node_srv.request.field = field;
    //  supervisor_field_import_node_srv.request.position = 6;
    //  supervisor_field_import_node_srv.request.filename = "cone_test.wbo";
    //  if (supervisor_field_import_node_client.call(supervisor_field_import_node_srv) && supervisor_field_import_node_srv.response.success == 1)
    //    ROS_INFO("new cone add in world");
    //  else
    //    ROS_ERROR("Failed to call service field_import_node");
  
  // The next 2 tests are commented but works if you want to use them.
  // Since they stop simulation we can't use them if we wants to do other tests afterwards
  
  //~ ros::ServiceClient supervisor_simulation_quit_client;
  //~ nodes::supervisor_simulation_quit supervisor_simulation_quit_srv;
  //~ supervisor_simulation_quit_client = n.serviceClient<nodes::supervisor_simulation_quit>(model_name+"/supervisor/simulation_quit");
  //~
  //~ supervisor_simulation_quit_srv.request.status = EXIT_SUCCESS;
  //~ if (supervisor_simulation_quit_client.call(supervisor_simulation_quit_srv) && supervisor_simulation_quit_srv.response.success == 1)
    //~ ROS_INFO("Webots quit successfully");
  //~ else
    //~ ROS_ERROR("Failed to call service simulation_quit");
  
  //~ ros::ServiceClient supervisor_simulation_revert_client;
  //~ nodes::supervisor_simulation_revert supervisor_simulation_revert_srv;
  //~ supervisor_simulation_revert_client = n.serviceClient<nodes::supervisor_simulation_revert>(model_name+"/supervisor/simulation_revert");
  //~
  //~ if (supervisor_simulation_revert_client.call(supervisor_simulation_revert_srv) && supervisor_simulation_revert_srv.response.success == 1)
    //~ ROS_INFO("simulation has revert successfully");
  //~ else
    //~ ROS_ERROR("Failed to call service simulation_revert");
  
  // end of tests
  time_step_srv.request.step = 0;
  if (time_step_client.call(time_step_srv) && time_step_srv.response.success == 2)
    ROS_INFO("robot's time step called to end tests");
  else
    ROS_ERROR("Failed to call service time_step to end tests");
    
  ros::shutdown();
  return 0;
}
