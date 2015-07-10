//#include <webots/robot.h>


#include "nodes/LegEncoders.h"
#include "nodes/LegCommand.h"
#include "ros/ros.h"


#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/UInt8MultiArray.h>
#include <cstdlib>
#include <signal.h>

#include <nodes/connector_get_presence.h>
#include <nodes/connector_lock.h>
#include <nodes/connector_set_presence.h>
#include <nodes/differential_wheels_enable_control.h>
#include <nodes/differential_wheels_get_speed.h>

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
//#include <nodes/field_set_string.h>
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
//#include <nodes/node_reset_physics.h>
//#include <nodes/pen_set_ink_color.h>
//#include <nodes/pen_write.h>
//#include <nodes/receiver_get_channel.h>
//#include <nodes/receiver_get_data_size.h>
//#include <nodes/receiver_get_emitter_direction.h>
//#include <nodes/receiver_get_queue_length.h>
//#include <nodes/receiver_get_signal_strength.h>
//#include <nodes/receiver_next_packet.h>
//#include <nodes/receiver_set_channel.h>
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
//#include <nodes/robot_set_keyboard.h>
#include <nodes/robot_set_mode.h>
#include <nodes/robot_set_time_step.h>
#include <nodes/sensor_get_type.h>
#include <nodes/sensor_sampling_period.h>
#include <nodes/sensor_set.h>
//#include <nodes/supervisor_export_image.h>
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

#define TIME_STEP 1

using namespace std;

static int model_count;
static vector<string> model_list;
//vector<unsigned char> imageColor;
//vector<float> imageRangeFinder;
//double accelerometerValues[3] = {0, 0, 0};
//double compassValues[3] = {0, 0, 0};
double GPSValues[3] = {0, 0, 0};
//double GyroValues[3] = {0, 0, 0};
double inertialUnitValues[3] = {0, 0, 0};
double touchSensorValues[3] = {0, 0, 0};
double positionsensor[3] = {0, 0, 0};
int8_t pwmHip1;  // Holds the pwm commands

ros::ServiceClient time_step_client;
nodes::robot_set_time_step time_step_srv;
nodes::LegEncoders encoders_msg;



// Constants
#define PI 3.141592f
#define MAXTORQUE 6.5
#define RAD2DEG (180.0f/PI)

// ROS parameter with list of controllers to load
#define ROSPARAM_CONTROLLERS_LIST "webots_controllers_to_load"


void modelNameCallback(const std_msgs::String::ConstPtr& name)
{
  model_count++;
  model_list.push_back(name->data);
  ROS_INFO("model #%d : %s", model_count, model_list.back().c_str());
}


void battery_sensorCallback(const std_msgs::Float64::ConstPtr& value)
{
  ROS_INFO("battery level is %f",value->data);
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

void positionSensorCallback1(const std_msgs::Float64::ConstPtr& value)
{
  
	ROS_INFO("position sensor sent value %f", value->data);

	// 2 * π = 6.2831853072;
	// π = 3.14159265359;
	// rad2deg = deg * 180 / π;
  	// counts per revolution = 500 * 4 = 2000
	// transmission ratio = 51
	// 360 degrees = 102000 counts
	// e.g. 36 degrees = 102000 * 36 / 360
	
	encoders_msg.encoder_hip = (int32_t)( 102000 * ( ( value->data * RAD2DEG ) / 360 ) );

  	//encoders_msg.encoder_knee = (int32_t)(value->data * 10000); 
  	//encoders_msg.encoder_heel = (int32_t)(value->data * 10000);
  	//encoders_msg.encoder_spring = (int32_t)(value->data * 10000);
  
	//ROS_INFO("position sensor sent value");
}

// Callback function for reading pwm command
void pwmCallback(const nodes::LegCommand::ConstPtr& msg)
{
	 pwmHip1 = (int8_t) msg->pwm_duty_hip;
	 //ROS_INFO("position sensor sent value %f", msg->pwm_duty_hip);
}

void positionSensorCallback2(const std_msgs::Float64::ConstPtr& value)
{
  ROS_INFO("position sensor sent value %f", value->data);
  
	//ROS_INFO("position sensor sent value");
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
  ROS_INFO("user stopped the node complete_test1.");
  ros::shutdown();
  exit(0);
}

int main(int argc, char **argv)
{
  int wanted_model = 0;
  string model_name;

  int i, msg_count;
 
  
  if (argc != 1)  {
    ROS_INFO("usage: $ complete_test1");
    return 1;
  }
  
  ros::init(argc, argv, "complete_test1", ros::init_options::AnonymousName);
 
  ros::NodeHandle n;
  ros::Publisher motor_interface_pub = n.advertise<nodes::LegEncoders>("FL_encoders_feedback", 1000);

  signal(SIGINT,quit);

  
  ros::Subscriber name_sub = n.subscribe("model_name", 100, modelNameCallback);
  
  ros::Subscriber motor_pid_sub = n.subscribe("FL_command_dispatch", 1000, pwmCallback);
  
  //ros::spin();
  
  ros::Rate loop_rate(12000);
  
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
  
  ////////////////////////////////////
  // MOTOR METHODS TEST * servo1_rot//
  ////////////////////////////////////

  ros::ServiceClient motor_get_type_client1;
  nodes::robot_get_type motor_get_type_srv1;
  motor_get_type_client1 = n.serviceClient<nodes::robot_get_type>(model_name+"/servo1_rot/get_type");
  motor_get_type_client1.call(motor_get_type_srv1);
  ROS_INFO("rotational_motor is of type %d", motor_get_type_srv1.response.type);
  
  time_step_client.call(time_step_srv);

  ros::ServiceClient set_torque_client1;
  nodes::motor_set_torque set_torque_srv1;
  set_torque_client1 = n.serviceClient<nodes::motor_set_torque>(model_name+"/servo1_rot/set_torque");
  
  set_torque_srv1.request.torque = MAXTORQUE * pwmHip1;
  if (set_torque_client1.call(set_torque_srv1) && set_torque_srv1.response.success == 1)
    ROS_INFO("torque set to 6.5");
  else
    ROS_ERROR("Failed to call service set_torque on motor");
  
  time_step_client.call(time_step_srv);
  
  ros::ServiceClient set_available_torque_client1;
  nodes::motor_set_available_torque set_available_torque_srv1;
  set_available_torque_client1 = n.serviceClient<nodes::motor_set_available_torque>(model_name+"/servo1_rot/set_available_torque");
  
  set_available_torque_srv1.request.torque = 6.5;
  if (set_available_torque_client1.call(set_available_torque_srv1) && set_available_torque_srv1.response.success == 1)
    ROS_INFO("available_torque set to 0.8");
  else
    ROS_ERROR("Failed to call service set_available_torque on motor");
  
  time_step_client.call(time_step_srv);

  ros::ServiceClient set_controlp_client1;
  nodes::motor_set_control_p set_controlp_srv1;
  set_controlp_client1 = n.serviceClient<nodes::motor_set_control_p>(model_name+"/servo1_rot/set_controlp");
  
/*

  set_controlp_srv1.request.controlp = 1;
  if (set_controlp_client1.call(set_controlp_srv1) && set_controlp_srv1.response.success == 1)
    ROS_INFO("controlp set to 1");
  else
    ROS_ERROR("Failed to call service set_controlp on motor");
  
  time_step_client.call(time_step_srv);

  ros::ServiceClient set_position_client1;
  nodes::motor_set_position set_position_srv1;
  set_position_client1 = n.serviceClient<nodes::motor_set_position>(model_name+"/servo1_rot/set_position");
  
  set_position_srv1.request.position = 0.3;
  if (set_position_client1.call(set_position_srv1) && set_position_srv1.response.success == 1)
    ROS_INFO("position set to 0.3");
  else
    ROS_ERROR("Failed to call service set_position on motor");
  
  time_step_client.call(time_step_srv);

  ros::ServiceClient get_target_position_client1;
  nodes::motor_get_target_position get_target_position_srv1;
  get_target_position_client1 = n.serviceClient<nodes::motor_get_target_position>(model_name+"/servo1_rot/get_target_position");
  
  get_target_position_client1.call(get_target_position_srv1);
  ROS_INFO("target position for rotational_motor is %f", get_target_position_srv1.response.position);
  
  time_step_client.call(time_step_srv);

  ros::ServiceClient get_min_position_client1;
  nodes::motor_get_min_position get_min_position_srv1;
  get_min_position_client1 = n.serviceClient<nodes::motor_get_min_position>(model_name+"/servo1_rot/get_min_position");
  
  get_min_position_client1.call(get_min_position_srv1);
  ROS_INFO("min position for rotational_motor is %f", get_min_position_srv1.response.position);
  
  time_step_client.call(time_step_srv);

  ros::ServiceClient get_max_position_client1;
  nodes::motor_get_max_position get_max_position_srv1;
  get_max_position_client1 = n.serviceClient<nodes::motor_get_max_position>(model_name+"/servo1_rot/get_max_position");
  
  get_max_position_client1.call(get_max_position_srv1);
  ROS_INFO("max position for rotational_motor is %f", get_max_position_srv1.response.position);
  
  time_step_client.call(time_step_srv);
  time_step_client.call(time_step_srv);
  
*/

  ////////////////////////////////////
  // MOTOR METHODS TEST * servo2_rot//
  ////////////////////////////////////

  ros::ServiceClient motor_get_type_client2;
  nodes::robot_get_type motor_get_type_srv2;
  motor_get_type_client2 = n.serviceClient<nodes::robot_get_type>(model_name+"/servo2_rot/get_type");
  motor_get_type_client2.call(motor_get_type_srv2);
  ROS_INFO("rotational_motor is of type %d", motor_get_type_srv2.response.type);
  
  time_step_client.call(time_step_srv);

  ros::ServiceClient set_torque_client2;
  nodes::motor_set_torque set_torque_srv2;
  set_torque_client2 = n.serviceClient<nodes::motor_set_torque>(model_name+"/servo2_rot/set_torque");
  
  set_torque_srv2.request.torque = 0.5;
  if (set_torque_client2.call(set_torque_srv2) && set_torque_srv2.response.success == 1)
    ROS_INFO("torque set to 0.5");
  else
    ROS_ERROR("Failed to call service set_torque on motor");
  
  time_step_client.call(time_step_srv);
  
  ros::ServiceClient set_available_torque_client2;
  nodes::motor_set_available_torque set_available_torque_srv2;
  set_available_torque_client2 = n.serviceClient<nodes::motor_set_available_torque>(model_name+"/servo2_rot/set_available_torque");
  
  set_available_torque_srv2.request.torque = 0.8;
  if (set_available_torque_client2.call(set_available_torque_srv2) && set_available_torque_srv2.response.success == 1)
    ROS_INFO("available_torque set to 0.8");
  else
    ROS_ERROR("Failed to call service set_available_torque on motor");
  
  time_step_client.call(time_step_srv);

  ros::ServiceClient set_controlp_client2;
  nodes::motor_set_control_p set_controlp_srv2;
  set_controlp_client2 = n.serviceClient<nodes::motor_set_control_p>(model_name+"/servo2_rot/set_controlp");
  
  set_controlp_srv2.request.controlp = 1;
  if (set_controlp_client2.call(set_controlp_srv2) && set_controlp_srv2.response.success == 1)
    ROS_INFO("controlp set to 1");
  else
    ROS_ERROR("Failed to call service set_controlp on motor");
  
  time_step_client.call(time_step_srv);
  time_step_client.call(time_step_srv);

  ros::ServiceClient set_position_client2;
  nodes::motor_set_position set_position_srv2;
  set_position_client2 = n.serviceClient<nodes::motor_set_position>(model_name+"/servo2_rot/set_position");
  
  set_position_srv2.request.position = 0.15;
  if (set_position_client2.call(set_position_srv2) && set_position_srv2.response.success == 1)
    ROS_INFO("position set to 0.3");
  else
    ROS_ERROR("Failed to call service set_position on motor");
  
  time_step_client.call(time_step_srv);
  time_step_client.call(time_step_srv);

  ros::ServiceClient get_target_position_client2;
  nodes::motor_get_target_position get_target_position_srv2;
  get_target_position_client2 = n.serviceClient<nodes::motor_get_target_position>(model_name+"/servo2_rot/get_target_position");
  
  get_target_position_client2.call(get_target_position_srv2);
  ROS_INFO("target position for rotational_motor is %f", get_target_position_srv2.response.position);
  
  time_step_client.call(time_step_srv);

  ros::ServiceClient get_min_position_client2;
  nodes::motor_get_min_position get_min_position_srv2;
  get_min_position_client2 = n.serviceClient<nodes::motor_get_min_position>(model_name+"/servo2_rot/get_min_position");
  
  get_min_position_client2.call(get_min_position_srv2);
  ROS_INFO("min position for rotational_motor is %f", get_min_position_srv2.response.position);
  
  time_step_client.call(time_step_srv);

  ros::ServiceClient get_max_position_client2;
  nodes::motor_get_max_position get_max_position_srv2;
  get_max_position_client2 = n.serviceClient<nodes::motor_get_max_position>(model_name+"/servo2_rot/get_max_position");
  
  get_max_position_client2.call(get_max_position_srv2);
  ROS_INFO("max position for rotational_motor is %f", get_max_position_srv2.response.position);
  
  time_step_client.call(time_step_srv);
  time_step_client.call(time_step_srv);
  
  ////////////////////////////////////
  // MOTOR METHODS TEST * servo3_rot//
  ////////////////////////////////////

  ros::ServiceClient motor_get_type_client3;
  nodes::robot_get_type motor_get_type_srv3;
  motor_get_type_client3 = n.serviceClient<nodes::robot_get_type>(model_name+"/servo3_rot/get_type");
  motor_get_type_client3.call(motor_get_type_srv3);
  ROS_INFO("rotational_motor is of type %d", motor_get_type_srv3.response.type);
  
  time_step_client.call(time_step_srv);

  ros::ServiceClient set_torque_client3;
  nodes::motor_set_torque set_torque_srv3;
  set_torque_client3 = n.serviceClient<nodes::motor_set_torque>(model_name+"/servo3_rot/set_torque");
  
  set_torque_srv3.request.torque = 0.5;
  if (set_torque_client3.call(set_torque_srv3) && set_torque_srv3.response.success == 1)
    ROS_INFO("torque set to 0.5");
  else
    ROS_ERROR("Failed to call service set_torque on motor");
  
  time_step_client.call(time_step_srv);
  
  ros::ServiceClient set_available_torque_client3;
  nodes::motor_set_available_torque set_available_torque_srv3;
  set_available_torque_client3 = n.serviceClient<nodes::motor_set_available_torque>(model_name+"/servo3_rot/set_available_torque");
  
  set_available_torque_srv3.request.torque = 0.8;
  if (set_available_torque_client3.call(set_available_torque_srv3) && set_available_torque_srv3.response.success == 1)
    ROS_INFO("available_torque set to 0.8");
  else
    ROS_ERROR("Failed to call service set_available_torque on motor");
  
  time_step_client.call(time_step_srv);

  ros::ServiceClient set_controlp_client3;
  nodes::motor_set_control_p set_controlp_srv3;
  set_controlp_client3 = n.serviceClient<nodes::motor_set_control_p>(model_name+"/servo3_rot/set_controlp");
  
  set_controlp_srv3.request.controlp = 1;
  if (set_controlp_client3.call(set_controlp_srv3) && set_controlp_srv3.response.success == 1)
    ROS_INFO("controlp set to 1");
  else
    ROS_ERROR("Failed to call service set_controlp on motor");
  
  time_step_client.call(time_step_srv);
  time_step_client.call(time_step_srv);

  ros::ServiceClient set_position_client3;
  nodes::motor_set_position set_position_srv3;
  set_position_client3 = n.serviceClient<nodes::motor_set_position>(model_name+"/servo3_rot/set_position");
  
  set_position_srv3.request.position = 0.15;
  if (set_position_client3.call(set_position_srv3) && set_position_srv3.response.success == 1)
    ROS_INFO("position set to 0.3");
  else
    ROS_ERROR("Failed to call service set_position on motor");
  
  time_step_client.call(time_step_srv);
  time_step_client.call(time_step_srv);

  ros::ServiceClient get_target_position_client3;
  nodes::motor_get_target_position get_target_position_srv3;
  get_target_position_client3 = n.serviceClient<nodes::motor_get_target_position>(model_name+"/servo3_rot/get_target_position");
  
  get_target_position_client3.call(get_target_position_srv3);
  ROS_INFO("target position for rotational_motor is %f", get_target_position_srv3.response.position);
  
  time_step_client.call(time_step_srv);

  ros::ServiceClient get_min_position_client3;
  nodes::motor_get_min_position get_min_position_srv3;
  get_min_position_client3 = n.serviceClient<nodes::motor_get_min_position>(model_name+"/servo3_rot/get_min_position");
  
  get_min_position_client3.call(get_min_position_srv3);
  ROS_INFO("min position for rotational_motor is %f", get_min_position_srv3.response.position);
  
  time_step_client.call(time_step_srv);

  ros::ServiceClient get_max_position_client3;
  nodes::motor_get_max_position get_max_position_srv3;
  get_max_position_client3 = n.serviceClient<nodes::motor_get_max_position>(model_name+"/servo3_rot/get_max_position");
  
  get_max_position_client3.call(get_max_position_srv3);
  ROS_INFO("max position for rotational_motor is %f", get_max_position_srv3.response.position);
  
  time_step_client.call(time_step_srv);
  time_step_client.call(time_step_srv);

  ////////////////////////////////////
  // MOTOR METHODS TEST * servo4_rot//
  ////////////////////////////////////

  ros::ServiceClient motor_get_type_client4;
  nodes::robot_get_type motor_get_type_srv4;
  motor_get_type_client4 = n.serviceClient<nodes::robot_get_type>(model_name+"/servo4_rot/get_type");
  motor_get_type_client4.call(motor_get_type_srv4);
  ROS_INFO("rotational_motor is of type %d", motor_get_type_srv4.response.type);
  
  time_step_client.call(time_step_srv);

  ros::ServiceClient set_torque_client4;
  nodes::motor_set_torque set_torque_srv4;
  set_torque_client4 = n.serviceClient<nodes::motor_set_torque>(model_name+"/servo4_rot/set_torque");
  
  set_torque_srv4.request.torque = 0.5;
  if (set_torque_client4.call(set_torque_srv4) && set_torque_srv4.response.success == 1)
    ROS_INFO("torque set to 0.5");
  else
    ROS_ERROR("Failed to call service set_torque on motor");
  
  time_step_client.call(time_step_srv);
  
  ros::ServiceClient set_available_torque_client4;
  nodes::motor_set_available_torque set_available_torque_srv4;
  set_available_torque_client4 = n.serviceClient<nodes::motor_set_available_torque>(model_name+"/servo4_rot/set_available_torque");
  
  set_available_torque_srv4.request.torque = 0.8;
  if (set_available_torque_client4.call(set_available_torque_srv4) && set_available_torque_srv4.response.success == 1)
    ROS_INFO("available_torque set to 0.8");
  else
    ROS_ERROR("Failed to call service set_available_torque on motor");
  
  time_step_client.call(time_step_srv);

  ros::ServiceClient set_controlp_client4;
  nodes::motor_set_control_p set_controlp_srv4;
  set_controlp_client4 = n.serviceClient<nodes::motor_set_control_p>(model_name+"/servo4_rot/set_controlp");
  
  set_controlp_srv4.request.controlp = 1;
  if (set_controlp_client4.call(set_controlp_srv4) && set_controlp_srv4.response.success == 1)
    ROS_INFO("controlp set to 1");
  else
    ROS_ERROR("Failed to call service set_controlp on motor");
  
  time_step_client.call(time_step_srv);
  time_step_client.call(time_step_srv);

  ros::ServiceClient set_position_client4;
  nodes::motor_set_position set_position_srv4;
  set_position_client4 = n.serviceClient<nodes::motor_set_position>(model_name+"/servo4_rot/set_position");
  
  set_position_srv4.request.position = 0.3;
  if (set_position_client4.call(set_position_srv4) && set_position_srv4.response.success == 1)
    ROS_INFO("position set to 0.3");
  else
    ROS_ERROR("Failed to call service set_position on motor");
  
  time_step_client.call(time_step_srv);
  time_step_client.call(time_step_srv);

  ros::ServiceClient get_target_position_client4;
  nodes::motor_get_target_position get_target_position_srv4;
  get_target_position_client4 = n.serviceClient<nodes::motor_get_target_position>(model_name+"/servo4_rot/get_target_position");
  
  get_target_position_client4.call(get_target_position_srv4);
  ROS_INFO("target position for rotational_motor is %f", get_target_position_srv4.response.position);
  
  time_step_client.call(time_step_srv);

  ros::ServiceClient get_min_position_client4;
  nodes::motor_get_min_position get_min_position_srv4;
  get_min_position_client4 = n.serviceClient<nodes::motor_get_min_position>(model_name+"/servo4_rot/get_min_position");
  
  get_min_position_client4.call(get_min_position_srv4);
  ROS_INFO("min position for rotational_motor is %f", get_min_position_srv4.response.position);
  
  time_step_client.call(time_step_srv);

  ros::ServiceClient get_max_position_client4;
  nodes::motor_get_max_position get_max_position_srv4;
  get_max_position_client4 = n.serviceClient<nodes::motor_get_max_position>(model_name+"/servo4_rot/get_max_position");
  
  get_max_position_client4.call(get_max_position_srv4);
  ROS_INFO("max position for rotational_motor is %f", get_max_position_srv4.response.position);
  
  time_step_client.call(time_step_srv);
  time_step_client.call(time_step_srv);

  //////////////////////////////////////////////////
  // POSITION SENSOR METHODS TEST * servo1_rot_pos//
  //////////////////////////////////////////////////

  ros::ServiceClient set_position_sensor_client;
  nodes::sensor_set position_sensor_srv;
  ros::Subscriber sub_position_sensor_1;
  set_position_sensor_client = n.serviceClient<nodes::sensor_set>(model_name+"/servo1_rot_pos/set_sensor");
  
  ros::ServiceClient sampling_period_position_sensor_client;
  nodes::sensor_sampling_period sampling_period_position_sensor_srv;
  sampling_period_position_sensor_client = n.serviceClient<nodes::sensor_sampling_period>(model_name+"/servo1_rot_pos/sampling_period");
  
  position_sensor_srv.request.period = 1;
  if (set_position_sensor_client.call(position_sensor_srv) && position_sensor_srv.response.success == 1)
  {
    ROS_INFO("position_sensor enabled");
    sub_position_sensor_1 = n.subscribe(model_name+"/servo1_rot_pos/1", 1, positionSensorCallback1);
    while (sub_position_sensor_1.getNumPublishers() == 0);
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
  position_sensor_get_type_client = n.serviceClient<nodes::sensor_get_type>(model_name+"/servo1_rot_pos/get_type");

  //ros::ServiceClient position_sensor_get_value_client;
  //nodes::sensor_get_value position_sensor_get_value_srv;
  //position_sensor_get_value_client = n.serviceClient<nodes::sensor_get_value>(model_name+"/servo1_rot_pos/get_value");
  
  position_sensor_get_type_client.call(position_sensor_get_type_srv);
  ROS_INFO("position_sensor is of type %d", position_sensor_get_type_srv.response.type);
  
  time_step_client.call(time_step_srv);

  //sub_position_sensor_1.shutdown();

  time_step_client.call(time_step_srv);
  time_step_client.call(time_step_srv);
  time_step_client.call(time_step_srv);
  
  sampling_period_position_sensor_client.call(sampling_period_position_sensor_srv);
  //ROS_INFO("position_sensor is disabled (sampling period is %d)", sampling_period_position_sensor_srv.response.period);
  
  //////////////////////////////////////////////////
  // POSITION SENSOR METHODS TEST * servo2_rot_pos//
  //////////////////////////////////////////////////

  ros::ServiceClient set_position_sensor_client2;
  nodes::sensor_set position_sensor_srv2;
  ros::Subscriber sub_position_sensor_2;
  set_position_sensor_client2 = n.serviceClient<nodes::sensor_set>(model_name+"/servo2_rot_pos/set_sensor");
  
  ros::ServiceClient sampling_period_position_sensor_client2;
  nodes::sensor_sampling_period sampling_period_position_sensor_srv2;
  sampling_period_position_sensor_client2 = n.serviceClient<nodes::sensor_sampling_period>(model_name+"/servo2_rot_pos/sampling_period");
  
  position_sensor_srv2.request.period = 1;
  if (set_position_sensor_client2.call(position_sensor_srv2) && position_sensor_srv2.response.success == 1)
  {
    ROS_INFO("position_sensor enabled");
    sub_position_sensor_2 = n.subscribe(model_name+"/servo2_rot_pos/1", 1, positionSensorCallback2);
    while (sub_position_sensor_2.getNumPublishers() == 0);
  }
  else
  {
    if (position_sensor_srv2.response.success == 2)
      ROS_ERROR("Sampling period is not valid");
    ROS_ERROR("Failed to enable position_sensor");
    return 1;
  }
  
  time_step_client.call(time_step_srv);
  
  sampling_period_position_sensor_client2.call(sampling_period_position_sensor_srv2);
  ROS_INFO("position_sensor is enable with a sampling period of %d", sampling_period_position_sensor_srv2.response.period);
  
  time_step_client.call(time_step_srv);
  
  ros::ServiceClient position_sensor_get_type_client2;
  nodes::sensor_get_type position_sensor_get_type_srv2;
  position_sensor_get_type_client2 = n.serviceClient<nodes::sensor_get_type>(model_name+"/servo2_rot_pos/get_type");

  
  position_sensor_get_type_client2.call(position_sensor_get_type_srv2);
  ROS_INFO("position_sensor is of type %d", position_sensor_get_type_srv2.response.type);
  
  time_step_client.call(time_step_srv);

  //sub_position_sensor_1.shutdown();

  time_step_client.call(time_step_srv);
  time_step_client.call(time_step_srv);
  time_step_client.call(time_step_srv);
  
  sampling_period_position_sensor_client2.call(sampling_period_position_sensor_srv2);
  //ROS_INFO("position_sensor is disabled (sampling period is %d)", sampling_period_position_sensor_srv.response.period);
  
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
  
  //ros::ServiceClient supervisor_node_get_orientation_client;
  //nodes::node_get_orientation supervisor_node_get_orientation_srv;
  //supervisor_node_get_orientation_client = n.serviceClient<nodes::node_get_orientation>(model_name+"/supervisor/node/get_orientation");
  
  //supervisor_node_get_orientation_srv.request.node = from_def_node;
  //supervisor_node_get_orientation_client.call(supervisor_node_get_orientation_srv);
  //ROS_INFO("from_def orientation matrix is :\n%f %f %f\n%f %f %f\n%f %f %f", supervisor_node_get_orientation_srv.response.orientation[0], supervisor_node_get_orientation_srv.response.orientation[1],  supervisor_node_get_orientation_srv.response.orientation[2], supervisor_node_get_orientation_srv.response.orientation[3], supervisor_node_get_orientation_srv.response.orientation[4], supervisor_node_get_orientation_srv.response.orientation[5], supervisor_node_get_orientation_srv.response.orientation[6], supervisor_node_get_orientation_srv.response.orientation[7], supervisor_node_get_orientation_srv.response.orientation[8]);
  
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
  
  //supervisor_node_get_field_srv.request.node = from_def_node;
  supervisor_node_get_field_srv.request.fieldName = "name";
  supervisor_node_get_field_client.call(supervisor_node_get_field_srv);
  field = supervisor_node_get_field_srv.response.field;
  
  // end of tests
  time_step_srv.request.step = 0;
  if (time_step_client.call(time_step_srv) && time_step_srv.response.success == 2)
    ROS_INFO("robot's time step called to end tests");
  else
    ROS_ERROR("Failed to call service time_step to end tests");
    
    // main loop
    while (ros::ok())
    {
		//ROS_INFO("position_sensor enabled");
		//ROS_INFO("position_sensor is enable with a sampling period of %d", sub_position_sensor_1 );
		positionSensorCallback1;
		motor_interface_pub.publish(encoders_msg);
		positionSensorCallback2;
		ros::spinOnce();
    }
    
  ros::shutdown();
  return 0;
}
