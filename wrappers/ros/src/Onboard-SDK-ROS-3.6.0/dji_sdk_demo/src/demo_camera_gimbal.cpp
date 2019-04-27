/** @file demo_camera_gimbal.cpp
 *  @version 3.3
 *  @date May, 2017
 *
 *  @brief
 *  demo sample of how to use camera and gimbal APIs
 *
 *  @copyright 2017 DJI. All rights reserved.
 *
 */

#include <dji_sdk_demo/demo_camera_gimbal.h>

// global variables
geometry_msgs::Vector3Stamped gimbal_angle;
ros::Subscriber         gimbal_angle_subscriber;
ros::Subscriber         gimbal_camera_control_subscriber;
ros::Publisher          gimbal_angle_cmd_publisher;
ros::Publisher          gimbal_speed_cmd_publisher;
ros::Publisher          gimbal_camera_answer_publisher;
ros::ServiceClient      drone_activation_service;
ros::ServiceClient      camera_action_service;

GimbalContainer gimbal;
RotationAngle initialAngle;
dji_sdk::GimbalCameraControlData gimbal_camera_control_cmd_data;
bool cmd_flag = false;
int
main(int argc, char** argv)
{
  ros::init(argc, argv, "sdk_demo_camera_gimbal");
  ros::NodeHandle nh;

  // ROS stuff
  gimbal_angle_subscriber           = nh.subscribe<geometry_msgs::Vector3Stamped>
    ("dji_sdk/gimbal_angle", 10, &gimbalAngleCallback);
  gimbal_camera_control_subscriber  = nh.subscribe<dji_sdk::GimbalCameraControlData>
    ("gimbal_camera_control_cmd", 10, &gimbal_camera_control_callback);
  gimbal_angle_cmd_publisher        = nh.advertise<dji_sdk::Gimbal>
    ("dji_sdk/gimbal_angle_cmd", 10);
  gimbal_speed_cmd_publisher        = nh.advertise<geometry_msgs::Vector3Stamped>
    ("dji_sdk/gimbal_speed_cmd", 10);
  gimbal_camera_answer_publisher    = nh.advertise<dji_sdk::GimbalAnswer>
    ("gimbal_camera_answer",10);
  drone_activation_service          = nh.serviceClient<dji_sdk::Activation>
    ("dji_sdk/activation");
  camera_action_service             = nh.serviceClient<dji_sdk::CameraAction>
    ("dji_sdk/camera_action");

  initialAngle.roll     = 0;
  initialAngle.pitch    = 0;
  initialAngle.yaw      = 0;
  // Activate
  if (activate().result) {
    ROS_INFO("Activated successfully");
  } else {
    ROS_WARN("Failed activation");
    return -1;
  }
  while(ros::ok())
  {
    ros::spinOnce();
    if(cmd_flag)
    {
      GimbalCameraControl(gimbal_camera_control_cmd_data);
      dji_sdk::GimbalAnswer answer;
      answer.action_finished = true;
      gimbal_camera_answer_publisher.publish(answer);
      cmd_flag = false;
    }
  }

  return 0;
}

void GimbalCameraControl(dji_sdk::GimbalCameraControlData control_cmd)
{
  uint8_t took_photo_number = 0;
  ros::spinOnce();
  gimbal = GimbalContainer(control_cmd.roll,control_cmd.pitch,control_cmd.yaw,2,1,initialAngle);
  doSetGimbalAngle(&gimbal);
  if(0x01 & control_cmd.action)
  {
    //take one photo at least when the user forget to set the number
    takePicture();
    sleep(0.5);
    while(ros::ok() && took_photo_number < control_cmd.photo_number - 1)
    {
      takePicture();
      took_photo_number++;
      sleep(0.5);
    }
  }
  if(0x02 & control_cmd.action)
  {
    startVideo();
    sleep(control_cmd.video_time);
    stopVideo();
  }
}

void
doSetGimbalAngle(GimbalContainer *gimbal)
{
  dji_sdk::Gimbal gimbal_angle_data;
  gimbal_angle_data.mode |= 0;
  gimbal_angle_data.mode |= gimbal->isAbsolute;
  gimbal_angle_data.mode |= gimbal->yaw_cmd_ignore << 1;
  gimbal_angle_data.mode |= gimbal->roll_cmd_ignore << 2;
  gimbal_angle_data.mode |= gimbal->pitch_cmd_ignore << 3;
  gimbal_angle_data.ts    = gimbal->duration;
  gimbal_angle_data.roll  = DEG2RAD(gimbal->roll);
  gimbal_angle_data.pitch = DEG2RAD(gimbal->pitch);
  gimbal_angle_data.yaw   = DEG2RAD(gimbal->yaw);

  gimbal_angle_cmd_publisher.publish(gimbal_angle_data);
  // Give time for gimbal to sync
  sleep(4);
}

ServiceAck
activate()
{
  dji_sdk::Activation activation;
  drone_activation_service.call(activation);
  if(!activation.response.result) {
    ROS_WARN("ack.info: set = %i id = %i", activation.response.cmd_set, activation.response.cmd_id);
    ROS_WARN("ack.data: %i", activation.response.ack_data);
  }
  return {activation.response.result, activation.response.cmd_set,
          activation.response.cmd_id, activation.response.ack_data};
}

bool
takePicture()
{
  dji_sdk::CameraAction cameraAction;
  cameraAction.request.camera_action = 0;
  camera_action_service.call(cameraAction);
  return cameraAction.response.result;
}

bool
startVideo()
{
  dji_sdk::CameraAction cameraAction;
  cameraAction.request.camera_action = 1;
  camera_action_service.call(cameraAction);
  return cameraAction.response.result;
}

bool
stopVideo()
{
  dji_sdk::CameraAction cameraAction;
  cameraAction.request.camera_action = 2;
  camera_action_service.call(cameraAction);
  return cameraAction.response.result;
}

void
gimbalAngleCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
  gimbal_angle = *msg;
}

void gimbal_camera_control_callback(const dji_sdk::GimbalCameraControlData::ConstPtr& msg)
{
  gimbal_camera_control_cmd_data.roll  = msg->roll;
  gimbal_camera_control_cmd_data.pitch = msg->pitch;
  gimbal_camera_control_cmd_data.yaw   = msg->yaw;
  gimbal_camera_control_cmd_data.action= msg->action;
  gimbal_camera_control_cmd_data.photo_number = msg->photo_number;
  gimbal_camera_control_cmd_data.video_time = msg->video_time;
  cmd_flag = true;
}
//get current angle, it's no use now, but might be used later
RotationAngle get_current_angle()
{
  ros::spinOnce();
  RotationAngle currentAngle;
  currentAngle.roll = gimbal_angle.vector.y;
  currentAngle.pitch = gimbal_angle.vector.x;
  currentAngle.yaw = gimbal_angle.vector.z;
  return currentAngle;
}
