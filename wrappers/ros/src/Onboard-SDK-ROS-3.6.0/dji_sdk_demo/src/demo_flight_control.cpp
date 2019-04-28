/** @file demo_flight_control.cpp
 *  @version 3.3
 *  @date May, 2017
 *
 *  @brief
 *  demo sample of how to use flight control APIs
 *
 *  @copyright 2017 DJI. All rights reserved.
 ii
*
 */



#include "dji_sdk_demo/demo_flight_control.h"
#include "dji_sdk/dji_sdk.h"
#include "djiosdk/dji_vehicle.hpp"
#include <dji_sdk_demo/demo_camera_gimbal.h>
#include <fstream>
#include <queue>
//#include <mutex>

using namespace std;

const float deg2rad = C_PI/180.0;
const float rad2deg = 180.0/C_PI;

bool takeoff_result;

//navigation define
#define TAG 1
#define GPS 2
#define VINS 3
#define BODY 1
#define GROUND 2
#define NAVIGATION_MODE VINS //if use tag, then TAG
#define FRAME_MODE BODY //if earth frame, then GROUND
#define LAST_TAG 350 //the tag whose id is 350 is the last one, the drone will take the last photo there and then RTL
#define POS_I_LIMIT 4.0*deg2rad
//mynteye to flight controler imu distance
#define DELTA_X -0.24
#define DELTA_Y 0.0
#define DELTA_Z 0.03

#define DEAD_ZONE 0.00 //dead zone, unit: m
#define POS_I_DEAD_ZONE 0.04 //in dead zone, don't intergrate
#define YAW_DEAD_ZONE 3
#define HEIGHT_DEAD_ZONE 0.05
#define TARGET_HEIGHT 1.2

const float I_VEL_LIMIT = 0.025; //the intergrate item limit of position control, limit the expected velocity
const float D_VEL_LIMIT = 0.015;
const float I_ANG_LIMIT = 0.5; //the intergrate item limit of velocity control, limit the expected angle
const float YAW_I_LIMIT = 2.0;
const float YAW_RATE_LIMIT = 10.0;

bool dead_zone_flag = false;

//control parameters
const float P_pos = 0.50;
const float I_pos = 0.10 ;
const float D_pos = 0.00;
const float P_vel = 6.00;
const float I_vel = 1.0;
const float D_vel = 0.00;
const float P_yaw = 0.1;
const float I_yaw = 0.00;
const float D_yaw = 0.00;
const float P_z = 1.00;

//gimbal and camera controll commands
const float GIMBAL_ROLL     = 0.0;
const float GIMBAL_PITCH    = 0.0;
const float GIMBAL_YAW      = 90.0;
const uint8_t CAMERA_ACTION = 1; //1:take photos 2:videos 3:photos and videos
const uint8_t PHOTO_NUMBER  = 1; //the number of photos to take
const uint8_t VIDEO_TIME    = 4; //the video time to take

//how long time between controls
const float delta_t = 1;

//tx2 used
//queue<nav_msgs::Odometry> curr_buf;
//queue<nav_msgs::Odometry> saved_buf;

//mutex z_buf;

ros::Publisher ctrlAttiVerPos;
ros::Publisher ctrlPosYawPub;
ros::Publisher ctrlVelYawRatePub;
ros::Publisher ctrlBrakePub;
ros::Publisher tag_position_converted;
ros::Publisher camera_gimbal_control;

ros::ServiceClient sdk_ctrl_authority_service;
ros::ServiceClient drone_task_service;
ros::ServiceClient query_version_service;
ros::ServiceClient set_local_pos_reference;
ros::ServiceClient m100_arm_service;

// global variables for subscribed topics
//*_count are counts for judgement, uint8_t
uint8_t flight_status = 255, display_mode  = 255, obtain_control_count = 0, tag_dist_count =0, tag_yaw_count = 0, tag_invalid_count = 0;
sensor_msgs::NavSatFix current_gps;
geometry_msgs::Quaternion current_atti,current_tag_atti;
geometry_msgs::Point current_local_pos, velocity_intgrated_position, tag_mission_reset;
geometry_msgs::Vector3 current_velocity, fc_attitude_rpy, tag_attitude_rpy;
geometry_msgs::Vector3 current_acceleration;
geometry_msgs::Point tag_position, convert_tag_position, error_pos_last, error_vel_last, error_pos_integrated, error_vel_integrated, attitude_expect, velocity_expected;
geometry_msgs::Pose tag_pose_pub;
float current_height;
apriltags2_ros::AprilTagDetectionArray convert_tag_detections;
uint32_t tag_id, tag_id_last;
//when detected tag ,then tag_valid is true, if tag always can't be detected, then false
bool tag_valid = false, tag_valid_flag = false;//judge if the tag data is valid
//next_tag: if or not go to next tag,   position_hold_flag: if or not hold above this tag
bool next_tag = true, position_hold_flag = false, tag_position_flag = false, gimbal_flag = false, RTL_flag = false;
bool photo_flag = false, only_once_flag = true;

geometry_msgs::Point current_point;
geometry_msgs::Point saved_point;
geometry_msgs::Quaternion current_angle;
geometry_msgs::Quaternion saved_angle;
geometry_msgs::Vector3 curr_angle;
geometry_msgs::Vector3 save_angle;
float current_yaw_angle;
float saved_yaw_angle;

Mission square_mission;

//for record
ofstream in;

float Radius = 8.0;
float test_velocity = 2.0;
float yaw_init = 0.0;
//for control
uint32_t fc_sec, fc_nsec, velocity_sec, velocity_nsec;
uint32_t tag_sec, tag_nsec;
uint32_t yaw_record_count = 0;
float yaw_average = 0;
double fc_time, fc_time_start, velocity_time = 0.0, velocity_time_last = 0.0, velocity_time_last_control =0.0; //flight controller time now
double tag_time, tag_time_last;
float error_yaw_last = 0.0, error_yaw_integrated = 0.0;
bool fc_time_valid = false, obtain_control_result = false;
bool saved_update_flag = false, current_update_flag = false;
double time_current_update = 0,time_current_update_last = 0,time_current_update_last2 = 0;

//to record the tag information, will used in RTL
vector<CodeInfo>  codes;

//gimbal camera control command
dji_sdk::GimbalCameraControlData gimbal_camera_control_data;


uint32_t cccc =0;


geometry_msgs::Point rotation_hehe(geometry_msgs::Vector3 angle)
{
    geometry_msgs::Point delta_distance;
    float Rbe[3][3] = {0};
    angle.z -= yaw_init;
    Rbe[0][0] = cos(angle.y) * cos(angle.z);
    Rbe[0][1] = cos(angle.z) * sin(angle.y) * sin(angle.x) - sin(angle.z) * cos(angle.x);
    Rbe[0][2] = cos(angle.z) * sin(angle.y) * cos(angle.x) + sin(angle.z) * sin(angle.x);
    Rbe[1][0] = cos(angle.y) * sin(angle.z);
    Rbe[1][1] = sin(angle.z) * sin(angle.y) * sin(angle.x) + cos(angle.z) * cos(angle.x);
    Rbe[1][2] = sin(angle.z) * sin(angle.y) * cos(angle.x) - cos(angle.z) * sin(angle.x);
    Rbe[2][0] = -sin(angle.y);
    Rbe[2][1] = sin(angle.x) * cos(angle.y);
    Rbe[2][2] = cos(angle.x) * cos(angle.y);
    delta_distance.x = Rbe[0][0] * DELTA_X + Rbe[0][1] * DELTA_Y + Rbe[0][2] * DELTA_Z;
    delta_distance.y = Rbe[1][0] * DELTA_X + Rbe[1][1] * DELTA_Y + Rbe[1][2] * DELTA_Z;
    delta_distance.z = Rbe[2][0] * DELTA_X + Rbe[2][1] * DELTA_Y + Rbe[2][2] * DELTA_Z;
    return delta_distance;
}

//vins subscriber 
void savedmap_callback(const nav_msgs::Odometry &saved_info)
{
  saved_point.x= saved_info.pose.pose.position.x;
  saved_point.y= saved_info.pose.pose.position.y;
  saved_point.z= saved_info.pose.pose.position.z;
  saved_angle.x= saved_info.pose.pose.orientation.x;
  saved_angle.y= saved_info.pose.pose.orientation.y;
  saved_angle.z= saved_info.pose.pose.orientation.z;
  saved_angle.w= saved_info.pose.pose.orientation.w;
  save_angle = toEulerAngle(saved_angle);
    geometry_msgs::Point rotation_hehe_data1 = rotation_hehe(save_angle);
    saved_point.x += rotation_hehe_data1.x;
    saved_point.y += rotation_hehe_data1.y;
    saved_point.z += rotation_hehe_data1.z;
  saved_yaw_angle = save_angle.z;
  ROS_INFO("saved %f %f %f %f",saved_point.x,saved_point.y,saved_point.z,saved_yaw_angle*rad2deg);

  saved_update_flag = true;
 // z_buf.lock();
  //saved_buf.push(saved_info);
 // z_buf.unlock();

}


void odometry_callback(const nav_msgs::Odometry &current_info)
{
    time_current_update = ros::Time::now().toSec();
  current_point.x= current_info.pose.pose.position.x;
  current_point.y= current_info.pose.pose.position.y;
  current_point.z= current_info.pose.pose.position.z;
  current_angle.x= current_info.pose.pose.orientation.x;
  current_angle.y= current_info.pose.pose.orientation.y;
  current_angle.z= current_info.pose.pose.orientation.z;
  current_angle.w= current_info.pose.pose.orientation.w;
  curr_angle = toEulerAngle(current_angle);
    geometry_msgs::Point rotation_hehe_data = rotation_hehe(curr_angle);
  current_point.x += rotation_hehe_data.x;
  current_point.y += rotation_hehe_data.y;
  current_point.z += rotation_hehe_data.z;
  current_yaw_angle = curr_angle.z;
  ROS_INFO("current %f %f %f",current_point.x,current_point.y,current_point.z);
//  ROS_INFO("angle   %f %f %f",curr_angle.x*rad2deg,curr_angle.y*rad2deg,curr_angle.z*rad2deg);
  current_update_flag = true;
 // z_buf.lock();C
  //curr_buf.push(current_info);
 // z_buf.unlock();

}




int main(int argc, char** argv)
{
  ros::init(argc, argv, "demo_flight_control_node");
  ros::NodeHandle nh;

  // Subscribe to messages from dji_sdk_node
  ros::Subscriber attitudeSub     = nh.subscribe("dji_sdk/attitude", 100, &attitude_callback);
  ros::Subscriber gpsSub          = nh.subscribe("dji_sdk/gps_position", 100, &gps_callback);
  ros::Subscriber flightStatusSub = nh.subscribe("dji_sdk/flight_status", 100, &flight_status_callback);
  ros::Subscriber displayModeSub  = nh.subscribe("dji_sdk/display_mode", 100, &display_mode_callback);
  ros::Subscriber localPosition   = nh.subscribe("dji_sdk/local_position", 100, &local_position_callback);
  ros::Subscriber TagPosition     = nh.subscribe("tag_detections",100,&tag_position_callback) ;
  ros::Subscriber height          = nh.subscribe("dji_sdk/height_above_takeoff",100,&height_callback);
  ros::Subscriber velocity        = nh.subscribe("dji_sdk/velocity",100,&velocity_callback);
  ros::Subscriber gimbal_answer   = nh.subscribe("gimbal_camera_answer",100,&gimbal_answer_callback);
  ros::Subscriber imu             = nh.subscribe("dji_sdk/imu",100,imu_callback);




  //subscribe to tx2  vins estimator node 
  ros::Subscriber odometrysub     = nh.subscribe("vins_estimator/odometry",100,odometry_callback);
  ros::Subscriber savedmapsub     = nh.subscribe("vins_estimator/saved_keyframe",100,savedmap_callback);





  // Publish the control signal
  ctrlAttiVerPos = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_rollpitch_yawrate_zposition",100);
  ctrlPosYawPub = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_ENUposition_yaw", 100);
  ctrlVelYawRatePub = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_ENUvelocity_yawrate", 100);
  // We could use dji_sdk/flight_control_setpoint_ENUvelocity_yawrate here
  ctrlBrakePub = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_generic", 100);
  //publish the converted tag position
  //tag_position_converted = nh.advertise<apriltags2_ros::AprilTagDetectionArray>("convert_tag_detections", 10);
  tag_position_converted = nh.advertise<geometry_msgs::Pose>("convert_tag_detections", 100);
  camera_gimbal_control  = nh.advertise<dji_sdk::GimbalCameraControlData>("gimbal_camera_control_cmd",100);

  // Basic services
  sdk_ctrl_authority_service = nh.serviceClient<dji_sdk::SDKControlAuthority> ("dji_sdk/sdk_control_authority");
  drone_task_service         = nh.serviceClient<dji_sdk::DroneTaskControl>("dji_sdk/drone_task_control");
  query_version_service      = nh.serviceClient<dji_sdk::QueryDroneVersion>("dji_sdk/query_drone_version");
  set_local_pos_reference    = nh.serviceClient<dji_sdk::SetLocalPosRef> ("dji_sdk/set_local_pos_ref");
  m100_arm_service           = nh.serviceClient<dji_sdk::DroneArmControl> ("dji_sdk/drone_arm_control");

  in.open("com.txt",ios::trunc); //ios::trunc表示在打开文件前将文件清空,由于是写入,文件不存在则创建
  in<<"time"<<"\t"<<"\t"<<"tag_roll"<<"\t"<<"tag_pitch"<<"\t"<<"tag_yaw"<<"\t"<<"tag_x"<<"\t"<<"tag_y"<<"\t"<<"tag_z"<<"\t"<<"tag_cov_x"<<"\t"<<"tag_cov_y"<<"\t"<<"tag_cov_z"<<"\n";
  velocity_intgrated_position.x = 0;
  velocity_intgrated_position.y = 0;
//  while(!current_update_flag)
//  {
//      ros::Duration(0.1).sleep();
//      ros::spinOnce();
//      ROS_INFO("getting ready...");
//  }
//  yaw_init = fc_attitude_rpy.z;

//
//  ros::spinOnce();
//  obtain_control_result = obtain_control();
  /*if (!set_local_position()) // We need this for height
  {
    ROS_ERROR("GPS health insufficient - No local frame reference for height. Exiting.");
    return 1;
  }*/
//
//  if(is_M100())
//  {
//    ROS_INFO("M100 taking off!");
//    takeoff_result = M100monitoredTakeoff();
//  }
//  else
//  {
//    ROS_INFO("A3/N3 taking off!");
//    takeoff_result = monitoredTakeoff();
//  }

  circle_central.x = 10;
  circle_central.y = 10;
//  takeoff_result = true;
//  if(takeoff_result)
//  {
    while(ros::ok())
    {
      ros::Duration(0.1).sleep();
      ros::spinOnce();
      //if(time_current_update - time_current_update_last >= 0.1)
//      {
//          if(time_current_update_last2 == 0)
//          {
//              time_current_update_last2 = time_current_update_last;
//              time_current_update_last  = time_current_update;
//              current_point_last2 = current_point_last;
//              current_point_last = current_point;
//          }
//          else
//          {
//              float vx_current = (current_point.x - current_point_last.x)/(time_current_update - time_current_update_last);
//              float vy_current = (current_point.y - current_point_last.y)/(time_current_update - time_current_update_last);
//              float vz_current = (current_point.z - current_point_last.z)/(time_current_update - time_current_update_last);
//              float vx_last = (current_point_last.x - current_point_last2.x)/(time_current_update_last - time_current_update_last2);
//              float vy_last = (current_point_last.y - current_point_last2.y)/(time_current_update_last - time_current_update_last2);
//              float vz_last = (current_point_last.z - current_point_last2.z)/(time_current_update_last - time_current_update_last2);
//              float ax = abs(2*(vx_current - vx_last)/(time_current_update-time_current_update_last2));
//              float ay = abs(2*(vy_current - vy_last)/(time_current_update-time_current_update_last2));
//              float az = abs(2*(vz_current - vz_last)/(time_current_update-time_current_update_last2));
//              ROS_INFO("ax ay az %f %f %f",ax,ay,az);
//              ROS_INFO("axfc ayfc azfc %f %f %f",current_acceleration.x,current_acceleration.y,current_acceleration.z);
//              //ROS_INFO("scale_x scale_y scale_z %f %f %f",ax/current_acceleration.x,ay/current_acceleration.y,az/current_acceleration.z);
//              //ROS_INFO("time %f",time_current_update);
//              ROS_INFO("dx dy dz %f %f %f",current_point.x - current_point_last.x,current_point.y - current_point_last.y,current_point.z - current_point_last.z);
//              ROS_INFO("dxl dyl dzl %f %f %f",current_point.x - current_point_last.x,current_point.y - current_point_last.y,current_point.z - current_point_last.z);
//              if(ax>=5.0||ay>=5.0||az>=5.0)
//              {
//                  send_cmd_roll_pitch_yawrate_verticalposition(0,0,0, current_height);
//                  continue;
//              }
//          }
//      }
      //the A3/N3 obtain mode function can be used anywhere


//      float epsiloni = 1.0;
//      float deltaxi = abs(current_point.x-saved_point.x);
//      float deltayi = abs(current_point.y-saved_point.y);
//      float deltazi = abs(current_point.z-saved_point.z);
//      if(deltaxi<=epsiloni && deltayi<=epsiloni && deltazi<=epsiloni)
//      {
//          position_pid_control(saved_point,current_point,0.5,saved_yaw_angle,0,3);
//
//      }
//      else
//      {
//          send_cmd_roll_pitch_yawrate_verticalposition(0,0,0, current_height);
//          ROS_INFO("the vins blowing");
//      }
        M100_obtain_mode();
//        obtain_control();
      if(current_update_flag && saved_update_flag)
      {
          position_pid_control(saved_point,current_point,0.2,saved_yaw_angle,0,7);
          send_cmd_xyz_velocity_yaw_rate(velocity_expected.x,velocity_expected.y,velocity_expected.z,attitude_expect.z);
          current_update_flag = false;
          saved_update_flag = false;
      }
      else if(ros::Time::now().toSec() - time_current_update >= 0.2)
      {
          send_cmd_xyz_velocity_yaw_rate(0,0,0,0);
          ROS_INFO("the vins blowing");
      }
    }
//  }
  return 0;
}



// Helper Functions

/*! Very simple calculation of local NED offset between two pairs of GPS
/coordinates. Accurate when distances are small.
!*/
void
localOffsetFromGpsOffset(geometry_msgs::Vector3&  deltaNed,
                         sensor_msgs::NavSatFix& target,
                         sensor_msgs::NavSatFix& origin)
{
  double deltaLon = target.longitude - origin.longitude;
  double deltaLat = target.latitude - origin.latitude;

  deltaNed.y = deltaLat * deg2rad * C_EARTH;
  deltaNed.x = deltaLon * deg2rad * C_EARTH * cos(deg2rad*target.latitude);
  deltaNed.z = target.altitude - origin.altitude;
}


geometry_msgs::Vector3 toEulerAngle(geometry_msgs::Quaternion quat)
{
  geometry_msgs::Vector3 ans;

  tf::Matrix3x3 R_FLU2ENU(tf::Quaternion(quat.x, quat.y, quat.z, quat.w));
  R_FLU2ENU.getRPY(ans.x, ans.y, ans.z);
  return ans;
}

bool takeoff_land(int task)
{
  dji_sdk::DroneTaskControl droneTaskControl;

  droneTaskControl.request.task = task;

  drone_task_service.call(droneTaskControl);

  if(!droneTaskControl.response.result)
  {
    ROS_ERROR("takeoff_land fail");
    return false;
  }

  return true;
}

bool obtain_control()
{
  dji_sdk::SDKControlAuthority authority;
  authority.request.control_enable=1;
  sdk_ctrl_authority_service.call(authority);

  if(!authority.response.result)
  {
    ROS_ERROR("obtain control failed!");
    return false;
  }

  return true;
}

//arm or disarm the m100
//can also be used in A3/N3, but not tested
//usage:
//1.arm:    M100_arm(dji_sdk::DroneArmControl::Request::ARM_COMMAND)
//2.disarm: M100_arm(dji_sdk::DroneArmControl::Request::DISARM_COMMAND)
bool M100_arm(uint8_t arm_disarm)
{
  dji_sdk::DroneArmControl m100_arm;
  m100_arm.request.arm = arm_disarm;
  m100_arm_service.call(m100_arm);

  if(!m100_arm.response.result)
  {
    ROS_ERROR("M100 arm failed!");
    return false;
  }
  ROS_INFO("M100 Armed");
  return true;
}

bool is_M100()
{
  dji_sdk::QueryDroneVersion query;
  query_version_service.call(query);

  if(query.response.version == DJISDK::DroneFirmwareVersion::M100_31)
  {
    return true;
  }

  return false;
}

void attitude_callback(const geometry_msgs::QuaternionStamped::ConstPtr& msg)
{
  current_atti = msg->quaternion;
  fc_attitude_rpy = toEulerAngle(current_atti);
  fc_sec = msg->header.stamp.sec;
  fc_nsec = msg->header.stamp.nsec;
  fc_time = (double)fc_sec + ((double)fc_nsec)/1e9;
  fc_time_valid = true;
  //ROS_INFO("A3 roll pitch yaw %f %f %f", fc_attitude_rpy.x * rad2deg, fc_attitude_rpy.y * rad2deg, fc_attitude_rpy.z * rad2deg);
}
void imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
    current_acceleration = msg->linear_acceleration;
}
void local_position_callback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
  current_local_pos = msg->point;
  //ROS_INFO("position x y z %f %f %f", current_local_pos.x, current_local_pos.y, current_local_pos.z);

  if (takeoff_result)
  {
    //ROS_INFO("circle");
    //rectangle_line(circle_central,4 * Radius,4 * Radius,test_velocity);
    //circle_line(circle_central,Radius,test_velocity);
  }
}

void velocity_callback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
  current_velocity = msg->vector;
  velocity_sec = msg->header.stamp.sec;
  velocity_nsec = msg->header.stamp.nsec;
  velocity_time = (double)fc_sec + ((double)fc_nsec)/1e9;
  velocity_intgrated_position.x += current_velocity.x * (velocity_time - velocity_time_last);
  velocity_intgrated_position.y += current_velocity.y * (velocity_time - velocity_time_last);
  velocity_time_last = velocity_time;
}

void height_callback(const std_msgs::Float32::ConstPtr& msg)
{
  current_height = msg->data;
}


void tag_position_callback(const apriltags2_ros::AprilTagDetectionArray::ConstPtr& msg)
{
  //record time now
  tag_time =ros::Time::now().toSec();
  //ROS_INFO("id1 %d", msg->detections[0].id);//Used to print how many tags within the camera
  if(nullptr != &(msg->detections[0]))
  {
    tag_valid = true;
    tag_position_flag = true;
    tag_id = msg->detections[0].id[0];
    current_tag_atti = msg->detections[0].pose.pose.pose.orientation;
    tag_position = msg->detections[0].pose.pose.pose.position;

    //when tag is valid, update the attitude immidiately
    //the quat of aprilTag is not correct
    float temp = current_tag_atti.x;
    current_tag_atti.x = current_tag_atti.y;
    current_tag_atti.y = current_tag_atti.z;
    current_tag_atti.z = current_tag_atti.w;
    current_tag_atti.w = temp;
    geometry_msgs::Vector3 angle, Angle = toEulerAngle(current_tag_atti);
    angle.x = -Angle.y;
    angle.y = -Angle.z;
    angle.z = Angle.x;
    tag_attitude_rpy.x = angle.x;
    tag_attitude_rpy.y = angle.y;
    tag_attitude_rpy.z = angle.z;

    //tag_attitude_rpy.y = acos(cos(angle.y)*cos(angle.z));

    //for A3 apriltag allignment
    /*
    angle.z = angle.z - yaw_bias;
    if(angle.z <= 0)
    {
      angle.z += 360.0/rad2deg;
    }
    else if(angle.z >= 360.0/rad2deg)
    {
      angle.z -= 360.0/rad2deg;
    }
    */
    temp = tag_position.x;
    tag_position.x = tag_position.y;
    tag_position.y = temp;
    //rotate the coordinate to ground frame
    float Rbe[3][3] = {0};
    Rbe[0][0] = cos(angle.y) * cos(angle.z);
    Rbe[0][1] = cos(angle.z) * sin(angle.y) * sin(angle.x) - sin(angle.z) * cos(angle.x);
    Rbe[0][2] = cos(angle.z) * sin(angle.y) * cos(angle.x) + sin(angle.z) * sin(angle.x);
    Rbe[1][0] = cos(angle.y) * sin(angle.z);
    Rbe[1][1] = sin(angle.z) * sin(angle.y) * sin(angle.x) + cos(angle.z) * cos(angle.x);
    Rbe[1][2] = sin(angle.z) * sin(angle.y) * cos(angle.x) - cos(angle.z) * sin(angle.x);
    Rbe[2][0] = -sin(angle.y);
    Rbe[2][1] = sin(angle.x) * cos(angle.y);
    Rbe[2][2] = cos(angle.x) * cos(angle.y);
    convert_tag_position.x = Rbe[0][0] * tag_position.x + Rbe[0][1] * tag_position.y + Rbe[0][2] * tag_position.z;
    convert_tag_position.y = Rbe[1][0] * tag_position.x + Rbe[1][1] * tag_position.y + Rbe[1][2] * tag_position.z;
    convert_tag_position.z = Rbe[2][0] * tag_position.x + Rbe[2][1] * tag_position.y + Rbe[2][2] * tag_position.z;

    //ROS_INFO("         body tag x y z %f %f %f", tag_position.x, tag_position.y, tag_position.z);
    //ROS_INFO("      convert tag x y z %f %f %f", convert_tag_position.x,convert_tag_position.y,convert_tag_position.z);
    //ROS_INFO("tag_atti roll pitch yaw %f %f %f",tag_attitude_rpy.x * rad2deg,tag_attitude_rpy.y * rad2deg,tag_attitude_rpy.z * rad2deg);
    //ROS_INFO(" fc_atti roll pitch yaw %f %f %f",fc_attitude_rpy.x * rad2deg,fc_attitude_rpy.y * rad2deg,fc_attitude_rpy.z * rad2deg);
    //rviz use
    geometry_msgs::PoseWithCovarianceStamped tag_pose;
    apriltags2_ros::AprilTagDetection cvt_tag_detection;
    convert_tag_detections = *msg;
    cvt_tag_detection = convert_tag_detections.detections[0];
    cvt_tag_detection.id.push_back(tag_id);
    cvt_tag_detection.pose.pose.pose.position.x    = convert_tag_position.x;
    cvt_tag_detection.pose.pose.pose.position.y    = convert_tag_position.y;
    cvt_tag_detection.pose.pose.pose.position.z    = convert_tag_position.z;
    cvt_tag_detection.pose.pose.pose.orientation.x = current_atti.x;
    cvt_tag_detection.pose.pose.pose.orientation.y = current_atti.y;
    cvt_tag_detection.pose.pose.pose.orientation.z = current_atti.z;
    cvt_tag_detection.pose.pose.pose.orientation.w = current_atti.w;
    convert_tag_detections.detections.push_back(cvt_tag_detection);
    tag_pose_pub.position.x=   convert_tag_position.x;
    tag_pose_pub.position.y    = convert_tag_position.y;
    tag_pose_pub.position.z    = convert_tag_position.z;
    tag_pose_pub.orientation.x = current_atti.x;
    tag_pose_pub.orientation.y = current_atti.y;
    tag_pose_pub.orientation.z = current_atti.z;
    tag_pose_pub.orientation.w = current_atti.w;
    convert_tag_detections.detections.push_back(cvt_tag_detection);
    //convert_tag_detections.detections[0].pose.pose.pose.position = convert_tag_position;
    //convert_tag_detections.detections[0].pose.pose.pose.orientation = current_atti;
    tag_position_converted.publish(tag_pose_pub);
  }
  else
  {
    tag_valid = false;
  }

  {
    in<<msg->header.stamp<<"\t"<<tag_attitude_rpy.x*rad2deg<<"\t"<<tag_attitude_rpy.y*rad2deg<<"\t"<<tag_attitude_rpy.z*rad2deg<<"\t"<<tag_position.x<<"\t"<<tag_position.y<<"\t"<<tag_position.z<<"\t"<<convert_tag_position.x<<"\t"<<convert_tag_position.y<<"\t"<<convert_tag_position.z<<"\n";
    //  in.close();//关闭文件
  }
}

void gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
  static ros::Time start_time = ros::Time::now();
  ros::Duration elapsed_time = ros::Time::now() - start_time;
  current_gps = *msg;
  /*
  // Down sampled to 50Hz loop
  if(elapsed_time > ros::Duration(0.02))
  {
    start_time = ros::Time::now();
    switch(square_mission.state)
    {
      case 0:
        break;

      case 1:
        if(!square_mission.finished)
        {
          square_mission.step();
        }
        else
        {
          square_mission.reset();
          square_mission.start_gps_location = current_gps;
          square_mission.start_local_position = current_local_pos;
          square_mission.setTarget(20, 0, 0, 0);
          square_mission.state = 2;
          ROS_INFO("##### Start route %d ....", square_mission.state);
        }
        break;

      case 2:
        if(!square_mission.finished)
        {
          square_mission.step();
        }
        else
        {
          square_mission.reset();
          square_mission.start_gps_location = current_gps;
          square_mission.start_local_position = current_local_pos;
          square_mission.setTarget(0, -20, 0, 0);
          square_mission.state = 3;
          ROS_INFO("##### Start route %d ....", square_mission.state);
        }
        break;
      case 3:
        if(!square_mission.finished)
        {
          square_mission.step();
        }
        else
        {
          square_mission.reset();
          square_mission.start_gps_location = current_gps;
          square_mission.start_local_position = current_local_pos;
          square_mission.setTarget(-20, 0, 0, 0);
          square_mission.state = 4;
          ROS_INFO("##### Start route %d ....", square_mission.state);
        }
        break;
      case 4:
        if(!square_mission.finished)
        {
          square_mission.step();
        }
        else
        {
          ROS_INFO("##### Mission %d Finished ....", square_mission.state);
          square_mission.state = 0;
        }
        break;
    }
  }*/
}

void flight_status_callback(const std_msgs::UInt8::ConstPtr& msg)
{
  flight_status = msg->data;
}

void display_mode_callback(const std_msgs::UInt8::ConstPtr& msg)
{
  display_mode = msg->data;
//  ROS_INFO("displaymode %d",display_mode);
}

void gimbal_answer_callback(const dji_sdk::GimbalAnswer::ConstPtr& msg)
{
  gimbal_flag = msg->action_finished;
}
/*!
 * This function demos how to use the flight_status
 * and the more detailed display_mode (only for A3/N3)
 * to monitor the take off process with some error
 * handling. Note M100 flight status is different
 * from A3/N3 flight status.
 */
bool
monitoredTakeoff()
{
  ros::Time start_time = ros::Time::now();

  if(!takeoff_land(dji_sdk::DroneTaskControl::Request::TASK_TAKEOFF)) {
    return false;
  }

  ros::Duration(0.01).sleep();
  ros::spinOnce();



//geometry_msgs
  // Step 1.1: Spin the motor
  while (flight_status != DJISDK::FlightStatus::STATUS_ON_GROUND &&
         display_mode != DJISDK::DisplayMode::MODE_ENGINE_START &&
         ros::Time::now() - start_time < ros::Duration(5)) {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  if(ros::Time::now() - start_time > ros::Duration(5)) {
    ROS_ERROR("Takeoff failed. Motors are not spinnning.");
    return false;
  }
  else {
    start_time = ros::Time::now();
    ROS_INFO("Motor Spinning ...");
    ros::spinOnce();
  }


  // Step 1.2: Get in to the air
  while (flight_status != DJISDK::FlightStatus::STATUS_IN_AIR &&
          (display_mode != DJISDK::DisplayMode::MODE_ASSISTED_TAKEOFF || display_mode != DJISDK::DisplayMode::MODE_AUTO_TAKEOFF) &&
          ros::Time::now() - start_time < ros::Duration(20)) {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  if(ros::Time::now() - start_time > ros::Duration(20)) {
    ROS_ERROR("Takeoff failed. Aircraft is still on the ground, but the motors are spinning.");
    return false;
  }
  else {
    start_time = ros::Time::now();
    ROS_INFO("Ascending...");
    ros::spinOnce();
  }

  // Final check: Finished takeoff
  while ( (display_mode == DJISDK::DisplayMode::MODE_ASSISTED_TAKEOFF || display_mode == DJISDK::DisplayMode::MODE_AUTO_TAKEOFF) &&
          ros::Time::now() - start_time < ros::Duration(20)) {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  if ( display_mode != DJISDK::DisplayMode::MODE_P_GPS || display_mode != DJISDK::DisplayMode::MODE_ATTITUDE)
  {
    ROS_INFO("Successful takeoff!");
    start_time = ros::Time::now();
  }
  else
  {
    ROS_ERROR("Takeoff finished, but the aircraft is in an unexpected mode. Please connect DJI GO.");
    return false;
  }

  return true;
}


/*!
 * This function demos how to use M100 flight_status
 * to monitor the take off process with some error
 * handling. Note M100 flight status is different
 * from A3/N3 flight status.
 */
bool
M100monitoredTakeoff()
{
  ros::Time start_time = ros::Time::now();

  float home_altitude = current_gps.altitude;
  if(!takeoff_land(dji_sdk::DroneTaskControl::Request::TASK_TAKEOFF))
  {
    return false;
  }

  ros::Duration(0.01).sleep();
  ros::spinOnce();

  // Step 1: If M100 is not in the air after 10 seconds, fail.
  while (ros::Time::now() - start_time < ros::Duration(10))
  {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  if(flight_status != DJISDK::M100FlightStatus::M100_STATUS_IN_AIR ||
      current_gps.altitude - home_altitude < 1.0)
  {
    ROS_ERROR("Takeoff failed.");
    return false;
  }
  else
  {
    start_time = ros::Time::now();
    ROS_INFO("Successful takeoff!");
  //  ros::spinOnce();
  }
  return true;
}

bool set_local_position()
{
  dji_sdk::SetLocalPosRef localPosReferenceSetter;
  set_local_pos_reference.call(localPosReferenceSetter);
  return localPosReferenceSetter.response.result;
}

//no altitude control
//dead_zone: unit m          mode: determines if use position or yaw control,1:position control 2:yaw control 3:position and yaw control
void position_pid_control(geometry_msgs::Point current_set_point,geometry_msgs::Point current_local_point,float velocity_limit,float target_yaw, float dead_zone, uint8_t mode)
{
    float vx = 0.0, vy = 0.0, vxp = 0.0, vyp = 0.0, vxi = 0.0, vyi = 0.0, vxd = 0.0, vyd = 0.0;
    float yaw_rate = 0.0, yaw_rate_p = 0.0, yaw_rate_i = 0.0, yaw_rate_d = 0.0, vz = 0;
    float roll = 0.0, pitch = 0.0;
    //position control
    if (mode & 0x01) {
        //calculate velocity, P control
        geometry_msgs::Point error_pos;
        error_pos.x = current_set_point.x - current_local_point.x;
        error_pos.y = current_set_point.y - current_local_point.y;
        vxp = P_pos * error_pos.x;
        vyp = P_pos * error_pos.y;
        vxd = D_pos * (error_pos.x - error_pos_last.x);
        vyd = D_pos * (error_pos.y - error_pos_last.y);
        vxd = limit(vxd, D_VEL_LIMIT);
        vyd = limit(vyd, D_VEL_LIMIT);
        /*if(abs(error_pos.x) < abs(error_pos_last.x) && (abs(error_pos.x) >= 1.5 * DEAD_ZONE))
        {
          vxd = 0.0;
        }
        if(abs(error_pos.y) < abs(error_pos_last.y) && (abs(error_pos.y) >= 1.5 * DEAD_ZONE))
        {
          vyd = 0.0;
        }*/
        if (abs(error_pos.x) >= POS_I_DEAD_ZONE) {
            error_pos_integrated.x += error_pos.x;
        } else {
            error_pos_integrated.x = 0.0;
        }
        if (abs(error_pos.y) >= POS_I_DEAD_ZONE) {
            error_pos_integrated.y += error_pos.y;
        } else {
            error_pos_integrated.y = 0.0;
        }
        if (I_pos > 0.0001) {
            error_pos_integrated.x = limit((float) error_pos_integrated.x, I_VEL_LIMIT / I_pos);
            error_pos_integrated.y = limit((float) error_pos_integrated.y, I_VEL_LIMIT / I_pos);
        }
        vxi = I_pos * error_pos_integrated.x;
        vyi = I_pos * error_pos_integrated.y;
        vx = vxp + vxi + vxd;
        vy = vyp + vyi + vyd;

        float x_offset = current_set_point.x - current_local_point.x;
        float y_offset = current_set_point.y - current_local_point.y;
        float distance = sqrt(x_offset * x_offset + y_offset * y_offset);
        if (distance <= dead_zone) {
            dead_zone_flag = true;
            error_pos_integrated.x = 0.0;
            error_pos_integrated.y = 0.0;
            vx = 0;
            vy = 0;
        } else {
            dead_zone_flag = false;
        }

        //limit the speed
        vx = limit_velocity(vx, vy, velocity_limit).x;
        vy = limit_velocity(vx, vy, velocity_limit).y;
        //ROS_INFO("vx_exp vy_exp %f %f",vx,vy);

        error_pos_last = error_pos;
    }
    //yaw control
    if (mode & 0x02) {
        //use the flight controller yaw in default
        float current_yaw = fc_attitude_rpy.z;
        //the attitude form tag is not correct when the drone is rotate, so we don't use it
        if (NAVIGATION_MODE == TAG) {
            current_yaw = tag_attitude_rpy.z;
        }
        if (NAVIGATION_MODE == VINS) {
            current_yaw = current_yaw_angle;
        }
        //Drone turns at the smallest angle
        float error_yaw = (target_yaw - current_yaw) * rad2deg;
        if (error_yaw < -180)
            error_yaw += 360;
        else if (error_yaw > 180)
            error_yaw -= 360;
        yaw_rate_p = P_yaw * error_yaw;
        yaw_rate_d = D_yaw * (error_yaw - error_yaw_last);
        error_yaw_integrated += error_yaw;
        if (I_yaw > 0.0001) {
            error_yaw_integrated = limit(error_yaw_integrated, YAW_I_LIMIT / I_yaw);
        }
        yaw_rate_i = I_yaw * error_yaw_integrated;
        if (abs(error_yaw) <= YAW_DEAD_ZONE) {
            yaw_rate_p = 0.0;
            yaw_rate_i = 0.0;
            yaw_rate_d = 0.0;
            error_yaw_integrated = 0.0;
        }
        yaw_rate = yaw_rate_p + yaw_rate_i + yaw_rate_d;
        if (abs(error_yaw) >= YAW_RATE_LIMIT) {
            yaw_rate = limit(yaw_rate, YAW_RATE_LIMIT);
        } else {
            yaw_rate = limit(yaw_rate, (float) (YAW_RATE_LIMIT * 0.4));
        }
        error_yaw_last = error_yaw;
    }

    //height control
    if (mode & 0x04) {
        //control the height around 1.5m, the height is from flight controller
        if (abs(current_local_point.z - current_set_point.z) >= HEIGHT_DEAD_ZONE) {
            vz = P_z * (current_set_point.z - current_local_point.z);
        } else {
            vz = 0.0;
        }
    }


    velocity_expected.x = vx * cos(curr_angle.z) + vy * sin(curr_angle.z);
    velocity_expected.y = vy * cos(curr_angle.z) - vx * sin(curr_angle.z);
    velocity_expected.z = vz;
    attitude_expect.z = yaw_rate * deg2rad;
}

//in ground frame: vx:E vy:N
geometry_msgs::Point horizon_velocity_pid_control(float vx, float vy)
{
  geometry_msgs::Point attitude, error_vel;
  float rollp, pitchp, rolli, pitchi, rolld, pitchd;
  float current_yaw_tag = tag_attitude_rpy.z, current_yaw_fc = fc_attitude_rpy.z;
  float vx_expected = 0.0, vy_expected = 0.0, vx_real = 0.0, vy_real = 0.0;

  //calculate the expected velocity and real velocity in body frame
  if(NAVIGATION_MODE == TAG)
  {
    vx_expected = vx * cos(current_yaw_tag) + vy * sin(current_yaw_tag);
    vy_expected = vy * cos(current_yaw_tag) - vx * sin(current_yaw_tag);
  }
  else
  {
    vx_expected = vx * cos(current_yaw_fc) + vy * sin(current_yaw_fc);
    vy_expected = vy * cos(current_yaw_fc) - vx * sin(current_yaw_fc);
  }
  vx_real = current_velocity.x * cos(current_yaw_fc) + current_velocity.y * sin(current_yaw_fc);
  vy_real = current_velocity.y * cos(current_yaw_fc) - current_velocity.x * sin(current_yaw_fc);

  error_vel.x = vx_expected - vx_real;
  error_vel.y = vy_expected - vy_real;

  //ROS_INFO("vx_ vy_ %f %f ",current_velocity.x,current_velocity.y);
  //ROS_INFO("erx ery %f %f ",error_vel.x,error_vel.y);

  rollp  = -P_vel * error_vel.y;
  pitchp =  P_vel * error_vel.x;

  if((velocity_time - velocity_time_last_control) <= 1.0 && (velocity_time - velocity_time_last_control) >= 0.0001)
  {
    rolld  = -D_vel * (error_vel.x - error_vel_last.x)/(velocity_time - velocity_time_last_control);
    pitchd = -D_vel * (error_vel.y - error_vel_last.y)/(velocity_time - velocity_time_last_control);
    error_vel_integrated.x -= error_vel.x * (velocity_time - velocity_time_last_control);
    error_vel_integrated.y -= error_vel.y * (velocity_time - velocity_time_last_control);
    if(I_vel > 0.001)
    {
      error_vel_integrated.x = limit(error_vel_integrated.x,2.0 / I_vel);
      error_vel_integrated.y = limit(error_vel_integrated.y,2.0 / I_vel);
    }
    rolli  = I_vel * error_vel_integrated.x;
    pitchi = I_vel * error_vel_integrated.y;
  }
  else
  {
    rolld  = 0;
    pitchd = 0;
    error_vel_integrated.x = 0;
    error_vel_integrated.y = 0;
    rolli  = 0;
    pitchi = 0;
  }
  attitude.x = (rollp  + rolli  + rolld) * deg2rad;
  attitude.y = (pitchp + pitchi + pitchd) * deg2rad;
  attitude.x = limit(attitude.x, 5.0);
  attitude.y = limit(attitude.y, 5.0);
  //ROS_INFO("ROLL p i d %f %f %f",rollp,rolli,rolld);
  //ROS_INFO("vel_x_gei vel_y_gei %f %f", vx,vy);
  //ROS_INFO("yawd error_vel.x error_vel.y %f %f", error_vel.x, error_vel.y);
  //ROS_INFO("roll_gei pitch_ge %f %f", attitude.x * rad2deg,attitude.y * rad2deg);

  error_vel_last = error_vel;
  velocity_time_last_control =velocity_time;
  //ROS_INFO("vx_error vy_error %f %f", error_vel.x,error_vel.y);
  //ROS_INFO("vx_curre vy_curre %f %f", current_velocity.x,current_velocity.y);

  return attitude;
}

float calculate_target_yaw(geometry_msgs::Point current_set_point,geometry_msgs::Point set_point)
{
  float central_offset_x  = set_point.x - current_set_point.x;
  float central_offset_y  = set_point.y - current_set_point.y;
  //calculate delta yaw angle
  float target_yaw = 0;
  //calculate target yaw
  if(std::abs(central_offset_x) < 0.000001)
  {
    if(central_offset_y >= 0)
      target_yaw = 90;
    else
      target_yaw = -90;
  }
  else if (central_offset_x > 0)
  {
    target_yaw = atan(central_offset_y / central_offset_x) * rad2deg;
  }
  else
  {
    if(central_offset_y <= 0)
      target_yaw = atan(central_offset_y / central_offset_x) * rad2deg - 180;
    else
      target_yaw = atan(central_offset_y / central_offset_x) * rad2deg + 180;
  }
  return target_yaw;
}

//hold the drone above the (0,0) of tag
void position_hold(geometry_msgs::Point tag_position, uint8_t mode)
{
  //ROS_INFO("altitude %f", convert_tag_position.z);
	geometry_msgs::Point current_set_point;
	current_set_point.x = 0;
	current_set_point.y = 0;
	current_set_point.z = 1.5;//It's OK no assigning here

	//no altitude control, the maximum velocity is 1m/s
  if(tag_position_flag == true)
  {
    position_pid_control(current_set_point,convert_tag_position,0.5,0,0,mode);
  }
  else
  {
    //no tag data, hold the position, vx vy vz and yaw rate are all equal zero
    //the velocity_limit equals zero, then the drone don't move
    position_pid_control(current_set_point,convert_tag_position,0,0,0.1,mode);
  }
}

//limit the velocity to be smaller than maximum
geometry_msgs::Point limit_velocity(float vx, float vy,float maximum)
{
	geometry_msgs::Point vel;
	float velocity = sqrt(vx * vx + vy * vy);
	if(maximum <= 0)
	{
		vel.x = 0;
		vel.y = 0;
	}
	if(velocity <= maximum)
	{
		vel.x = vx;
		vel.y = vy;
	}
	//if velocity is bigger than maximum, then limit the vx and vy, and keep the direction meanwhile.
	else
	{
		//the velocity must not be zero when in this step
		vel.x = vx / velocity * maximum;
		vel.y = vy / velocity * maximum;
	}
	return vel;
}

void tag_mission()
{
	if(tag_valid)
	{
    ROS_INFO("tag");

    //detect new tag, hold the position above
    if(tag_id != tag_id_last)
    {
      next_tag = false;
      photo_flag = false;
      position_hold_flag = true;
      //only set the photo_flag and store vectors once every tag
      only_once_flag = true;
    }

    //position hold
    if(position_hold_flag)
    {
      float distance_2_tag_origin = sqrt(convert_tag_position.x * convert_tag_position.x + convert_tag_position.y * convert_tag_position.y);
      if (distance_2_tag_origin <= 0.1)
      {
        tag_dist_count++;
      }
      if(abs(tag_attitude_rpy.z * rad2deg) <= 5.0)
      {
        tag_yaw_count++;
      }

      //judge what to controll
      //1:position 2:yaw 4:height, if more than one, then add the number
      if(tag_dist_count <= 2)
      {
        yaw_record_count++;
        yaw_average += (tag_attitude_rpy.z - yaw_average)/yaw_record_count;
        //the position is not on the origin ,then control the position first
        position_hold(convert_tag_position,1);
      }
      else
      {
        if(tag_id < 350)
        {
          yaw_record_count = 0;
          //then adjust the position and yaw
          position_hold(convert_tag_position,3);
          if(only_once_flag == true)
          {
            if(tag_id >= 250)
            {
              photo_flag = true;
            }
            codes.push_back(CodeInfo(tag_id, yaw_average));
            //control the gimbal and camera
            only_once_flag = false;
          }
        }
        else
        {
          //if tag_id = 350, only control the position, not control the yaw, and take photos
          position_hold(convert_tag_position,1);
          if(only_once_flag == true)
          {
            //control the gimbal and camera
            photo_flag = true;
            only_once_flag = false;
          }
        }
      }

      ROS_INFO("yaw_count %d ", tag_yaw_count);
      ROS_INFO("distance_count %d ", tag_dist_count);
    }
    if(photo_flag)
    {
      gimbal_camera_control_data.roll         = GIMBAL_ROLL;
      gimbal_camera_control_data.pitch        = GIMBAL_PITCH;
      gimbal_camera_control_data.yaw          = GIMBAL_YAW;
      gimbal_camera_control_data.action       = CAMERA_ACTION;
      gimbal_camera_control_data.photo_number = PHOTO_NUMBER;
      gimbal_camera_control_data.video_time   = VIDEO_TIME;
      camera_gimbal_control.publish(gimbal_camera_control_data);
      photo_flag = false;
    }
    ros::spinOnce();
    if(position_hold_flag && tag_dist_count >= 5 && tag_yaw_count >= 5)
    {
      if(tag_id < 250)
      {
        tag_dist_count = 0;
        tag_yaw_count = 0;
        next_tag = true;
        position_hold_flag = false;
      }
      else if (tag_id < 350)
      {
        if(gimbal_flag == true)
        {
          tag_dist_count = 0;
          tag_yaw_count = 0;
          next_tag = true;
          position_hold_flag = false;
          gimbal_flag = false;
        }
      }
      else
      {
        if (gimbal_flag == true)
        {
          tag_dist_count = 0;
          tag_yaw_count = 0;
          RTL_flag = true;
          position_hold_flag = false;
          gimbal_flag = false;
        }
      }

      tag_mission_reset = velocity_intgrated_position;
    }

    tag_id_last = tag_id;
	}
	else
  {
    ROS_INFO("no tag");
  }
  if(next_tag)
  {
    float dist_2_last_tag = sqrt((velocity_intgrated_position.x - tag_mission_reset.x) * (velocity_intgrated_position.x - tag_mission_reset.x) + (velocity_intgrated_position.y - tag_mission_reset.y) * (velocity_intgrated_position.y - tag_mission_reset.y));
    if(dist_2_last_tag <= 2.5)
    {
      send_cmd_xyz_velocity_yaw_rate(0.5, 0, 0, 0);
    }
    else
    {
      send_cmd_xyz_velocity_yaw_rate(0.1, 0, 0, 0);
    }
  }
}

void tag_RTL()
{
  //return to launch
  //....
  //takeoff_land(dji_sdk::DroneTaskControl::Request::TASK_LAND);
}

//the angle in rad and the length in meter
void send_cmd_roll_pitch_yawrate_verticalposition(float roll, float pitch, float yawrate, float ver_pos)
{
  sensor_msgs::Joy controlAttiVerPos;
  controlAttiVerPos.axes.push_back(roll);
  controlAttiVerPos.axes.push_back(pitch);
  controlAttiVerPos.axes.push_back(ver_pos);
  controlAttiVerPos.axes.push_back(yawrate);

  ctrlAttiVerPos.publish(controlAttiVerPos);

}
void send_cmd_xyz_velocity_yaw_rate(float vx, float vy, float vz, float yaw_rate)
{
  sensor_msgs::Joy controlVelYawRate;
  uint8_t flag;
  if(FRAME_MODE == BODY)
  {
    flag = (DJISDK::VERTICAL_VELOCITY   |
            DJISDK::HORIZONTAL_VELOCITY |
            DJISDK::YAW_RATE            |
            DJISDK::HORIZONTAL_BODY     |
            DJISDK::STABLE_ENABLE);

    float current_yaw = tag_attitude_rpy.z * rad2deg;
    float vx_temp = vx, vy_temp = vy;
    vx = vx_temp * cos(current_yaw / rad2deg) + vy_temp * sin(current_yaw / rad2deg);
    vy = vy_temp * cos(current_yaw / rad2deg) - vx_temp * sin(current_yaw / rad2deg);
  }
  else if(FRAME_MODE == GROUND)
  {
    flag = (DJISDK::VERTICAL_VELOCITY   |
            DJISDK::HORIZONTAL_VELOCITY |
            DJISDK::YAW_RATE            |
            DJISDK::HORIZONTAL_GROUND   |
            DJISDK::STABLE_ENABLE);
  }
  controlVelYawRate.axes.push_back(vx);
  controlVelYawRate.axes.push_back(vy);
  controlVelYawRate.axes.push_back(vz);
  controlVelYawRate.axes.push_back(yaw_rate);
  controlVelYawRate.axes.push_back(flag);
  ctrlBrakePub.publish(controlVelYawRate);

}
void send_cmd_xyz_position_yaw_angle(float x, float y, float z, float yaw)
{
  sensor_msgs::Joy controlPosYaw;

  controlPosYaw.axes.push_back(x);
  controlPosYaw.axes.push_back(y);
  controlPosYaw.axes.push_back(z);
  controlPosYaw.axes.push_back(yaw);
  ctrlPosYawPub.publish(controlPosYaw);
}
/*********************************************************
 *
 *for test, probably useful
 *
 ********************************************************/
void forward_backwack_vel(void)
{
  std::cout
    << "| Available commands:                                            |"
    << std::endl;
  std::cout
    << "| [f] fly forward by velocity control                            |"
    << "| [b] fly backward by velocity control                           |"
    << std::endl;
  char inputChar;
  std::cin >> inputChar;

  switch(inputChar)
  {
    case 'f':
    {
      //if(false == forward_init)
      {
        ros::spinOnce();
        fc_time_start = fc_time;
      }
      while(ros::ok() && fc_time - fc_time_start <= 6.0)
      {
        ros::Duration(0.01).sleep();
        ros::spinOnce();
        send_cmd_xyz_velocity_yaw_rate(0.5,0,0,0);
        ROS_INFO("forward %f", fc_time - fc_time_start);
      }
      break;
    }
    case 'b':
    {
      //if(false == forward_init)
      {
        ros::spinOnce();
        fc_time_start = fc_time;
      }
      while(ros::ok() && fc_time - fc_time_start <= 6.0)
      {
        ros::Duration(0.01).sleep();
        ros::spinOnce();
        send_cmd_xyz_velocity_yaw_rate(-0.5,0,0,0);
        ROS_INFO("backward %f", fc_time - fc_time_start);
      }
      break;
    }
    default:
      break;
  }
}
void forward_backward_position(void)
{
  geometry_msgs::Point current_set_point_forward, current_set_point_backward;
  current_set_point_forward.x   = 3.0;
  current_set_point_forward.y   = 0.0;
  current_set_point_forward.z   = 1.5;
  current_set_point_backward.x  = -3.0;
  current_set_point_backward.y  = 0.0;
  current_set_point_backward.z  = 1.5;

  std::cout
    << "| Available commands:                                            |"
    << std::endl;
  std::cout
    << "| [f] fly forward by position control                            |"
    << "| [b] fly backward by position control                           |"
    << std::endl;
  char inputChar;
  std::cin >> inputChar;

  switch(inputChar)
  {
    case 'f':
    {
      ros::spinOnce();
      fc_time_start = fc_time;
      while(ros::ok() && fc_time - fc_time_start <= 13.0)
      {
        ros::Duration(0.01).sleep();
        ros::spinOnce();
        //position_pid_control(current_set_point_forward, velocity_intgrated_position,1.0,0.0,0.0,3);
        position_pid_control(current_set_point_forward, velocity_intgrated_position,1.0,fc_attitude_rpy.z * rad2deg,0.0,3);
        send_cmd_xyz_position_yaw_angle(2,0,1.2,fc_attitude_rpy.z);
        ROS_INFO("forward time x y %f %f %f", fc_time - fc_time_start, velocity_intgrated_position.x, velocity_intgrated_position.y);
      }
      break;
    }
    case 'b':
    {
      ros::spinOnce();
      fc_time_start = fc_time;
      while(ros::ok() && fc_time - fc_time_start <= 13.0)
      {
        ros::Duration(0.01).sleep();
        ros::spinOnce();
        position_pid_control(current_set_point_backward, velocity_intgrated_position,1.0,fc_attitude_rpy.z * rad2deg,0.0,3);
        //send_cmd_xyz_position_yaw_angle(-2,0,1.2,fc_attitude_rpy.z);
        ROS_INFO("backward time x y %f %f %f", fc_time - fc_time_start, velocity_intgrated_position.x, velocity_intgrated_position.y);
      }
      break;
    }
    default:
      break;
  }
}

void M100_obtain_mode()
{
  obtain_control_count++;
  if(obtain_control_count >= 25)
  {
    obtain_control_result = obtain_control();
    obtain_control_count = 0;
//    if(obtain_control_result)
//     ROS_INFO("bangbangda ");
  }
}

void A3_N3_obtain_mode()
{
  //change modes
  if(6 == display_mode && true != obtain_control_result)
  {
    obtain_control_result = obtain_control();
//    if(obtain_control_result)
//        ROS_INFO("success!!!");
  }
  else
  {
    obtain_control_result = false;
//      ROS_INFO("failed!!!");
  }
}

/******************************************************************
 *
 * the codes below is no use here
 *
 *
 *****************************************************************/

void Mission::step()
{
  static int info_counter = 0;
  geometry_msgs::Vector3     localOffset;

  float speedFactor         = 2;
  float yawThresholdInDeg   = 2;

  float xCmd, yCmd, zCmd;

  localOffsetFromGpsOffset(localOffset, current_gps, start_gps_location);

  double xOffsetRemaining = target_offset_x - localOffset.x;
  double yOffsetRemaining = target_offset_y - localOffset.y;
  double zOffsetRemaining = target_offset_z - localOffset.z;

  double yawDesiredRad     = deg2rad * target_yaw;
  double yawThresholdInRad = deg2rad * yawThresholdInDeg;
  double yawInRad          = toEulerAngle(current_atti).z;

  info_counter++;
  if(info_counter > 25)
  {
    info_counter = 0;
    ROS_INFO("-----x=%f, y=%f, z=%f, yaw=%f ...", localOffset.x,localOffset.y, localOffset.z,yawInRad);
    ROS_INFO("+++++dx=%f, dy=%f, dz=%f, dyaw=%f ...", xOffsetRemaining,yOffsetRemaining, zOffsetRemaining,yawInRad - yawDesiredRad);
  }
  if (abs(xOffsetRemaining) >= speedFactor)
    xCmd = (xOffsetRemaining>0) ? speedFactor : -1 * speedFactor;
  else
    xCmd = xOffsetRemaining;

  if (abs(yOffsetRemaining) >= speedFactor)
    yCmd = (yOffsetRemaining>0) ? speedFactor : -1 * speedFactor;
  else
    yCmd = yOffsetRemaining;

  zCmd = start_local_position.z + target_offset_z;


  /*!
   * @brief: if we already started breaking, keep break for 50 sample (1sec)
   *         and call it done, else we send normal command
   */

  if (break_counter > 50)
  {
    ROS_INFO("##### Route %d finished....", state);
    finished = true;
    return;
  }
  else if(break_counter > 0)
  {
    sensor_msgs::Joy controlVelYawRate;
    uint8_t flag = (DJISDK::VERTICAL_VELOCITY   |
                DJISDK::HORIZONTAL_VELOCITY |
                DJISDK::YAW_RATE            |
                DJISDK::HORIZONTAL_GROUND   |
                DJISDK::STABLE_ENABLE);
    controlVelYawRate.axes.push_back(0);
    controlVelYawRate.axes.push_back(0);
    controlVelYawRate.axes.push_back(0);
    controlVelYawRate.axes.push_back(0);
    controlVelYawRate.axes.push_back(flag);

    ctrlBrakePub.publish(controlVelYawRate);
    break_counter++;
    return;
  }
  else //break_counter = 0, not in break stage
  {
    sensor_msgs::Joy controlPosYaw;


    controlPosYaw.axes.push_back(xCmd);
    controlPosYaw.axes.push_back(yCmd);
    controlPosYaw.axes.push_back(zCmd);
    controlPosYaw.axes.push_back(yawDesiredRad);
    ctrlPosYawPub.publish(controlPosYaw);
  }

  if (std::abs(xOffsetRemaining) < 0.5 &&
      std::abs(yOffsetRemaining) < 0.5 &&
      std::abs(zOffsetRemaining) < 0.5 &&
      std::abs(yawInRad - yawDesiredRad) < yawThresholdInRad)
  {
    //! 1. We are within bounds; start incrementing our in-bound counter
    inbound_counter ++;
  }
  else
  {
    if (inbound_counter != 0)
    {
      //! 2. Start incrementing an out-of-bounds counter
      outbound_counter ++;
    }
  }

  //! 3. Reset withinBoundsCounter if necessary
  if (outbound_counter > 10)
  {
    ROS_INFO("##### Route %d: out of bounds, reset....", state);
    inbound_counter  = 0;
    outbound_counter = 0;
  }

  if (inbound_counter > 50)
  {
    ROS_INFO("##### Route %d start break....", state);
    break_counter = 1;
  }

}

//Flying a circle air line, gx,gy,gz is the central point, gz is not used now
void circle_line(geometry_msgs::Point circle_central, float radius, float velocity)
{

  //  the parameters should be checked here


  if(velocity > radius )
  {
    //fastest 2 pi seconds finish a circle
    velocity = radius;
  }

  float central_offset_x  = circle_central.x - current_local_pos.x;
  float central_offset_y  = circle_central.y - current_local_pos.y;
  float central_offset_z  = 0.0;
  float dist_to_center    = sqrt(central_offset_x * central_offset_x + central_offset_y * central_offset_y);
  //calculate the crossover point
    //calculate unit vector, here don't consider the occation of dist_to_center equals zero
  float central_offset_x_unit = central_offset_x/dist_to_center;
  float central_offset_y_unit = central_offset_y/dist_to_center;
  float central_offset_z_unit = 0.0;
  float cross_x = circle_central.x - central_offset_x_unit * radius;
  float cross_y = circle_central.y - central_offset_y_unit * radius;
  float cross_offset_x = cross_x - current_local_pos.x;
  float cross_offset_y = cross_y - current_local_pos.y;
  float dist_to_cross = sqrt(cross_offset_x * cross_offset_x + cross_offset_x * cross_offset_x);

  //Firstly,calculate current setpoint according to the gx,gy,gz and current local position

  if(!drone_on_circle )
  {
      if(!inited)
      {
        start_point = current_local_pos;
        inited = true;
      }
    //Firstly, judging if the drone is on the circle
    if(dist_to_cross >= 0.2)
    {
      current_set_point.x = cross_x;
      current_set_point.y = cross_y;
      current_set_point.z = 0;
      target_yaw = calculate_target_yaw(start_point,current_set_point);
      //go to the nearest point on the circle
      position_pid_control(current_set_point,current_local_pos,velocity,target_yaw,0,3);
    }
    else
    {
      rcount++;
      if(rcount >= 100)
      {
        drone_on_circle = true;
        rcount = 0;
      }
    }
  }
  else
  {
    //calculate current setpoint
    float unit_vertical_x = -central_offset_y_unit;
    float unit_vertical_y = central_offset_x_unit;
    float vertical_x = velocity * delta_t * unit_vertical_x;
    float vertical_y = velocity * delta_t * unit_vertical_y;
    current_set_point.x = current_local_pos.x + vertical_x;
    current_set_point.y = current_local_pos.y + vertical_y;
    //now the surrent_set_point is not on the circle, there need some process
    float central_to_circle_x       = current_set_point.x - circle_central.x;
    float central_to_circle_y       = current_set_point.y - circle_central.y;
    float dist_central_to_circle    = sqrt(central_to_circle_x * central_to_circle_x + central_to_circle_y * central_to_circle_y);
    float unit_central_to_circle_x  = central_to_circle_x / dist_central_to_circle;
    float unit_central_to_circle_y  = central_to_circle_y / dist_central_to_circle;
    current_set_point.x = circle_central.x + unit_central_to_circle_x * radius;
    current_set_point.y = circle_central.y + unit_central_to_circle_y * radius;
    target_yaw = calculate_target_yaw(current_set_point, circle_central);
    //go to the next point
    position_pid_control(current_set_point,current_local_pos,velocity,target_yaw,0,3);
  }
}
//Flying a rectangle air line, gx,gy,gz is the central point, gz is not used now
void rectangle_line(geometry_msgs::Point set_point, float length, float width, float velocity)
{
  float current_to_point_x = 0, current_to_point_y = 0, dist_current_to_point = 0;
  //calculate rectangular's four vertices
  geometry_msgs::Point rectangle[4];
  rectangle[0].x = set_point.x - length * 0.5;
  rectangle[0].y = set_point.y - width * 0.5;
  rectangle[1].x = set_point.x + length * 0.5;
  rectangle[1].y = set_point.y - width * 0.5;
  rectangle[2].x = set_point.x + length * 0.5;
  rectangle[2].y = set_point.y + width * 0.5;
  rectangle[3].x = set_point.x - length * 0.5;
  rectangle[3].y = set_point.y + width * 0.5;

  //follow the line from the first point
  switch (stage)
  {
  //first stage, go to the first point from current point
    case 0:
      if(!inited)
      {
        start_point = current_local_pos;
        inited = true;
      }
      target_yaw  = calculate_target_yaw(start_point, rectangle[0]);
      position_pid_control(rectangle[0],current_local_pos,velocity,target_yaw,0,3);
      current_to_point_x   = rectangle[0].x - current_local_pos.x;
      current_to_point_y   = rectangle[0].y - current_local_pos.y;
      dist_current_to_point = sqrt(current_to_point_x * current_to_point_x + current_to_point_y * current_to_point_y);
      position_pid_control(rectangle[0],current_local_pos,velocity,target_yaw,0,3);
      if(dist_current_to_point <= 0.1)
        rcount++;
      if(rcount >= 100)
      {
        stage = 1;
        rcount = 0;
        inited = false;
      }
      break;
    case 1:
      if(!inited)
      {
        start_point = rectangle[0];
        inited = true;
      }
      target_yaw  = calculate_target_yaw(start_point, rectangle[1]);
      position_pid_control(rectangle[1],current_local_pos,velocity,target_yaw,0,3);
      current_to_point_x   = rectangle[1].x - current_local_pos.x;
      current_to_point_y   = rectangle[1].y - current_local_pos.y;
      dist_current_to_point = sqrt(current_to_point_x * current_to_point_x + current_to_point_y * current_to_point_y);
      position_pid_control(rectangle[1],current_local_pos,velocity,target_yaw,0,3);
      if(dist_current_to_point <= 0.1)
        rcount++;
      if(rcount >= 100)
      {
        stage = 2;
        rcount = 0;
        inited = false;
      }
      break;
    case 2:
      if(!inited)
      {
        start_point = rectangle[1];
        inited = true;
      }
      target_yaw  = calculate_target_yaw(start_point, rectangle[2]);
      position_pid_control(rectangle[2],current_local_pos,velocity,target_yaw,0,3);
      current_to_point_x   = rectangle[2].x - current_local_pos.x;
      current_to_point_y   = rectangle[2].y - current_local_pos.y;
      dist_current_to_point = sqrt(current_to_point_x * current_to_point_x + current_to_point_y * current_to_point_y);
      position_pid_control(rectangle[2],current_local_pos,velocity,target_yaw,0,3);
      if(dist_current_to_point <= 0.1)
        rcount++;
      if(rcount >= 100)
      {
        stage = 3;
        rcount = 0;
        inited = false;
      }
      break;
    case 3:
      if(!inited)
      {
        start_point = rectangle[2];
        inited = true;
      }
      target_yaw  = calculate_target_yaw(start_point, rectangle[3]);
      position_pid_control(rectangle[3],current_local_pos,velocity,target_yaw,0,3);
      current_to_point_x   = rectangle[3].x - current_local_pos.x;
      current_to_point_y   = rectangle[3].y - current_local_pos.y;
      dist_current_to_point = sqrt(current_to_point_x * current_to_point_x + current_to_point_y * current_to_point_y);
      position_pid_control(rectangle[3],current_local_pos,velocity,target_yaw,0,3);
      if(dist_current_to_point <= 0.1)
        rcount++;
      if(rcount >= 100)
      {
        stage = 4;
        rcount = 0;
        inited = false;
      }
      break;
    case 4:
      if(!inited)
      {
        start_point = rectangle[3];
        inited = true;
      }
      target_yaw  = calculate_target_yaw(start_point, rectangle[0]);
      position_pid_control(rectangle[0],current_local_pos,velocity,target_yaw,0,3);
      current_to_point_x   = rectangle[0].x - current_local_pos.x;
      current_to_point_y   = rectangle[0].y - current_local_pos.y;
      dist_current_to_point = sqrt(current_to_point_x * current_to_point_x + current_to_point_y * current_to_point_y);
      position_pid_control(rectangle[0],current_local_pos,velocity,target_yaw,0,3);
      if(dist_current_to_point <= 0.1)
        rcount++;
      if(rcount >= 100)
      {
        stage = 1;
        rcount = 0;
        inited = false;
      }
      break;
    default:
        stage = 0;
        rcount = 0;
        inited = false;
      break;
  }
}
void init_yaw_alignment()
{
  std::cout<< "| Available commands:    |"  << std::endl;
  std::cout<< "| [y/n] 'y' to execuse yaw alignment, please be sure the heading of the drone and the two-dimensional code is consistent |" << std::endl;
  char inputChar;
  std::cin >> inputChar;

  switch(inputChar)
  {
    case 'y':
    {
      yaw_alignment();
      break;
    }
    default:
      break;
  }
}

void yaw_alignment()
{
  ros::spinOnce();
  //yaw_bias: the angle between two-dimensional code and flight control
  //the first usage of yaw_bias: target_yaw = yaw_bias, to follow the direction of two-dimensional code
  //the second usage of yaw_bias: angle.z = fc_attitude_rpy.z - yaw_bias, for rotation matrix
  yaw_bias = fc_attitude_rpy.z - tag_attitude_rpy.z;
  ROS_INFO("yaw bias %f", yaw_bias * rad2deg);
}

void commandTest()
{
  uint8_t vel_count = 0;
  std::cout<< "| Available commands:    |"  << std::endl;
  std::cout<< "| [a/b] 'a' to roll +, 'b' to pitch + |" << std::endl;
  char inputChar;
  std::cin >> inputChar;

  switch(inputChar)
  {
    case 'a':
    {
      while(ros::ok()&&cccc<=500)
      {
        ros::Duration(0.01).sleep();
        ros::spinOnce();
        //geometry_msgs::Point attitude = horizon_velocity_pid_control(1,0);
        //send_cmd_roll_pitch_yawrate_verticalposition(attitude.x,attitude.y,0,2.0);
        send_cmd_roll_pitch_yawrate_verticalposition(3.0*deg2rad,0,0,2.0);
        cccc++;
        if(vel_count >= 10)
        {
          ROS_INFO("vx vy %f %f",current_velocity.x, current_velocity.y);
          vel_count = 0;
        }
        vel_count++;
      }
      cccc = 0;
      break;
    }
    case 'b':
    {
      while(ros::ok()&&cccc<=500)
      {
        ros::Duration(0.01).sleep();
        ros::spinOnce();
        //geometry_msgs::Point attitude = horizon_velocity_pid_control(0,1);
        //send_cmd_roll_pitch_yawrate_verticalposition(attitude.x,attitude.y,0,2.0);
        send_cmd_roll_pitch_yawrate_verticalposition(0,3.0*deg2rad,0,2.0);
        cccc++;
        if(vel_count >= 10)
        {
          ROS_INFO("vx vy %f %f",current_velocity.x, current_velocity.y);
          vel_count = 0;
        }
        vel_count++;
      }
      cccc = 0;
      break;
    }
    case 'x':
    {
      while(ros::ok()&&cccc<=500)
      {
        ros::Duration(0.01).sleep();
        ros::spinOnce();
        geometry_msgs::Point attitude = horizon_velocity_pid_control(1,0);
        send_cmd_roll_pitch_yawrate_verticalposition(attitude.x,attitude.y,0,2.0);
        cccc++;
        if(vel_count >= 10)
        {
          ROS_INFO("vx vy %f %f",current_velocity.x, current_velocity.y);
          vel_count = 0;
        }
        vel_count++;
      }
      cccc = 0;
      break;
    }
    case 'y':
    {
      while(ros::ok()&&cccc<=500)
      {

        ros::Duration(0.01).sleep();
        ros::spinOnce();
        geometry_msgs::Point attitude = horizon_velocity_pid_control(0,1);
        send_cmd_roll_pitch_yawrate_verticalposition(attitude.x,attitude.y,0,2.0);
        cccc++;
        if(vel_count >= 10)
        {
          ROS_INFO("vx vy %f %f",current_velocity.x, current_velocity.y);
          vel_count = 0;
        }
        vel_count++;
      }
      cccc = 0;
      break;
    }
    default:
      break;
  }
}
