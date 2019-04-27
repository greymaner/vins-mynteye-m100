/** @file demo_flight_control.h
 *  @version 3.3
 *  @date May, 2017
 *
 *  @brief
 *  demo sample of how to use flight control APIs
 *
 *  @copyright 2017 DJI. All rights reserved.
 *
 */

#ifndef DEMO_FLIGHT_CONTROL_H
#define DEMO_FLIGHT_CONTROL_H

#include <iostream>
#include <vector>
#include <algorithm>
// ROS includes
#include <ros/ros.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt8.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

// DJI SDK includes
#include <dji_sdk/DroneTaskControl.h>
#include <dji_sdk/SDKControlAuthority.h>
#include <dji_sdk/QueryDroneVersion.h>
#include <dji_sdk/SetLocalPosRef.h>
#include <dji_sdk/GimbalCameraControlData.h>
#include <dji_sdk/GimbalAnswer.h>
#include <dji_sdk/DroneArmControl.h>

#include <tf/tf.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Imu.h>

#include "apriltags2_ros/AprilTagDetection.h"
#include "apriltags2_ros/AprilTagDetectionArray.h"

//gimbal control
#include <dji_sdk_demo/demo_camera_gimbal.h>

#define C_EARTH (double)6378137.0
#define C_PI (double)3.141592653589793
#define DEG2RAD(DEG) ((DEG) * ((C_PI) / (180.0)))

/*!
 * @brief a bare bone state machine to track the stage of the mission
 */
class Mission
{
public:
  // The basic state transition flow is:
  // 0---> 1 ---> 2 ---> ... ---> N ---> 0
  // where state 0 means the mission is note started
  // and each state i is for the process of moving to a target point.
  int state;

  int inbound_counter;
  int outbound_counter;
  int break_counter;

  float target_offset_x;
  float target_offset_y;
  float target_offset_z;
  float target_yaw;
  sensor_msgs::NavSatFix start_gps_location;
  geometry_msgs::Point start_local_position;

  bool finished;

  Mission() : state(0), inbound_counter(0), outbound_counter(0), break_counter(0),
              target_offset_x(0.0), target_offset_y(0.0), target_offset_z(0.0),
              finished(false)
  {
  }

  void step();

  void savedmap_callback(const nav_msgs::Odometry &saved_info);
  void odometry_callback(const nav_msgs::Odometry &current_info);

  void setTarget(float x, float y, float z, float yaw)
  {
    target_offset_x = x;
    target_offset_y = y;
    target_offset_z = z;
    target_yaw      = yaw;
  }

  void reset()
  {
    inbound_counter = 0;
    outbound_counter = 0;
    break_counter = 0;
    finished = false;
  }

};

void localOffsetFromGpsOffset(geometry_msgs::Vector3&  deltaNed,
                         sensor_msgs::NavSatFix& target,
                         sensor_msgs::NavSatFix& origin);

geometry_msgs::Vector3 toEulerAngle(geometry_msgs::Quaternion quat);

void display_mode_callback(const std_msgs::UInt8::ConstPtr& msg);

void flight_status_callback(const std_msgs::UInt8::ConstPtr& msg);

void gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg);

void attitude_callback(const geometry_msgs::QuaternionStamped::ConstPtr& msg);

void imu_callback(const sensor_msgs::Imu::ConstPtr& msg);

void local_position_callback(const geometry_msgs::PointStamped::ConstPtr& msg);

void velocity_callback(const geometry_msgs::Vector3Stamped::ConstPtr& msg);

void height_callback(const std_msgs::Float32::ConstPtr& msg);

void tag_position_callback(const apriltags2_ros::AprilTagDetectionArray::ConstPtr& msg);

void gimbal_answer_callback(const dji_sdk::GimbalAnswer::ConstPtr& msg);

bool takeoff_land(int task);

bool obtain_control();

bool is_M100();

bool monitoredTakeoff();

bool M100monitoredTakeoff();

bool set_local_position();

bool M100_arm(uint8_t arm_disarm);

/**************************************************************************
 *
 *the circle line and retangular line for test
 *
 *************************************************************************/
//gx,gz,gz is the central point
void circle_line(geometry_msgs::Point circle_central, float radius, float velocity);
//gx,gz,gz is the central point
void rectangle_line(geometry_msgs::Point set_point, float length, float width, float velocity);

/**************************************************************************
 *
 *functions for position control
 *
 *************************************************************************/
//dead_zone: unit m          mode: determines if use position or yaw control,1:position control 2:yaw control 3:position and yaw control
void position_pid_control(geometry_msgs::Point current_set_point,geometry_msgs::Point current_local_point,float velocity_limit,float target_yaw, float dead_zone, uint8_t mode);
//in ground frame: vx:E vy:N
geometry_msgs::Point horizon_velocity_pid_control(float vx, float vy);

float calculate_target_yaw(geometry_msgs::Point current_set_point,geometry_msgs::Point set_point);

geometry_msgs::Point limit_velocity(float vx, float vy,float maximum);

void position_hold(geometry_msgs::Point tag_position, uint8_t mode);

void tag_mission();

void tag_RTL();

//yaw alignment for initial
void init_yaw_alignment();

//yaw alignment for general use
void yaw_alignment();

//the angle in rad and the length in meter
void send_cmd_roll_pitch_yawrate_verticalposition(float roll, float pitch, float yawrate, float ver_pos);
void send_cmd_xyz_velocity_yaw_rate(float vx, float vy, float vz, float yaw_rate);
void send_cmd_xyz_position_yaw_angle(float x, float y, float z, float yaw);

void forward_backwack_vel(void);
void forward_backward_position(void);

/**************************************************************************
 *
 *some aided functions
 *
 *************************************************************************/

void M100_obtain_mode();
void A3_N3_obtain_mode();


template <typename T>
T limit(T a, T max)
{
  if(a > max)
    a = max;
  if(a < -max)
    a = -max;
  return a;
}

/**************************************************************************
 *
 *variables
 *
 *************************************************************************/
geometry_msgs::Point current_set_point;
geometry_msgs::Point circle_central;
geometry_msgs::Point start_point;
geometry_msgs::Point earth_point_last;
bool drone_on_circle  = false, inited = false;
uint8_t stage = 0, rcount = 0;
float target_yaw = 0.0;
//yaw_bias: the angle between two-dimensional code and flight control
//the first usage of yaw_bias: target_yaw = yaw_bias, to follow the direction of two-dimensional code
//the second usage of yaw_bias: angle.z = fc_attitude_rpy.z - yaw_bias, for rotation matrix
float yaw_bias= 0.0;

void commandTest();

//vector functions and data structure, to store the tag messages, they will be used in RTL


struct CodeInfo
{
	uint32_t	id;
	float	    angle;

	CodeInfo(const uint32_t& idInit, const float& angleInit)
	{
		id		= idInit;
		angle	= angleInit;
	}

	bool operator == (const CodeInfo& itemToCompare) const
	{
		return (itemToCompare.id == this->id);
	}
};

//display vector messages
template <typename T>
void DisplayVector(const T& codes)
{
	for (size_t index = 0; index < codes.size(); ++index)
	{
    ROS_INFO("Element[%d]:",index);
    ROS_INFO("%d %f",codes[index].id,codes[index].angle);
	}
}

#endif // DEMO_FLIGHT_CONTROL_H
