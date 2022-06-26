
#pragma once

// ROS Headers
#include "ros/ros.h"

#include "std_msgs/String.h"

#include "geometry_msgs/PoseStamped.h"  // Setpoint publish, current position
#include "geometry_msgs/Quaternion.h"

#include "geographic_msgs/GeoPoint.h"

#include </opt/ros/noetic/include/mavlink/v2.0/common/common.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>

// Standard C++ headers
#include <string>
#include <sstream>
#include <iostream>
#include <vector>
#include <array>
#include <algorithm>
#include <stdlib.h>
#include <time.h>

//======= Functions
void publish_global_target  (double lat, double lon, double alt);
void publish_local_target   (double x, double y, double alt);
void publish_attitude_target(double roll, double pitch, double yaw, double thrust);

// Initialisation
void init_publishers (ros::NodeHandle n);
void init_subscribers(ros::NodeHandle n);
void init_services   (ros::NodeHandle n);

bool setMessageRate(uint32_t msg_id, float rate);
bool cmdSetSpeed(float newSpeed);
bool setMode_AUTO();
bool setMode_GUIDED();
bool setMode_RTL();

// Helpers
void XY_to_latlon(double   X, double   Y, double *lat, double *lon);
void latlon_to_XY(double lat, double lon, double   *X, double   *Y);

//======= Variables
extern geographic_msgs::GeoPoint _ekfOrigin;

// Debugging (main)
extern uint _counter;
