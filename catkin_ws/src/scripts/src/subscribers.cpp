#include "/home/newHomeDir/Controller_c-_wrapper/catkin_ws/src/scripts/src/talker.h"

#include <math.h>
#include "mavros_msgs/HomePosition.h"
#include "geographic_msgs/GeoPointStamped.h"

// Subscribers
ros::Subscriber homePositionSub;
ros::Subscriber gpOriginSub;

// Callbacks
void homePosition_callback(const mavros_msgs::HomePosition::ConstPtr& msg);
void gpOrigin_callback(const geographic_msgs::GeoPointStamped::ConstPtr &msg);

void init_subscribers(ros::NodeHandle n)
{

  // Initialise subscribers
  homePositionSub = n.subscribe("mavros/home_position/home"       , 1, homePosition_callback);
  gpOriginSub     = n.subscribe("mavros/global_position/gp_origin", 1, gpOrigin_callback);

  return;
}

void homePosition_callback(const mavros_msgs::HomePosition::ConstPtr &msg)
{
  // Subscribes to /mavros/home_position/home
  // Gives the 'home' position of the aircraft (not always at the EKF origin)

  _ekfOrigin.longitude = msg->geo.longitude;
  _ekfOrigin.latitude  = msg->geo.latitude;
  _ekfOrigin.altitude  = msg->geo.altitude;

  return;
}


void gpOrigin_callback(const geographic_msgs::GeoPointStamped::ConstPtr &msg)
{

  // Callback for processing the EKF Origin message

  double rad2deg = (180.0 / M_PI);

  double X = msg->position.latitude;
  double Y = msg->position.longitude;
  double Z = msg->position.altitude;

  // Planet properties
  double a = 6378137.0;
  double b = 6356752.31424518;
  double e = sqrt((a*a-b*b)/(a*a));
  double e_dash = sqrt((a*a-b*b)/(b*b));

  double p = sqrt(X*X + Y*Y);
  double theta = atan2(Z*a,p*b);

  double lon = atan2(Y,X);
  double lat = atan2(Z+e_dash*e_dash*b*pow(sin(theta),3),
                     p-e*e*a*pow(cos(theta),3));

  lat = lat * rad2deg;
  lon = lon * rad2deg;

  // Debugging

  //printf("Calculated:\n");
  //printf("lat: %.7f, lon: %.7f\n",lat*rad2deg,lon*rad2deg);


  // All done
  return;
}