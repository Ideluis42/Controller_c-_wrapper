
#include "/home/newHomeDir/Controller_c-_wrapper/catkin_ws/src/scripts/src/talker.h"

#include "mavros_msgs/SetMode.h" 
#include "mavros_msgs/CommandLong.h"
#include "mavros_msgs/CommandCode.h"
#include "mavros_msgs/MessageInterval.h"

// We can find services and call them through rqt > Service Caller

// === Globals
// Services
ros::ServiceClient setModeClient;
ros::ServiceClient commandLongClient;
ros::ServiceClient setMessageRateClient;

void init_services(ros::NodeHandle n) {
  setModeClient        = n.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
  commandLongClient    = n.serviceClient<mavros_msgs::CommandLong>("mavros/cmd/command");
  setMessageRateClient = n.serviceClient<mavros_msgs::MessageInterval>("mavros/set_message_interval");
}

bool setMessageRate(uint32_t msg_id, float rate) {
  mavros_msgs::MessageInterval cmd;

  cmd.request.message_id = msg_id;
  cmd.request.message_rate = rate;

  setMessageRateClient.call(cmd);

  if (cmd.response.success) {
    ROS_INFO("Set message rate (#%d) successful",msg_id);
  } else {
    ROS_INFO("!!! Set message rate (#%d) failed !!!",msg_id);
  }

  return (cmd.response.success);

}

bool cmdSetSpeed(float newSpeed) {
  mavros_msgs::CommandLong cmd;
  cmd.request.command = mavros_msgs::CommandCode::SET_MESSAGE_INTERVAL;
  cmd.request.broadcast = 1;
  cmd.request.param2 = newSpeed;
  commandLongClient.call(cmd);

  if (cmd.response.success) {
    printf("Speed set to %f\n",cmd.request.param2);
  } else {
    printf("!!! Speed Set Failed :( !!!\n");
  }

  return (cmd.response.success);

}

bool setMode_AUTO() {
  // Sets the current mode to AUTO
  mavros_msgs::SetMode new_mode;
  new_mode.request.custom_mode = "AUTO";

  // Send and check that everythign went correctly
  if (setModeClient.call(new_mode) && new_mode.response.mode_sent) {
    ROS_INFO("AUTO Enabled");
    return (1);
  } else {
    ROS_INFO("!! SET MODE FAILED !!");
    return (0);
  }
  
  // We should never get here, assume our request failed
  return (0);

}

bool setMode_GUIDED() {
  // Sets the current mode to GUIDED
  mavros_msgs::SetMode new_mode;
  new_mode.request.custom_mode = "GUIDED";

  // Send and check that everythign went correctly
  if (setModeClient.call(new_mode) && new_mode.response.mode_sent) {
    ROS_INFO("GUIDED Enabled");
    return (1);
  } else {
    ROS_INFO("!! SET MODE FAILED !!");
    return (0);
  }
  
  // We should never get here, assume our request failed
  return (0);

}

bool setMode_RTL() {
  // Sets the current mode to GUIDED
  mavros_msgs::SetMode new_mode;
  new_mode.request.custom_mode = "RTL";

  // Send and check that everythign went correctly
  if (setModeClient.call(new_mode) && new_mode.response.mode_sent) {
    ROS_INFO("RTL Enabled");
    return (1);
  } else {
    ROS_INFO("!! SET MODE FAILED !!");
    return (0);
  }
  
  // We should never get here, assume our request failed
  return (0);

}
