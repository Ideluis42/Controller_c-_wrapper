#include "/home/newHomeDir/Controller_c-_wrapper/catkin_ws/src/scripts/src/talker_test.h"
#include "/home/newHomeDir/Controller_c-_wrapper/catkin_ws/src/scripts/src/publishers.cpp"
#include "/home/newHomeDir/Controller_c-_wrapper/catkin_ws/src/scripts/src/subscribers.cpp"
#include "/home/newHomeDir/Controller_c-_wrapper/catkin_ws/src/scripts/src/services.cpp"
#include "/home/newHomeDir/Controller_c-_wrapper/catkin_ws/src/scripts/src/helpers.cpp"

uint _counter = 0;

enum commandModeEnum {local, global, attitude} commandMode;

int main(int argc, char **argv)
{
    // Initialise variables
    commandMode = commandModeEnum::attitude;

    // Initialise ROS
    ros::init(argc, argv, "aircraft_positioning");
    ros::NodeHandle n;

    // Initialise Publishers
    init_publishers(n);
    init_subscribers(n);

    init_services(n);

    // Set up loop rates
    ros::Rate loop_rate(20);

    ROS_INFO("==== Aircraft Positioning Test Node ====");

    setMessageRate(mavlink::common::msg::GPS_GLOBAL_ORIGIN::MSG_ID,   1.0f); // GPS_GLOBAL_ORIGIN (mavros/global_postion/gp_offset)
    setMessageRate(mavlink::common::msg::ATTITUDE::MSG_ID,           50.0f); // ATTITUDE (mavros/imu/data) 
    setMessageRate(mavlink::common::msg::LOCAL_POSITION_NED::MSG_ID, 25.0f); // LOCAL_POSITION_NED (mavros/local_postion/pose)

    // ROS Loop
    while (ros::ok()) {

    _counter++;

    // Position Targets
    double X = 20;
    double Y = 0;
    double Z = 15.0 + 5.0*sin(3*_counter/100.0);

    // Attitude Targets
    double roll = 0, pitch = 0;
    double yaw = 0;
    double thrust = 0.50;

    // Convert to Lat/Lon
    double lat, lon, alt;
    XY_to_latlon(X, Y, &lat, &lon);
    alt = Z;

    // Send the command
    switch (commandMode)
    {
        case (commandModeEnum::attitude) :
            publish_attitude_target(roll,pitch,yaw,thrust);
            break;
        
        case (commandModeEnum::global) :
            publish_global_target(lat, lon, alt);
            break;

        case (commandModeEnum::local) :
            publish_local_target(X,Y,Z);
            break;

        default :
            // do nothing
            break;

    }

    // Switch between each of the control modes
    if ((_counter % 400) == 0) {
        switch (commandMode)
        {

        case (commandModeEnum::attitude) :
            commandMode = local;
            ROS_INFO("Switching to Local  Positioning Mode");
            break;
        
        case (commandModeEnum::local) :
            commandMode = global;
            ROS_INFO("Switching to Global Positioning Mode");
            break;

        case (commandModeEnum::global) :
            commandMode = attitude;
            ROS_INFO("Switching to Attitude           Mode");
            break;
        
        default:
            break;
        }
    }   

    // Debugging
    //printf("Commanded Position: (Lat: %6.2f, Lon: %6.2f, Alt: %6.2f)\n",
    //   lat, lon, alt);

    // Give ROS time to do it's thing
    ros::spinOnce();
    loop_rate.sleep();

    }
    return 0;
}