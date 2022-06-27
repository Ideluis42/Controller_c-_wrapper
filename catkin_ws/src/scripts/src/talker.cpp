#include </home/newHomeDir/Controller_c-_wrapper/catkin_ws/src/scripts/src/talker.h>
#include "/home/newHomeDir/Controller_c-_wrapper/catkin_ws/src/scripts/src/publishers.cpp"
#include "/home/newHomeDir/Controller_c-_wrapper/catkin_ws/src/scripts/src/subscribers.cpp"
#include "/home/newHomeDir/Controller_c-_wrapper/catkin_ws/src/scripts/src/services.cpp"
#include "/home/newHomeDir/Controller_c-_wrapper/catkin_ws/src/scripts/src/helpers.cpp"
#include </home/newHomeDir/Controller_c-_wrapper/catkin_ws/src/scripts/src/cpp_getter.cpp>


int counter = 0;
enum commandModeEnum {local, global, attitude} commandMode;

void talker(double pos, int argc, char **argv)
{
  // Initialise variables
  commandMode = commandModeEnum::attitude;

  ros::init(argc, argv, "talker");
  ros::NodeHandle n;

  // Initialise Publishers
  init_publishers(n);
  init_subscribers(n);

  init_services(n);
  ros::Rate loop_rate(20);
  setMessageRate(mavlink::common::msg::GPS_GLOBAL_ORIGIN::MSG_ID,   1.0f); // GPS_GLOBAL_ORIGIN (mavros/global_postion/gp_offset)
  setMessageRate(mavlink::common::msg::ATTITUDE::MSG_ID,           50.0f); // ATTITUDE (mavros/imu/data) 
  setMessageRate(mavlink::common::msg::LOCAL_POSITION_NED::MSG_ID, 25.0f); // LOCAL_POSITION_NED (mavros/local_postion/pose)
  
  std_msgs::Float32 msg;

  double X = pos;
  double Y = 0;
  double Z = 0;

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
  if ((counter % 400) == 0) {
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
          ROS_INFO("Switching to Attitude Mode");
          break;
      
      default:
          break;
      }
  }   

  
  ros::spinOnce();

  loop_rate.sleep();
  ++counter;


}

int main(int argc, char **argv)
{
    starter();
    // define the parameters for the class
    // They are as follows:
    // Proportional gain, integral gain, derivative gain, dt, final time, mass, and desired final position
    double Kp = .5;
    double Ki = 1;
    double Kd = 1.5;
    double dt = 0.01;
    double ft = 10; // seconds
    double mass = 1; //kg
    double desiredPos = 20.0;
    PyObject* class_params = PyTuple_Pack(Py_ssize_t(7), PyFloat_FromDouble(Kp), PyFloat_FromDouble(Ki), PyFloat_FromDouble(Kd), PyFloat_FromDouble(dt), PyFloat_FromDouble(ft), PyFloat_FromDouble(mass), PyFloat_FromDouble(desiredPos)); 

    // get the class
    PyObject* controller = get_class("PID", class_params);    
    float pos = 0;
    double orig_vel = 1; //m/s^2

    while(pos < desiredPos)
    {
        // Arguments: Current position, current velocity, current acceleration
        PyObject* starting_args = PyTuple_Pack(Py_ssize_t(1), PyFloat_FromDouble((double)pos));

        // call function
        PyObject* py_force = get_function((char*)"get_force", controller, starting_args, PyDict_New());
        double force = PyFloat_AsDouble(py_force);

        double new_accel = force/mass;
        double new_vel = orig_vel + new_accel*dt;
        double pos = pos + new_vel*dt;

        // turn position into a float
       
        // send arguments to hardware w/ ros
        printf("pos: %f \n", pos);
        talker(pos, argc, argv);
        printf("sent to ros \n");

        orig_vel = new_vel;
    }
        

    return 0;
}