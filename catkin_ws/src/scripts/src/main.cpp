#include </home/newHomeDir/Controller_c-_wrapper/catkin_ws/src/scripts/src/cpp_getter.cpp>
#include </home/newHomeDir/Controller_c-_wrapper/catkin_ws/src/scripts/src/talker.cpp>

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
    double desiredPos = 10.0;
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
        double new_position = pos + new_vel*dt;

        // turn position into a float
        pos = (float)new_position;

        // send arguments to hardware w/ ros
        printf("pos: %f \n", pos);
        talker(pos, argc, argv);
        printf("sent to ros \n");

        orig_vel = new_vel;
    }
        

    return 0;
}