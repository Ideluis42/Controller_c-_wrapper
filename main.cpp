#include </home/newHomeDir/Controller_c-_wrapper/cpp_getter.cpp>

int main()
{
    starter();
    // define the parameters for the class
    // They are as follows:
    // Proportional gain, integral gain, derivative gain, dt, final time, mass, and desired final position
    PyObject* class_params = PyTuple_Pack(Py_ssize_t(7), PyFloat_FromDouble(.5), PyFloat_FromDouble(1.0), PyFloat_FromDouble(1.5), PyFloat_FromDouble(.01), PyFloat_FromDouble(10.0), PyFloat_FromDouble(1.0), PyFloat_FromDouble(10.0)); 

    // get the class
    PyObject* controller = get_class("PID", class_params);

    PyObject* starting_args = PyTuple_Pack(Py_ssize_t(3), PyLong_FromLong(0), PyLong_FromLong(0), PyLong_FromLong(0));
    // get position
    PyObject* position = get_function((char*)"get_new_position", controller, starting_args, PyDict_New());

    // send arguments to hardware w/ ros
}