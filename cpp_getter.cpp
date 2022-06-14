// Code for python controller to interface with ros in c++
// By: Isabel de Luis (ideluis@olin.edu)
// Last edited: June 14, 2022

#include </home/newHomeDir/Controller_c-_wrapper/cpp_getter.h>


void controller_cpp_interface()
{
    Py_Initialize(); // Initialize the python interpreter

    // Import the system path
    PyRun_SimpleString("import sys");
    PyRun_SimpleString("sys.path.append(\"~/Controller_c-_wrapper\")");

    // Import module
    CPyObject python_module = PyImport_ImportModule("controller");
    
    // Get the controller class
    CPyObject controller = PyObject_GetAttrString(python_module, (char*)"Controller");
    
    // Create an instance of the class
    CPyObject controller_instance = PyInstanceMethod_New(controller);     
}

void logger_cpp_interface()
{
    Py_Initialize(); // Initialize the python interpreter

    // Import the system path
    PyRun_SimpleString("import sys");
    PyRun_SimpleString("sys.path.append(\"~/Controller_c-_wrapper\")");
     
    CPyObject python_module = PyImport_ImportModule("controller");

    CPyObject logger = PyObject_GetAttrString(python_module, (char*)"Logger");
    
    CPyObject logger_instance = PyInstanceMethod_New(logger);


}
int main()
{
    return 0;
}