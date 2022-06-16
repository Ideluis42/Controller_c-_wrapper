// Code for python controller to interface with ros in c++
// By: Isabel de Luis (ideluis@olin.edu)
// Last edited: June 16, 2022

#include </home/newHomeDir/Controller_c-_wrapper/cpp_getter.h>


/** 
 * Code for getting method within class:

    printf("getting method...\n");
    py_meth = PyObject_GetAttrString(py_inst, "method");
    
    if(py_meth == NULL)
    {
        PyErr_Print();
        exit(-1);
    }

    printf("method retrieved...\n");
    Py_DECREF(py_inst);

    printf("calling method...\n");
    py_res = PyObject_CallObject(py_meth, NULL);
    printf("Method called...\n");
    
    // Convert result to C--no code implemented because returns can vary
 */

PyObject *py_mod; // PyObject for the module/.py file

void starter()
{
    Py_Initialize(); // initialize interpreter

    // imports

    PyRun_SimpleString("import sys");
    PyRun_SimpleString("import os");
    PyRun_SimpleString("sys.path.append(os.getcwd())");


    // get module
    printf("importing module...\n");
    
    py_mod = PyImport_ImportModule("py_controller");
    
    // if module doesn't exist, print error and exit program
    if(py_mod == NULL)
    {
        PyErr_Print();
        exit(-1);
    }

    printf("module imported...\n");
}

void controller()
{
    // Initializing 
    PyObject *py_class, *py_inst;
    starter();

    // get class from python module
    // printf("getting class...\n");
    py_class = PyObject_GetAttrString(py_mod, "Controller");

    // check that the class exists--if it doesn't, print error and exit program
    if(py_class == NULL)
    {
        PyErr_Print();
        exit(-1);
    }

    // printf("class retrieved...\n");
    Py_DECREF(py_mod); // decrement the reference count because no longer needed

    // instantiate an obj of the class so we can use it
    // printf("instantiating class... \n");

    py_inst = PyEval_CallObject(py_class, NULL);
    
    // if the instantiation doesn't work, print error and exit program
    if(py_inst == NULL)
    {
        PyErr_Print();
        exit(-1);
    }

    printf("class instantiated...\n");
     
    Py_DECREF(py_class); // decrement the reference count because no longer needed

    printf("controller function called and worked \n"); //print statement for assurance the function ran
}

void logger()
{
    // Initializing 
    PyObject *py_class, *py_inst;
    starter();

    // get class from python module
    // printf("getting class...\n");
    py_class = PyObject_GetAttrString(py_mod, "Logger");

    // check that the class exists--if it doesn't, print error and exit program
    if(py_class == NULL)
    {
        PyErr_Print();
        exit(-1);
    }

    // printf("class retrieved...\n");
    Py_DECREF(py_mod); // decrement the reference count because no longer needed

    // instantiate an obj of the class so we can use it
    // printf("instantiating class... \n");

    py_inst = PyEval_CallObject(py_class, NULL);
    
    // if the instantiation doesn't work, print error and exit program
    if(py_inst == NULL)
    {
        PyErr_Print();
        exit(-1);
    }

    printf("class instantiated...\n");
     
    Py_DECREF(py_class); // decrement the reference count because no longer needed

    printf("logger function called and worked \n"); //print statement for assurance the function ran

}
int main()
{
    controller();
    logger();
    return 1;
}