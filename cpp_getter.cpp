// Code for python controller to interface with ros in c++
// By: Isabel de Luis (ideluis@olin.edu)
// Last edited: June 17, 2022

#include </home/newHomeDir/Controller_c-_wrapper/cpp_getter.h>

PyObject *py_mod; // PyObject for the module/.py file

void starter()
{
    printf("Initializing...\n");
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

PyObject* get_class(const char* class_name)
{
    // returns are instantiated object of the class inputted
    PyObject *py_class, *py_inst;
    
    printf("inside fetcher_base... \n");
    starter();

    printf("getting class...\n");
    // get class
    py_class = PyObject_GetAttrString(py_mod, class_name);

    // check that the class exists--if it doesn't, print error and exit program
    if(py_class == NULL)
    {
        PyErr_Print();
        exit(-1);
    }

    Py_DECREF(py_mod); // decrement the reference count because no longer needed

    printf("class retrieved...\n");

    printf("instantiating class...\n");
    py_inst = PyEval_CallObject(py_class, NULL);

    // if the instantiation doesn't work, print error and exit program
    if(py_inst == NULL)
    {
        PyErr_Print();
        exit(-1);
    }

    Py_DECREF(py_class); // decrement the reference count because no longer needed
    printf("class instantiated...\n");
    printf("about to return...\n");
    return py_inst;
}

// TO-DO: check args
// if function has no args, just pass an empty tuple
PyObject* get_function(char* func_name, PyObject *py_inst, PyObject *args, PyObject *kwargs)
{

    // To note: this func returns the PyObject* version of the results
    // NOT a C++ object--will need to be converted afterwards

    PyObject *py_meth, *py_res;
    
    printf("getting method...\n");
    // retrieve the method
    py_meth = PyObject_GetAttrString(py_inst, func_name);
    
    // check to make sure the method actually exists--if it doesn't, print the error and exit the program
    if(py_meth == NULL)
    {
        PyErr_Print();
        exit(-1);
    }

    Py_DECREF(py_inst); // decrement the reference count b/c no longer needed 
    printf("method retrieved...\n");

    printf("checking args type...\n");
    // check if type is tuple, if not print error and exit program
    
    if(!PyTuple_Check(args))
    {
        //Call the method. Save the returns in a variable (if they exist)
        printf("ERROR: args is not a tuple or null");
        exit(-1);
    }
    printf("args type checked...\n");

    printf("checking kwargs type...\n");

    if(!PyDict_Check(kwargs))
    {
        PyErr_Print();
        printf("ERROR: kwargs is not a dictionary");
        exit(-1);
    }
    printf("kwargs type checked...\n");

    printf("calling method...\n");
    // error here!
    py_res = PyObject_Call(py_meth, args, kwargs); // save the result
    if (py_res == NULL)
    {
        PyErr_Print();
        exit(-1);
    }
    
    printf("method called...\n");

    return py_res;
}

int main()
{
    //measure time takes to get classes
    time_t begin, end;
    time(&begin);

    PyObject* controller = get_class("Controller");
    PyObject* args = PyTuple_Pack(Py_ssize_t(3), PyLong_FromLong(1), PyLong_FromLong(10), PyLong_FromLong(15)); 
    
    PyObject* kwargs = PyDict_New();
    PyObject* key = PyUnicode_FromString("key");

    int check = PyDict_SetItem(kwargs, key, PyLong_FromLong(1));
    
    PyObject* result = get_function((char*)"test", controller, args, kwargs); // will need to change to c++ object

    time(&end);

    // print elapsed time and print
    time_t elapsed = end-begin;

    printf("time elapsed: %ld \n", elapsed);

    return 1;
}