// Code for python controller to interface with ros in c++
// By: Isabel de Luis (ideluis@olin.edu)
// Last edited: June 17, 2022

#include </home/newHomeDir/Controller_c-_wrapper/cpp_getter.h>

PyObject *py_mod; // PyObject for the module/.py file


// PyObject -> Vector
/**
 * @brief A function to transform a PyObject list or tuple to a vector.
 * Gotten from: https://gist.github.com/rjzak/5681680, modified for my needs
 * @param incoming: A PyObject list or tuple of floats
 * @return data: a vector of doubles
 */
vector<double> listTupleToVector_Float(PyObject* incoming) {
	vector<double> data;
	if (PyTuple_Check(incoming)) {
		for(Py_ssize_t i = 0; i < PyTuple_Size(incoming); i++) {
			PyObject *value = PyTuple_GetItem(incoming, i);
			data.push_back(PyFloat_AsDouble(value) );
		}
	} else {
		if (PyList_Check(incoming)) {
			for(Py_ssize_t i = 0; i < PyList_Size(incoming); i++) {
				PyObject *value = PyList_GetItem(incoming, i);
				data.push_back( PyFloat_AsDouble(value) );
			}
		} else {
			throw logic_error("Passed PyObject pointer was not a list or tuple!");
		}
	}
	return data;
}

/**
 * @brief Starter function for class importation that initializes the python interpreter 
 *        and imports the module
 */
void starter()
{
    // printf("Initializing...\n");
    Py_Initialize(); // initialize interpreter

    // imports

    PyRun_SimpleString("import sys");
    PyRun_SimpleString("import os");
    PyRun_SimpleString("sys.path.append(os.getcwd())");


    // get module
    printf("importing module...\n");
    
    py_mod = PyImport_ImportModule("PID_controller");
    
    // if module doesn't exist, print error and exit program
    if(py_mod == NULL)
    {
        PyErr_Print();
        exit(-1);
    }

    // printf("module imported...\n");
}

/**
 * @brief Get a class object from a python module
 * 
 * @param class_name: a const char* representing the name of the class
 * @param args: a PyObject* tuple with the arguments of the class
 * @return PyObject* py_inst: An instantiated object of the class
 */

PyObject* get_class(const char* class_name, PyObject *args)
{
    // returns are instantiated object of the class inputted
    PyObject *py_class, *py_inst;
    
    printf("inside fetcher_base... \n");
   // starter();

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
    py_inst = PyObject_Call(py_class, args, PyDict_New());

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

/**
 * @brief Get a function within a class and call it
 * 
 * @param func_name: char* representing the name of the function 
 * @param py_inst char* the instantiated object of a python class
 * @param args: PyObject* tuple with the arguments within. 
 *              If there are no arguments, the tuple should be empty
 * @param kwargs PyObject* dictionary with the keyword arguments within. 
 *               If there are no arguments the dictionary should be empty.
 * @return PyObject* py_res: The return from the function called
 */
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
        printf("ERROR: kwargs is not a dictionary");
        exit(-1);
    }
    printf("kwargs type checked...\n");

    printf("calling method...\n");
   
    py_res = PyObject_Call(py_meth, args, kwargs); // save the result
    if (py_res == NULL)
    {
        PyErr_Print();
        exit(-1);
    }
    
    printf("method called...\n");

    return py_res;
}

// int main()
// {

//     //measure time takes to get classes
//     time_t begin, end;
//     time(&begin);
//     // declare args
//     starter();

//     // declare the args
//     PyObject* args = PyTuple_Pack(Py_ssize_t(7), PyFloat_FromDouble(.5), PyFloat_FromDouble(1.0), PyFloat_FromDouble(1.5), PyFloat_FromDouble(.01), PyFloat_FromDouble(10.0), PyFloat_FromDouble(1.0), PyFloat_FromDouble(10.0)); 
//     // get the class
//     PyObject* controller = get_class("PID", args);
    
//     // get the returns from the function
//     PyObject* result = get_function((char*)"new_position", controller, PyTuple_New(0), PyDict_New()); // will need to change to c++ object

//     // can change the result so it's a C++ Object not a PyObject if needed
//     // PyObject *position = result[0];    
//     // PyObject *velocity = result[1];
//     // PyObject *acceleration = result[2];      

//     //vector<double> pos = listTupleToVector_Float((PyObject*)result[1]);


//     time(&end);

//     // print elapsed time and print
//     time_t elapsed = end-begin;

//     printf("time elapsed: %ld \n", elapsed);

//     return 1;
// }