#include "Python.h"
#include "c_gpio.h"

static PyObject *WrongDirectionException;
static PyObject *InvalidModeException;
static PyObject *InvalidDirectionException;
static PyObject *InvalidChannelException;
static PyObject *InvalidPullException;
static PyObject *ModeNotSetException;
static PyObject *SetupException;
static PyObject *high;
static PyObject *low;
static PyObject *input;
static PyObject *output;
static PyObject *version;


// setup function run on import of the RPi.GPIO module
static int module_setup(void)
{
   gpio_init();

   return SETUP_OK;
}

// python function cleanup()
static PyObject *py_cleanup(PyObject *self, PyObject *args)
{
    gpio_close();
    
   Py_INCREF(Py_None);
   return Py_None;
}

// python function setup(channel, direction, pull_up_down=PUD_OFF)
static PyObject *py_setup_channel(PyObject *self, PyObject *args, PyObject *kwargs)
{
   int channel, direction;
   static char *kwlist[] = {"channel", "direction", NULL};
   int func;
   
   if (!PyArg_ParseTupleAndKeywords(args, kwargs, "ii|i", kwlist, &channel, &direction))
      return NULL;

   if (direction != INPUT && direction != OUTPUT)
   {
      PyErr_SetString(InvalidDirectionException, "An invalid direction was passed to setup()");
      return NULL;
   }

   func = gpio_open(channel, direction);
   if (func != 0)  // already an output not set from this program)
   {
      PyErr_WarnEx(NULL, "This channel is already in use, continuing anyway.  Use GPIO.setwarnings(False) to disable warnings.", 1);
   }

   Py_INCREF(Py_None);
   return Py_None;
}

// python function output(channel, value)
static PyObject *py_output_gpio(PyObject *self, PyObject *args)
{
   int channel, value;

   if (!PyArg_ParseTuple(args, "ii", &channel, &value)) 
      return NULL;

//   printf("Output GPIO %d value %d\n", gpio, value);
   if(gpio_setpin(channel, value)!=0)
   {
      PyErr_WarnEx(NULL, "Error on setting pin.", 1);
   }

   Py_INCREF(Py_None);
   return Py_None;
}

// python function value = input(channel)
static PyObject *py_input_gpio(PyObject *self, PyObject *args)
{
   int channel;

   if (!PyArg_ParseTuple(args, "i", &channel))
      return NULL;
   
   //   printf("Input GPIO %d\n", gpio);
   if (gpio_getpin(channel))
      Py_RETURN_TRUE;
   else
      Py_RETURN_FALSE;
}

PyMethodDef rpi_gpio_methods[] = {
   {"setup", (PyCFunction)py_setup_channel, METH_VARARGS | METH_KEYWORDS, "Set up the GPIO channel,direction and (optional) pull/up down control\nchannel   - Either: RPi board pin number (not BCM GPIO 00..nn number).  Pins start from 1\n            or    : BCM GPIO number\ndirection - INPUT or OUTPUT\n[pull_up_down] - PUD_OFF (default), PUD_UP or PUD_DOWN"},
   {"cleanup", py_cleanup, METH_VARARGS, "Clean up by resetting all GPIO channels that have been used by this program to INPUT with no pullup/pulldown and no event detection"},
   {"output", py_output_gpio, METH_VARARGS, "Output to a GPIO channel"},
   {"input", py_input_gpio, METH_VARARGS, "Input from a GPIO channel"},
   {NULL, NULL, 0, NULL}
};

#if PY_MAJOR_VERSION > 2
static struct PyModuleDef rpigpiomodule = {
   PyModuleDef_HEAD_INIT,
   "RPi.GPIO", /* name of module */
   NULL,       /* module documentation, may be NULL */
   -1,         /* size of per-interpreter state of the module,
                  or -1 if the module keeps state in global variables. */
   rpi_gpio_methods
};
#endif

#if PY_MAJOR_VERSION > 2
PyMODINIT_FUNC PyInit_SPI(void)
#else
PyMODINIT_FUNC initSPI(void)
#endif
{
   PyObject *module = NULL;

#if PY_MAJOR_VERSION > 2
   if ((module = PyModule_Create(&rpigpiomodule)) == NULL)
      goto exit;
#else
   if ((module = Py_InitModule("RPi.GPIO", rpi_gpio_methods)) == NULL)
      goto exit;
#endif

   WrongDirectionException = PyErr_NewException("RPi.GPIO.WrongDirectionException", NULL, NULL);
   PyModule_AddObject(module, "WrongDirectionException", WrongDirectionException);

   InvalidModeException = PyErr_NewException("RPi.GPIO.InvalidModeException", NULL, NULL);
   PyModule_AddObject(module, "InvalidModeException", InvalidModeException);

   InvalidDirectionException = PyErr_NewException("RPi.GPIO.InvalidDirectionException", NULL, NULL);
   PyModule_AddObject(module, "InvalidDirectionException", InvalidDirectionException);

   InvalidChannelException = PyErr_NewException("RPi.GPIO.InvalidChannelException", NULL, NULL);
   PyModule_AddObject(module, "InvalidChannelException", InvalidChannelException);

   InvalidPullException = PyErr_NewException("RPi.GPIO.InvalidPullException", NULL, NULL);
   PyModule_AddObject(module, "InvalidPullException", InvalidPullException);

   ModeNotSetException = PyErr_NewException("RPi.GPIO.ModeNotSetException", NULL, NULL);
   PyModule_AddObject(module, "ModeNotSetException", ModeNotSetException);

   SetupException = PyErr_NewException("RPi.GPIO.SetupException", NULL, NULL);
   PyModule_AddObject(module, "SetupException", SetupException);

   high = Py_BuildValue("i", HIGH);
   PyModule_AddObject(module, "HIGH", high);

   low = Py_BuildValue("i", LOW);
   PyModule_AddObject(module, "LOW", low);

   output = Py_BuildValue("i", OUTPUT);
   PyModule_AddObject(module, "OUT", output);

   input = Py_BuildValue("i", INPUT);
   PyModule_AddObject(module, "IN", input);
      
   version = Py_BuildValue("s", "0.4.1a");
   PyModule_AddObject(module, "VERSION", version);
   
   // set up mmaped areas
   if (module_setup() != SETUP_OK )
   {
#if PY_MAJOR_VERSION > 2
      return NULL;
#else
      return;
#endif
   }
      
   if (Py_AtExit(gpio_close) != 0)
   {
     gpio_close();
#if PY_MAJOR_VERSION > 2
      return NULL;
#else
      return;
#endif
   }

exit:
#if PY_MAJOR_VERSION > 2
   return module;
#else
   return;
#endif
}
