/*
 * @author 	Alexander Rüedlinger <a.rueedlinger@gmail.com>
 * @date 	26.02.2015
 *
 * Python bindings for the BMP180 driver written in C.
 *
 */

#include <Python.h>
#include <structmember.h>
#include "bmp180.h"

typedef struct {
	PyObject_HEAD
	void *bmp180;
} BMP180_Object;



static void BMP180_dealloc(BMP180_Object *self) {
	bmp180_close(self->bmp180);
	self->ob_type->tp_free((PyObject*)self);
}


static PyObject *BMP180_new(PyTypeObject *type, PyObject *args, PyObject *kwds) {
	BMP180_Object *self;
	self = (BMP180_Object *) type->tp_alloc(type, 0);
	return (PyObject *) self;
}


static int BMP180_init(BMP180_Object *self, PyObject *args, PyObject *kwds) {
	int address;
	const char *i2c_device;
	static char *kwlist[] = {"address", "i2c_devcie", NULL};

	if(!PyArg_ParseTupleAndKeywords(args, kwds, "is", kwlist, &address, &i2c_device))
		return -1;

	if(i2c_device) {
		self->bmp180 = bmp180_init(address, i2c_device);
		if(self->bmp180 == NULL) {
			PyErr_SetString(PyExc_RuntimeError, "Cannot initialize sensor. Run program as root and check i2c device / address.");
			return -1;
		}
	}
	return 0;
}


static PyObject *BMP180_pressure(BMP180_Object *self) {
	PyObject *result;
	long pressure = bmp180_pressure(self->bmp180);
	result = PyInt_FromLong(pressure);
	return result;
}


static PyObject *BMP180_temperature(BMP180_Object *self) {
	PyObject *result;
	double temperature = bmp180_temperature(self->bmp180);
	result = PyFloat_FromDouble(temperature);
	return result;
}


static PyObject *BMP180_altitude(BMP180_Object *self) {
	PyObject *result;
	double altitude = bmp180_altitude(self->bmp180);
	result = PyFloat_FromDouble(altitude);
	return result;
}


static PyObject *BMP180_set_oss(BMP180_Object *self, PyObject *args) {
	int oss;
	if(!PyArg_ParseTuple(args, "i", &oss))
		return NULL;

	bmp180_set_oss(self->bmp180, oss);
	return Py_None;
}


static PyMethodDef BMP180_methods[] = {
	{"temperature", (PyCFunction) BMP180_temperature, METH_NOARGS, "Returns a temperature value"},
	{"altitude", (PyCFunction) BMP180_altitude, METH_NOARGS, "Returns a altitude value"},
	{"pressure", (PyCFunction) BMP180_pressure, METH_NOARGS, "Returns a pressure value"},
	{"set_oss", (PyCFunction) BMP180_set_oss, METH_VARARGS, "Set oss"},
	{NULL}  /* Sentinel */
};


static PyMemberDef BMP180_members[] = {
    {NULL}  /* Sentinel */
};


static PyTypeObject BMP180_Type = {
	PyObject_HEAD_INIT(NULL)
	0,                         /*ob_size*/
	"tentacle_pi.BMP180",             /*tp_name*/
	sizeof(BMP180_Object),             /*tp_basicsize*/
	0,                         /*tp_itemsize*/
	(destructor)BMP180_dealloc, /*tp_dealloc*/
	0,                         /*tp_print*/
	0,                         /*tp_getattr*/
	0,                         /*tp_setattr*/
	0,                         /*tp_compare*/
	0,                         /*tp_repr*/
	0,                         /*tp_as_number*/
	0,                         /*tp_as_sequence*/
	0,                         /*tp_as_mapping*/
	0,                         /*tp_hash */
	0,                         /*tp_call*/
	0,                         /*tp_str*/
	0,                         /*tp_getattro*/
	0,                         /*tp_setattro*/
	0,                         /*tp_as_buffer*/
	Py_TPFLAGS_DEFAULT | Py_TPFLAGS_BASETYPE, /*tp_flags*/
	"BMP180 objects",           /* tp_doc */
	0,		               /* tp_traverse */
	0,		               /* tp_clear */
	0,		               /* tp_richcompare */
	0,		               /* tp_weaklistoffset */
	0,		               /* tp_iter */
	0,		               /* tp_iternext */
	BMP180_methods,             /* tp_methods */
	BMP180_members,             /* tp_members */
	0,                         /* tp_getset */
	0,                         /* tp_base */
	0,                         /* tp_dict */
	0,                         /* tp_descr_get */
	0,                         /* tp_descr_set */
	0,                         /* tp_dictoffset */
	(initproc)BMP180_init,      /* tp_init */
	0,                         /* tp_alloc */
	BMP180_new,                 /* tp_new */
};

static PyMethodDef module_methods[] = {
	{NULL}  /* Sentinel */
};



PyMODINIT_FUNC initBMP180(void) {
	PyObject *m;

	if(PyType_Ready(&BMP180_Type) < 0)
		return;

	m = Py_InitModule3("BMP180", module_methods, "BMP180 extension module");

	if(m == NULL)
		return;

	Py_INCREF(&BMP180_Type);
	PyModule_AddObject(m, "BMP180", (PyObject *)&BMP180_Type);
}
