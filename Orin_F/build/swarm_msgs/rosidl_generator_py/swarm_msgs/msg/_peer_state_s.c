// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from swarm_msgs:msg/PeerState.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "swarm_msgs/msg/detail/peer_state__struct.h"
#include "swarm_msgs/msg/detail/peer_state__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool swarm_msgs__msg__peer_state__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[37];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("swarm_msgs.msg._peer_state.PeerState", full_classname_dest, 36) == 0);
  }
  swarm_msgs__msg__PeerState * ros_message = _ros_message;
  {  // drone_id
    PyObject * field = PyObject_GetAttrString(_pymsg, "drone_id");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->drone_id = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // latitude
    PyObject * field = PyObject_GetAttrString(_pymsg, "latitude");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->latitude = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // longitude
    PyObject * field = PyObject_GetAttrString(_pymsg, "longitude");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->longitude = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // altitude
    PyObject * field = PyObject_GetAttrString(_pymsg, "altitude");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->altitude = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // vn
    PyObject * field = PyObject_GetAttrString(_pymsg, "vn");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->vn = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // ve
    PyObject * field = PyObject_GetAttrString(_pymsg, "ve");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->ve = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // vd
    PyObject * field = PyObject_GetAttrString(_pymsg, "vd");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->vd = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // heading
    PyObject * field = PyObject_GetAttrString(_pymsg, "heading");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->heading = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // stamp
    PyObject * field = PyObject_GetAttrString(_pymsg, "stamp");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->stamp = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // guidance_mode
    PyObject * field = PyObject_GetAttrString(_pymsg, "guidance_mode");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->guidance_mode = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // fc_mode_code
    PyObject * field = PyObject_GetAttrString(_pymsg, "fc_mode_code");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->fc_mode_code = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // fc_armed
    PyObject * field = PyObject_GetAttrString(_pymsg, "fc_armed");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->fc_armed = (Py_True == field);
    Py_DECREF(field);
  }
  {  // fc_connected
    PyObject * field = PyObject_GetAttrString(_pymsg, "fc_connected");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->fc_connected = (Py_True == field);
    Py_DECREF(field);
  }
  {  // battery_voltage
    PyObject * field = PyObject_GetAttrString(_pymsg, "battery_voltage");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->battery_voltage = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // battery_current
    PyObject * field = PyObject_GetAttrString(_pymsg, "battery_current");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->battery_current = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // battery_remaining
    PyObject * field = PyObject_GetAttrString(_pymsg, "battery_remaining");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->battery_remaining = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * swarm_msgs__msg__peer_state__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of PeerState */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("swarm_msgs.msg._peer_state");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "PeerState");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  swarm_msgs__msg__PeerState * ros_message = (swarm_msgs__msg__PeerState *)raw_ros_message;
  {  // drone_id
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->drone_id);
    {
      int rc = PyObject_SetAttrString(_pymessage, "drone_id", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // latitude
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->latitude);
    {
      int rc = PyObject_SetAttrString(_pymessage, "latitude", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // longitude
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->longitude);
    {
      int rc = PyObject_SetAttrString(_pymessage, "longitude", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // altitude
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->altitude);
    {
      int rc = PyObject_SetAttrString(_pymessage, "altitude", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // vn
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->vn);
    {
      int rc = PyObject_SetAttrString(_pymessage, "vn", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // ve
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->ve);
    {
      int rc = PyObject_SetAttrString(_pymessage, "ve", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // vd
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->vd);
    {
      int rc = PyObject_SetAttrString(_pymessage, "vd", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // heading
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->heading);
    {
      int rc = PyObject_SetAttrString(_pymessage, "heading", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // stamp
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->stamp);
    {
      int rc = PyObject_SetAttrString(_pymessage, "stamp", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // guidance_mode
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->guidance_mode);
    {
      int rc = PyObject_SetAttrString(_pymessage, "guidance_mode", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // fc_mode_code
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->fc_mode_code);
    {
      int rc = PyObject_SetAttrString(_pymessage, "fc_mode_code", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // fc_armed
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->fc_armed ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "fc_armed", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // fc_connected
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->fc_connected ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "fc_connected", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // battery_voltage
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->battery_voltage);
    {
      int rc = PyObject_SetAttrString(_pymessage, "battery_voltage", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // battery_current
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->battery_current);
    {
      int rc = PyObject_SetAttrString(_pymessage, "battery_current", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // battery_remaining
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->battery_remaining);
    {
      int rc = PyObject_SetAttrString(_pymessage, "battery_remaining", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
