// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from swarm_msgs:msg/PeerStates.idl
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
#include "swarm_msgs/msg/detail/peer_states__struct.h"
#include "swarm_msgs/msg/detail/peer_states__functions.h"

#include "rosidl_runtime_c/primitives_sequence.h"
#include "rosidl_runtime_c/primitives_sequence_functions.h"

// Nested array functions includes
#include "swarm_msgs/msg/detail/peer_state__functions.h"
// end nested array functions include
ROSIDL_GENERATOR_C_IMPORT
bool builtin_interfaces__msg__time__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * builtin_interfaces__msg__time__convert_to_py(void * raw_ros_message);
bool swarm_msgs__msg__peer_state__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * swarm_msgs__msg__peer_state__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool swarm_msgs__msg__peer_states__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[39];
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
    assert(strncmp("swarm_msgs.msg._peer_states.PeerStates", full_classname_dest, 38) == 0);
  }
  swarm_msgs__msg__PeerStates * ros_message = _ros_message;
  {  // header_stamp
    PyObject * field = PyObject_GetAttrString(_pymsg, "header_stamp");
    if (!field) {
      return false;
    }
    if (!builtin_interfaces__msg__time__convert_from_py(field, &ros_message->header_stamp)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // peers
    PyObject * field = PyObject_GetAttrString(_pymsg, "peers");
    if (!field) {
      return false;
    }
    PyObject * seq_field = PySequence_Fast(field, "expected a sequence in 'peers'");
    if (!seq_field) {
      Py_DECREF(field);
      return false;
    }
    Py_ssize_t size = PySequence_Size(field);
    if (-1 == size) {
      Py_DECREF(seq_field);
      Py_DECREF(field);
      return false;
    }
    if (!swarm_msgs__msg__PeerState__Sequence__init(&(ros_message->peers), size)) {
      PyErr_SetString(PyExc_RuntimeError, "unable to create swarm_msgs__msg__PeerState__Sequence ros_message");
      Py_DECREF(seq_field);
      Py_DECREF(field);
      return false;
    }
    swarm_msgs__msg__PeerState * dest = ros_message->peers.data;
    for (Py_ssize_t i = 0; i < size; ++i) {
      if (!swarm_msgs__msg__peer_state__convert_from_py(PySequence_Fast_GET_ITEM(seq_field, i), &dest[i])) {
        Py_DECREF(seq_field);
        Py_DECREF(field);
        return false;
      }
    }
    Py_DECREF(seq_field);
    Py_DECREF(field);
  }
  {  // peer_jetson_alive
    PyObject * field = PyObject_GetAttrString(_pymsg, "peer_jetson_alive");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->peer_jetson_alive = (Py_True == field);
    Py_DECREF(field);
  }
  {  // peer_last_heartbeat
    PyObject * field = PyObject_GetAttrString(_pymsg, "peer_last_heartbeat");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->peer_last_heartbeat = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // own_battery_voltage
    PyObject * field = PyObject_GetAttrString(_pymsg, "own_battery_voltage");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->own_battery_voltage = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // own_battery_current
    PyObject * field = PyObject_GetAttrString(_pymsg, "own_battery_current");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->own_battery_current = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // own_battery_remaining
    PyObject * field = PyObject_GetAttrString(_pymsg, "own_battery_remaining");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->own_battery_remaining = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // radio_rssi
    PyObject * field = PyObject_GetAttrString(_pymsg, "radio_rssi");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->radio_rssi = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // radio_remrssi
    PyObject * field = PyObject_GetAttrString(_pymsg, "radio_remrssi");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->radio_remrssi = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // radio_txbuf
    PyObject * field = PyObject_GetAttrString(_pymsg, "radio_txbuf");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->radio_txbuf = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // radio_noise
    PyObject * field = PyObject_GetAttrString(_pymsg, "radio_noise");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->radio_noise = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // radio_remnoise
    PyObject * field = PyObject_GetAttrString(_pymsg, "radio_remnoise");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->radio_remnoise = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // radio_rxerrors
    PyObject * field = PyObject_GetAttrString(_pymsg, "radio_rxerrors");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->radio_rxerrors = (uint16_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // radio_link_alive
    PyObject * field = PyObject_GetAttrString(_pymsg, "radio_link_alive");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->radio_link_alive = (Py_True == field);
    Py_DECREF(field);
  }
  {  // radio_last_status
    PyObject * field = PyObject_GetAttrString(_pymsg, "radio_last_status");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->radio_last_status = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * swarm_msgs__msg__peer_states__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of PeerStates */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("swarm_msgs.msg._peer_states");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "PeerStates");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  swarm_msgs__msg__PeerStates * ros_message = (swarm_msgs__msg__PeerStates *)raw_ros_message;
  {  // header_stamp
    PyObject * field = NULL;
    field = builtin_interfaces__msg__time__convert_to_py(&ros_message->header_stamp);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "header_stamp", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // peers
    PyObject * field = NULL;
    size_t size = ros_message->peers.size;
    field = PyList_New(size);
    if (!field) {
      return NULL;
    }
    swarm_msgs__msg__PeerState * item;
    for (size_t i = 0; i < size; ++i) {
      item = &(ros_message->peers.data[i]);
      PyObject * pyitem = swarm_msgs__msg__peer_state__convert_to_py(item);
      if (!pyitem) {
        Py_DECREF(field);
        return NULL;
      }
      int rc = PyList_SetItem(field, i, pyitem);
      (void)rc;
      assert(rc == 0);
    }
    assert(PySequence_Check(field));
    {
      int rc = PyObject_SetAttrString(_pymessage, "peers", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // peer_jetson_alive
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->peer_jetson_alive ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "peer_jetson_alive", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // peer_last_heartbeat
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->peer_last_heartbeat);
    {
      int rc = PyObject_SetAttrString(_pymessage, "peer_last_heartbeat", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // own_battery_voltage
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->own_battery_voltage);
    {
      int rc = PyObject_SetAttrString(_pymessage, "own_battery_voltage", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // own_battery_current
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->own_battery_current);
    {
      int rc = PyObject_SetAttrString(_pymessage, "own_battery_current", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // own_battery_remaining
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->own_battery_remaining);
    {
      int rc = PyObject_SetAttrString(_pymessage, "own_battery_remaining", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // radio_rssi
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->radio_rssi);
    {
      int rc = PyObject_SetAttrString(_pymessage, "radio_rssi", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // radio_remrssi
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->radio_remrssi);
    {
      int rc = PyObject_SetAttrString(_pymessage, "radio_remrssi", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // radio_txbuf
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->radio_txbuf);
    {
      int rc = PyObject_SetAttrString(_pymessage, "radio_txbuf", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // radio_noise
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->radio_noise);
    {
      int rc = PyObject_SetAttrString(_pymessage, "radio_noise", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // radio_remnoise
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->radio_remnoise);
    {
      int rc = PyObject_SetAttrString(_pymessage, "radio_remnoise", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // radio_rxerrors
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->radio_rxerrors);
    {
      int rc = PyObject_SetAttrString(_pymessage, "radio_rxerrors", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // radio_link_alive
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->radio_link_alive ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "radio_link_alive", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // radio_last_status
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->radio_last_status);
    {
      int rc = PyObject_SetAttrString(_pymessage, "radio_last_status", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
