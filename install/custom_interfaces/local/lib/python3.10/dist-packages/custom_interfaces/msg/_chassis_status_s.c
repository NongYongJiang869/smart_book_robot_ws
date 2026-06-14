// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from custom_interfaces:msg/ChassisStatus.idl
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
#include "custom_interfaces/msg/detail/chassis_status__struct.h"
#include "custom_interfaces/msg/detail/chassis_status__functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__header__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__header__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool custom_interfaces__msg__chassis_status__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[52];
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
    assert(strncmp("custom_interfaces.msg._chassis_status.ChassisStatus", full_classname_dest, 51) == 0);
  }
  custom_interfaces__msg__ChassisStatus * ros_message = _ros_message;
  {  // header
    PyObject * field = PyObject_GetAttrString(_pymsg, "header");
    if (!field) {
      return false;
    }
    if (!std_msgs__msg__header__convert_from_py(field, &ros_message->header)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // motor_enabled
    PyObject * field = PyObject_GetAttrString(_pymsg, "motor_enabled");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->motor_enabled = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // emergency_stop
    PyObject * field = PyObject_GetAttrString(_pymsg, "emergency_stop");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->emergency_stop = (Py_True == field);
    Py_DECREF(field);
  }
  {  // collision_front
    PyObject * field = PyObject_GetAttrString(_pymsg, "collision_front");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->collision_front = (Py_True == field);
    Py_DECREF(field);
  }
  {  // collision_rear
    PyObject * field = PyObject_GetAttrString(_pymsg, "collision_rear");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->collision_rear = (Py_True == field);
    Py_DECREF(field);
  }
  {  // error_code
    PyObject * field = PyObject_GetAttrString(_pymsg, "error_code");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->error_code = (uint16_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // cmd_latency_ms
    PyObject * field = PyObject_GetAttrString(_pymsg, "cmd_latency_ms");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->cmd_latency_ms = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // lost_frames
    PyObject * field = PyObject_GetAttrString(_pymsg, "lost_frames");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->lost_frames = (uint16_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * custom_interfaces__msg__chassis_status__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of ChassisStatus */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("custom_interfaces.msg._chassis_status");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "ChassisStatus");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  custom_interfaces__msg__ChassisStatus * ros_message = (custom_interfaces__msg__ChassisStatus *)raw_ros_message;
  {  // header
    PyObject * field = NULL;
    field = std_msgs__msg__header__convert_to_py(&ros_message->header);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "header", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // motor_enabled
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->motor_enabled);
    {
      int rc = PyObject_SetAttrString(_pymessage, "motor_enabled", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // emergency_stop
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->emergency_stop ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "emergency_stop", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // collision_front
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->collision_front ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "collision_front", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // collision_rear
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->collision_rear ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "collision_rear", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // error_code
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->error_code);
    {
      int rc = PyObject_SetAttrString(_pymessage, "error_code", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // cmd_latency_ms
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->cmd_latency_ms);
    {
      int rc = PyObject_SetAttrString(_pymessage, "cmd_latency_ms", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // lost_frames
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->lost_frames);
    {
      int rc = PyObject_SetAttrString(_pymessage, "lost_frames", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
