// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from custom_interfaces:msg/ChassisStatus.idl
// generated code does not contain a copyright notice
#include "custom_interfaces/msg/detail/chassis_status__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
custom_interfaces__msg__ChassisStatus__init(custom_interfaces__msg__ChassisStatus * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    custom_interfaces__msg__ChassisStatus__fini(msg);
    return false;
  }
  // motor_enabled
  // emergency_stop
  // collision_front
  // collision_rear
  // error_code
  // cmd_latency_ms
  // lost_frames
  return true;
}

void
custom_interfaces__msg__ChassisStatus__fini(custom_interfaces__msg__ChassisStatus * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // motor_enabled
  // emergency_stop
  // collision_front
  // collision_rear
  // error_code
  // cmd_latency_ms
  // lost_frames
}

bool
custom_interfaces__msg__ChassisStatus__are_equal(const custom_interfaces__msg__ChassisStatus * lhs, const custom_interfaces__msg__ChassisStatus * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // motor_enabled
  if (lhs->motor_enabled != rhs->motor_enabled) {
    return false;
  }
  // emergency_stop
  if (lhs->emergency_stop != rhs->emergency_stop) {
    return false;
  }
  // collision_front
  if (lhs->collision_front != rhs->collision_front) {
    return false;
  }
  // collision_rear
  if (lhs->collision_rear != rhs->collision_rear) {
    return false;
  }
  // error_code
  if (lhs->error_code != rhs->error_code) {
    return false;
  }
  // cmd_latency_ms
  if (lhs->cmd_latency_ms != rhs->cmd_latency_ms) {
    return false;
  }
  // lost_frames
  if (lhs->lost_frames != rhs->lost_frames) {
    return false;
  }
  return true;
}

bool
custom_interfaces__msg__ChassisStatus__copy(
  const custom_interfaces__msg__ChassisStatus * input,
  custom_interfaces__msg__ChassisStatus * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // motor_enabled
  output->motor_enabled = input->motor_enabled;
  // emergency_stop
  output->emergency_stop = input->emergency_stop;
  // collision_front
  output->collision_front = input->collision_front;
  // collision_rear
  output->collision_rear = input->collision_rear;
  // error_code
  output->error_code = input->error_code;
  // cmd_latency_ms
  output->cmd_latency_ms = input->cmd_latency_ms;
  // lost_frames
  output->lost_frames = input->lost_frames;
  return true;
}

custom_interfaces__msg__ChassisStatus *
custom_interfaces__msg__ChassisStatus__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom_interfaces__msg__ChassisStatus * msg = (custom_interfaces__msg__ChassisStatus *)allocator.allocate(sizeof(custom_interfaces__msg__ChassisStatus), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(custom_interfaces__msg__ChassisStatus));
  bool success = custom_interfaces__msg__ChassisStatus__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
custom_interfaces__msg__ChassisStatus__destroy(custom_interfaces__msg__ChassisStatus * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    custom_interfaces__msg__ChassisStatus__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
custom_interfaces__msg__ChassisStatus__Sequence__init(custom_interfaces__msg__ChassisStatus__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom_interfaces__msg__ChassisStatus * data = NULL;

  if (size) {
    data = (custom_interfaces__msg__ChassisStatus *)allocator.zero_allocate(size, sizeof(custom_interfaces__msg__ChassisStatus), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = custom_interfaces__msg__ChassisStatus__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        custom_interfaces__msg__ChassisStatus__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
custom_interfaces__msg__ChassisStatus__Sequence__fini(custom_interfaces__msg__ChassisStatus__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      custom_interfaces__msg__ChassisStatus__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

custom_interfaces__msg__ChassisStatus__Sequence *
custom_interfaces__msg__ChassisStatus__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom_interfaces__msg__ChassisStatus__Sequence * array = (custom_interfaces__msg__ChassisStatus__Sequence *)allocator.allocate(sizeof(custom_interfaces__msg__ChassisStatus__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = custom_interfaces__msg__ChassisStatus__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
custom_interfaces__msg__ChassisStatus__Sequence__destroy(custom_interfaces__msg__ChassisStatus__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    custom_interfaces__msg__ChassisStatus__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
custom_interfaces__msg__ChassisStatus__Sequence__are_equal(const custom_interfaces__msg__ChassisStatus__Sequence * lhs, const custom_interfaces__msg__ChassisStatus__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!custom_interfaces__msg__ChassisStatus__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
custom_interfaces__msg__ChassisStatus__Sequence__copy(
  const custom_interfaces__msg__ChassisStatus__Sequence * input,
  custom_interfaces__msg__ChassisStatus__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(custom_interfaces__msg__ChassisStatus);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    custom_interfaces__msg__ChassisStatus * data =
      (custom_interfaces__msg__ChassisStatus *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!custom_interfaces__msg__ChassisStatus__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          custom_interfaces__msg__ChassisStatus__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!custom_interfaces__msg__ChassisStatus__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
