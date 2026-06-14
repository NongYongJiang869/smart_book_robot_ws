# generated from rosidl_generator_py/resource/_idl.py.em
# with input from custom_interfaces:msg/ChassisStatus.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_ChassisStatus(type):
    """Metaclass of message 'ChassisStatus'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'ERR_NONE': 0,
        'ERR_ESTOP': 1,
        'ERR_FRONT_COLLISION': 2,
        'ERR_REAR_COLLISION': 4,
        'ERR_COMM_TIMEOUT': 256,
        'ERR_IMU_FAULT': 1024,
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('custom_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'custom_interfaces.msg.ChassisStatus')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__chassis_status
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__chassis_status
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__chassis_status
            cls._TYPE_SUPPORT = module.type_support_msg__msg__chassis_status
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__chassis_status

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'ERR_NONE': cls.__constants['ERR_NONE'],
            'ERR_ESTOP': cls.__constants['ERR_ESTOP'],
            'ERR_FRONT_COLLISION': cls.__constants['ERR_FRONT_COLLISION'],
            'ERR_REAR_COLLISION': cls.__constants['ERR_REAR_COLLISION'],
            'ERR_COMM_TIMEOUT': cls.__constants['ERR_COMM_TIMEOUT'],
            'ERR_IMU_FAULT': cls.__constants['ERR_IMU_FAULT'],
        }

    @property
    def ERR_NONE(self):
        """Message constant 'ERR_NONE'."""
        return Metaclass_ChassisStatus.__constants['ERR_NONE']

    @property
    def ERR_ESTOP(self):
        """Message constant 'ERR_ESTOP'."""
        return Metaclass_ChassisStatus.__constants['ERR_ESTOP']

    @property
    def ERR_FRONT_COLLISION(self):
        """Message constant 'ERR_FRONT_COLLISION'."""
        return Metaclass_ChassisStatus.__constants['ERR_FRONT_COLLISION']

    @property
    def ERR_REAR_COLLISION(self):
        """Message constant 'ERR_REAR_COLLISION'."""
        return Metaclass_ChassisStatus.__constants['ERR_REAR_COLLISION']

    @property
    def ERR_COMM_TIMEOUT(self):
        """Message constant 'ERR_COMM_TIMEOUT'."""
        return Metaclass_ChassisStatus.__constants['ERR_COMM_TIMEOUT']

    @property
    def ERR_IMU_FAULT(self):
        """Message constant 'ERR_IMU_FAULT'."""
        return Metaclass_ChassisStatus.__constants['ERR_IMU_FAULT']


class ChassisStatus(metaclass=Metaclass_ChassisStatus):
    """
    Message class 'ChassisStatus'.

    Constants:
      ERR_NONE
      ERR_ESTOP
      ERR_FRONT_COLLISION
      ERR_REAR_COLLISION
      ERR_COMM_TIMEOUT
      ERR_IMU_FAULT
    """

    __slots__ = [
        '_header',
        '_motor_enabled',
        '_emergency_stop',
        '_collision_front',
        '_collision_rear',
        '_error_code',
        '_cmd_latency_ms',
        '_lost_frames',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'motor_enabled': 'uint8',
        'emergency_stop': 'boolean',
        'collision_front': 'boolean',
        'collision_rear': 'boolean',
        'error_code': 'uint16',
        'cmd_latency_ms': 'float',
        'lost_frames': 'uint16',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.motor_enabled = kwargs.get('motor_enabled', int())
        self.emergency_stop = kwargs.get('emergency_stop', bool())
        self.collision_front = kwargs.get('collision_front', bool())
        self.collision_rear = kwargs.get('collision_rear', bool())
        self.error_code = kwargs.get('error_code', int())
        self.cmd_latency_ms = kwargs.get('cmd_latency_ms', float())
        self.lost_frames = kwargs.get('lost_frames', int())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.header != other.header:
            return False
        if self.motor_enabled != other.motor_enabled:
            return False
        if self.emergency_stop != other.emergency_stop:
            return False
        if self.collision_front != other.collision_front:
            return False
        if self.collision_rear != other.collision_rear:
            return False
        if self.error_code != other.error_code:
            return False
        if self.cmd_latency_ms != other.cmd_latency_ms:
            return False
        if self.lost_frames != other.lost_frames:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def header(self):
        """Message field 'header'."""
        return self._header

    @header.setter
    def header(self, value):
        if __debug__:
            from std_msgs.msg import Header
            assert \
                isinstance(value, Header), \
                "The 'header' field must be a sub message of type 'Header'"
        self._header = value

    @builtins.property
    def motor_enabled(self):
        """Message field 'motor_enabled'."""
        return self._motor_enabled

    @motor_enabled.setter
    def motor_enabled(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'motor_enabled' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'motor_enabled' field must be an unsigned integer in [0, 255]"
        self._motor_enabled = value

    @builtins.property
    def emergency_stop(self):
        """Message field 'emergency_stop'."""
        return self._emergency_stop

    @emergency_stop.setter
    def emergency_stop(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'emergency_stop' field must be of type 'bool'"
        self._emergency_stop = value

    @builtins.property
    def collision_front(self):
        """Message field 'collision_front'."""
        return self._collision_front

    @collision_front.setter
    def collision_front(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'collision_front' field must be of type 'bool'"
        self._collision_front = value

    @builtins.property
    def collision_rear(self):
        """Message field 'collision_rear'."""
        return self._collision_rear

    @collision_rear.setter
    def collision_rear(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'collision_rear' field must be of type 'bool'"
        self._collision_rear = value

    @builtins.property
    def error_code(self):
        """Message field 'error_code'."""
        return self._error_code

    @error_code.setter
    def error_code(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'error_code' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'error_code' field must be an unsigned integer in [0, 65535]"
        self._error_code = value

    @builtins.property
    def cmd_latency_ms(self):
        """Message field 'cmd_latency_ms'."""
        return self._cmd_latency_ms

    @cmd_latency_ms.setter
    def cmd_latency_ms(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'cmd_latency_ms' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'cmd_latency_ms' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._cmd_latency_ms = value

    @builtins.property
    def lost_frames(self):
        """Message field 'lost_frames'."""
        return self._lost_frames

    @lost_frames.setter
    def lost_frames(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'lost_frames' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'lost_frames' field must be an unsigned integer in [0, 65535]"
        self._lost_frames = value
