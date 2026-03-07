# generated from rosidl_generator_py/resource/_idl.py.em
# with input from swarm_msgs:msg/SwarmCommand.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_SwarmCommand(type):
    """Metaclass of message 'SwarmCommand'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'CMD_RTL': 1,
        'CMD_LAND': 2,
        'CMD_KILL': 3,
        'CMD_FOLLOW': 4,
        'CMD_HOVER': 5,
        'CMD_TAKEOFF': 6,
        'CMD_WASD': 10,
        'CMD_WAYPOINT': 11,
        'CMD_SPEED': 12,
        'CMD_ALTITUDE': 13,
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('swarm_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'swarm_msgs.msg.SwarmCommand')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__swarm_command
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__swarm_command
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__swarm_command
            cls._TYPE_SUPPORT = module.type_support_msg__msg__swarm_command
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__swarm_command

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'CMD_RTL': cls.__constants['CMD_RTL'],
            'CMD_LAND': cls.__constants['CMD_LAND'],
            'CMD_KILL': cls.__constants['CMD_KILL'],
            'CMD_FOLLOW': cls.__constants['CMD_FOLLOW'],
            'CMD_HOVER': cls.__constants['CMD_HOVER'],
            'CMD_TAKEOFF': cls.__constants['CMD_TAKEOFF'],
            'CMD_WASD': cls.__constants['CMD_WASD'],
            'CMD_WAYPOINT': cls.__constants['CMD_WAYPOINT'],
            'CMD_SPEED': cls.__constants['CMD_SPEED'],
            'CMD_ALTITUDE': cls.__constants['CMD_ALTITUDE'],
        }

    @property
    def CMD_RTL(self):
        """Message constant 'CMD_RTL'."""
        return Metaclass_SwarmCommand.__constants['CMD_RTL']

    @property
    def CMD_LAND(self):
        """Message constant 'CMD_LAND'."""
        return Metaclass_SwarmCommand.__constants['CMD_LAND']

    @property
    def CMD_KILL(self):
        """Message constant 'CMD_KILL'."""
        return Metaclass_SwarmCommand.__constants['CMD_KILL']

    @property
    def CMD_FOLLOW(self):
        """Message constant 'CMD_FOLLOW'."""
        return Metaclass_SwarmCommand.__constants['CMD_FOLLOW']

    @property
    def CMD_HOVER(self):
        """Message constant 'CMD_HOVER'."""
        return Metaclass_SwarmCommand.__constants['CMD_HOVER']

    @property
    def CMD_TAKEOFF(self):
        """Message constant 'CMD_TAKEOFF'."""
        return Metaclass_SwarmCommand.__constants['CMD_TAKEOFF']

    @property
    def CMD_WASD(self):
        """Message constant 'CMD_WASD'."""
        return Metaclass_SwarmCommand.__constants['CMD_WASD']

    @property
    def CMD_WAYPOINT(self):
        """Message constant 'CMD_WAYPOINT'."""
        return Metaclass_SwarmCommand.__constants['CMD_WAYPOINT']

    @property
    def CMD_SPEED(self):
        """Message constant 'CMD_SPEED'."""
        return Metaclass_SwarmCommand.__constants['CMD_SPEED']

    @property
    def CMD_ALTITUDE(self):
        """Message constant 'CMD_ALTITUDE'."""
        return Metaclass_SwarmCommand.__constants['CMD_ALTITUDE']


class SwarmCommand(metaclass=Metaclass_SwarmCommand):
    """
    Message class 'SwarmCommand'.

    Constants:
      CMD_RTL
      CMD_LAND
      CMD_KILL
      CMD_FOLLOW
      CMD_HOVER
      CMD_TAKEOFF
      CMD_WASD
      CMD_WAYPOINT
      CMD_SPEED
      CMD_ALTITUDE
    """

    __slots__ = [
        '_cmd',
        '_vn',
        '_ve',
        '_vd',
        '_lat',
        '_lon',
        '_speed',
    ]

    _fields_and_field_types = {
        'cmd': 'uint8',
        'vn': 'float',
        've': 'float',
        'vd': 'float',
        'lat': 'double',
        'lon': 'double',
        'speed': 'float',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.cmd = kwargs.get('cmd', int())
        self.vn = kwargs.get('vn', float())
        self.ve = kwargs.get('ve', float())
        self.vd = kwargs.get('vd', float())
        self.lat = kwargs.get('lat', float())
        self.lon = kwargs.get('lon', float())
        self.speed = kwargs.get('speed', float())

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
        if self.cmd != other.cmd:
            return False
        if self.vn != other.vn:
            return False
        if self.ve != other.ve:
            return False
        if self.vd != other.vd:
            return False
        if self.lat != other.lat:
            return False
        if self.lon != other.lon:
            return False
        if self.speed != other.speed:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def cmd(self):
        """Message field 'cmd'."""
        return self._cmd

    @cmd.setter
    def cmd(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'cmd' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'cmd' field must be an unsigned integer in [0, 255]"
        self._cmd = value

    @builtins.property
    def vn(self):
        """Message field 'vn'."""
        return self._vn

    @vn.setter
    def vn(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'vn' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'vn' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._vn = value

    @builtins.property
    def ve(self):
        """Message field 've'."""
        return self._ve

    @ve.setter
    def ve(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 've' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 've' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._ve = value

    @builtins.property
    def vd(self):
        """Message field 'vd'."""
        return self._vd

    @vd.setter
    def vd(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'vd' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'vd' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._vd = value

    @builtins.property
    def lat(self):
        """Message field 'lat'."""
        return self._lat

    @lat.setter
    def lat(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'lat' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'lat' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._lat = value

    @builtins.property
    def lon(self):
        """Message field 'lon'."""
        return self._lon

    @lon.setter
    def lon(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'lon' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'lon' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._lon = value

    @builtins.property
    def speed(self):
        """Message field 'speed'."""
        return self._speed

    @speed.setter
    def speed(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'speed' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'speed' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._speed = value
