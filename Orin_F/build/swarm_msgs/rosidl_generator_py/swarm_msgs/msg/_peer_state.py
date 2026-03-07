# generated from rosidl_generator_py/resource/_idl.py.em
# with input from swarm_msgs:msg/PeerState.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_PeerState(type):
    """Metaclass of message 'PeerState'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
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
                'swarm_msgs.msg.PeerState')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__peer_state
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__peer_state
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__peer_state
            cls._TYPE_SUPPORT = module.type_support_msg__msg__peer_state
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__peer_state

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class PeerState(metaclass=Metaclass_PeerState):
    """Message class 'PeerState'."""

    __slots__ = [
        '_drone_id',
        '_latitude',
        '_longitude',
        '_altitude',
        '_vn',
        '_ve',
        '_vd',
        '_heading',
        '_stamp',
        '_guidance_mode',
        '_fc_mode_code',
        '_fc_armed',
        '_fc_connected',
        '_battery_voltage',
        '_battery_current',
        '_battery_remaining',
    ]

    _fields_and_field_types = {
        'drone_id': 'uint32',
        'latitude': 'double',
        'longitude': 'double',
        'altitude': 'double',
        'vn': 'double',
        've': 'double',
        'vd': 'double',
        'heading': 'double',
        'stamp': 'double',
        'guidance_mode': 'uint8',
        'fc_mode_code': 'uint8',
        'fc_armed': 'boolean',
        'fc_connected': 'boolean',
        'battery_voltage': 'float',
        'battery_current': 'float',
        'battery_remaining': 'float',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.drone_id = kwargs.get('drone_id', int())
        self.latitude = kwargs.get('latitude', float())
        self.longitude = kwargs.get('longitude', float())
        self.altitude = kwargs.get('altitude', float())
        self.vn = kwargs.get('vn', float())
        self.ve = kwargs.get('ve', float())
        self.vd = kwargs.get('vd', float())
        self.heading = kwargs.get('heading', float())
        self.stamp = kwargs.get('stamp', float())
        self.guidance_mode = kwargs.get('guidance_mode', int())
        self.fc_mode_code = kwargs.get('fc_mode_code', int())
        self.fc_armed = kwargs.get('fc_armed', bool())
        self.fc_connected = kwargs.get('fc_connected', bool())
        self.battery_voltage = kwargs.get('battery_voltage', float())
        self.battery_current = kwargs.get('battery_current', float())
        self.battery_remaining = kwargs.get('battery_remaining', float())

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
        if self.drone_id != other.drone_id:
            return False
        if self.latitude != other.latitude:
            return False
        if self.longitude != other.longitude:
            return False
        if self.altitude != other.altitude:
            return False
        if self.vn != other.vn:
            return False
        if self.ve != other.ve:
            return False
        if self.vd != other.vd:
            return False
        if self.heading != other.heading:
            return False
        if self.stamp != other.stamp:
            return False
        if self.guidance_mode != other.guidance_mode:
            return False
        if self.fc_mode_code != other.fc_mode_code:
            return False
        if self.fc_armed != other.fc_armed:
            return False
        if self.fc_connected != other.fc_connected:
            return False
        if self.battery_voltage != other.battery_voltage:
            return False
        if self.battery_current != other.battery_current:
            return False
        if self.battery_remaining != other.battery_remaining:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def drone_id(self):
        """Message field 'drone_id'."""
        return self._drone_id

    @drone_id.setter
    def drone_id(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'drone_id' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'drone_id' field must be an unsigned integer in [0, 4294967295]"
        self._drone_id = value

    @builtins.property
    def latitude(self):
        """Message field 'latitude'."""
        return self._latitude

    @latitude.setter
    def latitude(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'latitude' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'latitude' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._latitude = value

    @builtins.property
    def longitude(self):
        """Message field 'longitude'."""
        return self._longitude

    @longitude.setter
    def longitude(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'longitude' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'longitude' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._longitude = value

    @builtins.property
    def altitude(self):
        """Message field 'altitude'."""
        return self._altitude

    @altitude.setter
    def altitude(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'altitude' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'altitude' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._altitude = value

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
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'vn' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
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
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 've' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
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
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'vd' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._vd = value

    @builtins.property
    def heading(self):
        """Message field 'heading'."""
        return self._heading

    @heading.setter
    def heading(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'heading' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'heading' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._heading = value

    @builtins.property
    def stamp(self):
        """Message field 'stamp'."""
        return self._stamp

    @stamp.setter
    def stamp(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'stamp' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'stamp' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._stamp = value

    @builtins.property
    def guidance_mode(self):
        """Message field 'guidance_mode'."""
        return self._guidance_mode

    @guidance_mode.setter
    def guidance_mode(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'guidance_mode' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'guidance_mode' field must be an unsigned integer in [0, 255]"
        self._guidance_mode = value

    @builtins.property
    def fc_mode_code(self):
        """Message field 'fc_mode_code'."""
        return self._fc_mode_code

    @fc_mode_code.setter
    def fc_mode_code(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'fc_mode_code' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'fc_mode_code' field must be an unsigned integer in [0, 255]"
        self._fc_mode_code = value

    @builtins.property
    def fc_armed(self):
        """Message field 'fc_armed'."""
        return self._fc_armed

    @fc_armed.setter
    def fc_armed(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'fc_armed' field must be of type 'bool'"
        self._fc_armed = value

    @builtins.property
    def fc_connected(self):
        """Message field 'fc_connected'."""
        return self._fc_connected

    @fc_connected.setter
    def fc_connected(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'fc_connected' field must be of type 'bool'"
        self._fc_connected = value

    @builtins.property
    def battery_voltage(self):
        """Message field 'battery_voltage'."""
        return self._battery_voltage

    @battery_voltage.setter
    def battery_voltage(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'battery_voltage' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'battery_voltage' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._battery_voltage = value

    @builtins.property
    def battery_current(self):
        """Message field 'battery_current'."""
        return self._battery_current

    @battery_current.setter
    def battery_current(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'battery_current' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'battery_current' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._battery_current = value

    @builtins.property
    def battery_remaining(self):
        """Message field 'battery_remaining'."""
        return self._battery_remaining

    @battery_remaining.setter
    def battery_remaining(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'battery_remaining' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'battery_remaining' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._battery_remaining = value
