# generated from rosidl_generator_py/resource/_idl.py.em
# with input from swarm_msgs:msg/PeerStates.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_PeerStates(type):
    """Metaclass of message 'PeerStates'."""

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
                'swarm_msgs.msg.PeerStates')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__peer_states
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__peer_states
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__peer_states
            cls._TYPE_SUPPORT = module.type_support_msg__msg__peer_states
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__peer_states

            from builtin_interfaces.msg import Time
            if Time.__class__._TYPE_SUPPORT is None:
                Time.__class__.__import_type_support__()

            from swarm_msgs.msg import PeerState
            if PeerState.__class__._TYPE_SUPPORT is None:
                PeerState.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class PeerStates(metaclass=Metaclass_PeerStates):
    """Message class 'PeerStates'."""

    __slots__ = [
        '_header_stamp',
        '_peers',
        '_peer_jetson_alive',
        '_peer_last_heartbeat',
        '_own_battery_voltage',
        '_own_battery_current',
        '_own_battery_remaining',
        '_radio_rssi',
        '_radio_remrssi',
        '_radio_txbuf',
        '_radio_noise',
        '_radio_remnoise',
        '_radio_rxerrors',
        '_radio_link_alive',
        '_radio_last_status',
    ]

    _fields_and_field_types = {
        'header_stamp': 'builtin_interfaces/Time',
        'peers': 'sequence<swarm_msgs/PeerState>',
        'peer_jetson_alive': 'boolean',
        'peer_last_heartbeat': 'double',
        'own_battery_voltage': 'float',
        'own_battery_current': 'float',
        'own_battery_remaining': 'float',
        'radio_rssi': 'uint8',
        'radio_remrssi': 'uint8',
        'radio_txbuf': 'uint8',
        'radio_noise': 'uint8',
        'radio_remnoise': 'uint8',
        'radio_rxerrors': 'uint16',
        'radio_link_alive': 'boolean',
        'radio_last_status': 'double',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['builtin_interfaces', 'msg'], 'Time'),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.NamespacedType(['swarm_msgs', 'msg'], 'PeerState')),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from builtin_interfaces.msg import Time
        self.header_stamp = kwargs.get('header_stamp', Time())
        self.peers = kwargs.get('peers', [])
        self.peer_jetson_alive = kwargs.get('peer_jetson_alive', bool())
        self.peer_last_heartbeat = kwargs.get('peer_last_heartbeat', float())
        self.own_battery_voltage = kwargs.get('own_battery_voltage', float())
        self.own_battery_current = kwargs.get('own_battery_current', float())
        self.own_battery_remaining = kwargs.get('own_battery_remaining', float())
        self.radio_rssi = kwargs.get('radio_rssi', int())
        self.radio_remrssi = kwargs.get('radio_remrssi', int())
        self.radio_txbuf = kwargs.get('radio_txbuf', int())
        self.radio_noise = kwargs.get('radio_noise', int())
        self.radio_remnoise = kwargs.get('radio_remnoise', int())
        self.radio_rxerrors = kwargs.get('radio_rxerrors', int())
        self.radio_link_alive = kwargs.get('radio_link_alive', bool())
        self.radio_last_status = kwargs.get('radio_last_status', float())

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
        if self.header_stamp != other.header_stamp:
            return False
        if self.peers != other.peers:
            return False
        if self.peer_jetson_alive != other.peer_jetson_alive:
            return False
        if self.peer_last_heartbeat != other.peer_last_heartbeat:
            return False
        if self.own_battery_voltage != other.own_battery_voltage:
            return False
        if self.own_battery_current != other.own_battery_current:
            return False
        if self.own_battery_remaining != other.own_battery_remaining:
            return False
        if self.radio_rssi != other.radio_rssi:
            return False
        if self.radio_remrssi != other.radio_remrssi:
            return False
        if self.radio_txbuf != other.radio_txbuf:
            return False
        if self.radio_noise != other.radio_noise:
            return False
        if self.radio_remnoise != other.radio_remnoise:
            return False
        if self.radio_rxerrors != other.radio_rxerrors:
            return False
        if self.radio_link_alive != other.radio_link_alive:
            return False
        if self.radio_last_status != other.radio_last_status:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def header_stamp(self):
        """Message field 'header_stamp'."""
        return self._header_stamp

    @header_stamp.setter
    def header_stamp(self, value):
        if __debug__:
            from builtin_interfaces.msg import Time
            assert \
                isinstance(value, Time), \
                "The 'header_stamp' field must be a sub message of type 'Time'"
        self._header_stamp = value

    @builtins.property
    def peers(self):
        """Message field 'peers'."""
        return self._peers

    @peers.setter
    def peers(self, value):
        if __debug__:
            from swarm_msgs.msg import PeerState
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, PeerState) for v in value) and
                 True), \
                "The 'peers' field must be a set or sequence and each value of type 'PeerState'"
        self._peers = value

    @builtins.property
    def peer_jetson_alive(self):
        """Message field 'peer_jetson_alive'."""
        return self._peer_jetson_alive

    @peer_jetson_alive.setter
    def peer_jetson_alive(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'peer_jetson_alive' field must be of type 'bool'"
        self._peer_jetson_alive = value

    @builtins.property
    def peer_last_heartbeat(self):
        """Message field 'peer_last_heartbeat'."""
        return self._peer_last_heartbeat

    @peer_last_heartbeat.setter
    def peer_last_heartbeat(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'peer_last_heartbeat' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'peer_last_heartbeat' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._peer_last_heartbeat = value

    @builtins.property
    def own_battery_voltage(self):
        """Message field 'own_battery_voltage'."""
        return self._own_battery_voltage

    @own_battery_voltage.setter
    def own_battery_voltage(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'own_battery_voltage' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'own_battery_voltage' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._own_battery_voltage = value

    @builtins.property
    def own_battery_current(self):
        """Message field 'own_battery_current'."""
        return self._own_battery_current

    @own_battery_current.setter
    def own_battery_current(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'own_battery_current' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'own_battery_current' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._own_battery_current = value

    @builtins.property
    def own_battery_remaining(self):
        """Message field 'own_battery_remaining'."""
        return self._own_battery_remaining

    @own_battery_remaining.setter
    def own_battery_remaining(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'own_battery_remaining' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'own_battery_remaining' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._own_battery_remaining = value

    @builtins.property
    def radio_rssi(self):
        """Message field 'radio_rssi'."""
        return self._radio_rssi

    @radio_rssi.setter
    def radio_rssi(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'radio_rssi' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'radio_rssi' field must be an unsigned integer in [0, 255]"
        self._radio_rssi = value

    @builtins.property
    def radio_remrssi(self):
        """Message field 'radio_remrssi'."""
        return self._radio_remrssi

    @radio_remrssi.setter
    def radio_remrssi(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'radio_remrssi' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'radio_remrssi' field must be an unsigned integer in [0, 255]"
        self._radio_remrssi = value

    @builtins.property
    def radio_txbuf(self):
        """Message field 'radio_txbuf'."""
        return self._radio_txbuf

    @radio_txbuf.setter
    def radio_txbuf(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'radio_txbuf' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'radio_txbuf' field must be an unsigned integer in [0, 255]"
        self._radio_txbuf = value

    @builtins.property
    def radio_noise(self):
        """Message field 'radio_noise'."""
        return self._radio_noise

    @radio_noise.setter
    def radio_noise(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'radio_noise' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'radio_noise' field must be an unsigned integer in [0, 255]"
        self._radio_noise = value

    @builtins.property
    def radio_remnoise(self):
        """Message field 'radio_remnoise'."""
        return self._radio_remnoise

    @radio_remnoise.setter
    def radio_remnoise(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'radio_remnoise' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'radio_remnoise' field must be an unsigned integer in [0, 255]"
        self._radio_remnoise = value

    @builtins.property
    def radio_rxerrors(self):
        """Message field 'radio_rxerrors'."""
        return self._radio_rxerrors

    @radio_rxerrors.setter
    def radio_rxerrors(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'radio_rxerrors' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'radio_rxerrors' field must be an unsigned integer in [0, 65535]"
        self._radio_rxerrors = value

    @builtins.property
    def radio_link_alive(self):
        """Message field 'radio_link_alive'."""
        return self._radio_link_alive

    @radio_link_alive.setter
    def radio_link_alive(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'radio_link_alive' field must be of type 'bool'"
        self._radio_link_alive = value

    @builtins.property
    def radio_last_status(self):
        """Message field 'radio_last_status'."""
        return self._radio_last_status

    @radio_last_status.setter
    def radio_last_status(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'radio_last_status' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'radio_last_status' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._radio_last_status = value
