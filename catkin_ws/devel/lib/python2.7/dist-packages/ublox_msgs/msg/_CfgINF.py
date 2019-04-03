# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from ublox_msgs/CfgINF.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import ublox_msgs.msg

class CfgINF(genpy.Message):
  _md5sum = "42fb321cf0c63684f2f7086e850ed61e"
  _type = "ublox_msgs/CfgINF"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """# CFG-INF  (0x06 0x02)
# Information message configuration
#
# The value of infMsgMask[x] below are that each bit represents one of the INF 
# class messages (Bit 0 for ERROR, Bit 1 for WARNING and so on.). For a complete 
# list, see the Message Class INF. Several configurations can be concatenated to 
# one input message.
# In this case the payload length can be a multiple of the normal length. Output 
# messages from the module contain only one configuration unit. Note that I/O 
# Ports 1 and 2 correspond to serial ports 1 and 2. I/O port 0 is DDC. I/O port 
# 3 is USB. I/O port 4 is SPI. I/O port 5 is reserved for future use
#

uint8 CLASS_ID = 6
uint8 MESSAGE_ID = 2

# start of repeated block
CfgINF_Block[] blocks
# end of repeated block

================================================================================
MSG: ublox_msgs/CfgINF_Block
# See CfgINF message
#

uint8 protocolID          # Protocol Identifier, identifying for which
                          # protocol the configuration is set/get. The
                          # following are valid Protocol Identifiers:
                          # 0: UBX Protocol
                          # 1: NMEA Protocol
                          # 2-255: Reserved
uint8 PROTOCOL_ID_UBX = 0
uint8 PROTOCOL_ID_NMEA = 1

uint8[3] reserved1        # Reserved

uint8[6] infMsgMask       # A bit mask, saying which information messages
                          # are enabled on each I/O port
uint8 INF_MSG_ERROR = 1              # enable ERROR
uint8 INF_MSG_WARNING = 2            # enable WARNING
uint8 INF_MSG_NOTICE = 4             # enable NOTICE
uint8 INF_MSG_TEST = 8               # enable TEST
uint8 INF_MSG_DEBUG = 16             # enable DEBUG"""
  # Pseudo-constants
  CLASS_ID = 6
  MESSAGE_ID = 2

  __slots__ = ['blocks']
  _slot_types = ['ublox_msgs/CfgINF_Block[]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       blocks

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(CfgINF, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.blocks is None:
        self.blocks = []
    else:
      self.blocks = []

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      length = len(self.blocks)
      buff.write(_struct_I.pack(length))
      for val1 in self.blocks:
        buff.write(_struct_B.pack(val1.protocolID))
        _x = val1.reserved1
        # - if encoded as a list instead, serialize as bytes instead of string
        if type(_x) in [list, tuple]:
          buff.write(_struct_3B.pack(*_x))
        else:
          buff.write(_struct_3s.pack(_x))
        _x = val1.infMsgMask
        # - if encoded as a list instead, serialize as bytes instead of string
        if type(_x) in [list, tuple]:
          buff.write(_struct_6B.pack(*_x))
        else:
          buff.write(_struct_6s.pack(_x))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.blocks is None:
        self.blocks = None
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.blocks = []
      for i in range(0, length):
        val1 = ublox_msgs.msg.CfgINF_Block()
        start = end
        end += 1
        (val1.protocolID,) = _struct_B.unpack(str[start:end])
        start = end
        end += 3
        val1.reserved1 = str[start:end]
        start = end
        end += 6
        val1.infMsgMask = str[start:end]
        self.blocks.append(val1)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      length = len(self.blocks)
      buff.write(_struct_I.pack(length))
      for val1 in self.blocks:
        buff.write(_struct_B.pack(val1.protocolID))
        _x = val1.reserved1
        # - if encoded as a list instead, serialize as bytes instead of string
        if type(_x) in [list, tuple]:
          buff.write(_struct_3B.pack(*_x))
        else:
          buff.write(_struct_3s.pack(_x))
        _x = val1.infMsgMask
        # - if encoded as a list instead, serialize as bytes instead of string
        if type(_x) in [list, tuple]:
          buff.write(_struct_6B.pack(*_x))
        else:
          buff.write(_struct_6s.pack(_x))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.blocks is None:
        self.blocks = None
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.blocks = []
      for i in range(0, length):
        val1 = ublox_msgs.msg.CfgINF_Block()
        start = end
        end += 1
        (val1.protocolID,) = _struct_B.unpack(str[start:end])
        start = end
        end += 3
        val1.reserved1 = str[start:end]
        start = end
        end += 6
        val1.infMsgMask = str[start:end]
        self.blocks.append(val1)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_3s = struct.Struct("<3s")
_struct_3B = struct.Struct("<3B")
_struct_6B = struct.Struct("<6B")
_struct_B = struct.Struct("<B")
_struct_6s = struct.Struct("<6s")