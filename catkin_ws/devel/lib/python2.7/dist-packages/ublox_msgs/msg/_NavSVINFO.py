# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from ublox_msgs/NavSVINFO.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import ublox_msgs.msg

class NavSVINFO(genpy.Message):
  _md5sum = "869d73cea8ef4943b3f757dcb81282ea"
  _type = "ublox_msgs/NavSVINFO"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """# NAV-SVINFO (0x01 0x30)
# Space Vehicle Information
#

uint8 CLASS_ID = 1
uint8 MESSAGE_ID = 48

uint32 iTOW             # GPS Millisecond time of week [ms]

uint8 numCh             # Number of channels

uint8 globalFlags       # Bitmask
# Chip Hardware generation flags
uint8 CHIPGEN_ANTARIS = 0   # Antaris, Antaris 4
uint8 CHIPGEN_UBLOX5 = 1    # u-blox 5
uint8 CHIPGEN_UBLOX6 = 2    # u-blox 6
uint8 CHIPGEN_UBLOX7 = 3    # u-blox 7
uint8 CHIPGEN_UBLOX8 = 4    # u-blox 8 / u-blox M8

uint16 reserved2        # Reserved

NavSVINFO_SV[] sv

================================================================================
MSG: ublox_msgs/NavSVINFO_SV
# see message NavSVINFO
#

uint8 chn             # Channel number, 255 for SVs not assigned to a channel
uint8 svid            # Satellite ID

uint8 flags           # Bitmask
uint8 FLAGS_SV_USED = 1                     # SV is used for navigation
uint8 FLAGS_DIFF_CORR = 2                   # Differential correction data 
                                            # is available for this SV
uint8 FLAGS_ORBIT_AVAIL = 4                 # Orbit information is available for 
                                            # this SV (Ephemeris or Almanach)
uint8 FLAGS_ORBIT_EPH = 8                   # Orbit information is Ephemeris
uint8 FLAGS_UNHEALTHY = 16                  # SV is unhealthy / shall not be 
                                            # used
uint8 FLAGS_ORBIT_ALM = 32                  # Orbit information is Almanac Plus
uint8 FLAGS_ORBIT_AOP = 64                  # Orbit information is AssistNow 
                                            # Autonomous
uint8 FLAGS_SMOOTHED = 128                  # Carrier smoothed pseudorange used

uint8 quality         # Bitfield
# qualityInd: Signal Quality indicator (range 0..7). The following list shows 
# the meaning of the different QI values:
# Note: Since IMES signals are not time synchronized, a channel tracking an IMES
# signal can never reach a quality indicator value of higher than 3.
uint8 QUALITY_IDLE = 0                      # This channel is idle
uint8 QUALITY_SEARCHING = 1                 # Channel is searching
uint8 QUALITY_ACQUIRED = 2                   # Signal acquired
uint8 QUALITY_DETECTED = 3                  # Signal detected but unusable
uint8 QUALITY_CODE_LOCK = 4                 # Code Lock on Signal
uint8 QUALITY_CODE_AND_CARRIER_LOCKED1 = 5  # Code and Carrier locked 
                                            # and time synchronized
uint8 QUALITY_CODE_AND_CARRIER_LOCKED2 = 6  # Code and Carrier locked 
                                            # and time synchronized
uint8 QUALITY_CODE_AND_CARRIER_LOCKED3 = 7  # Code and Carrier locked 
                                            # and time synchronized

uint8 cno             # Carrier to Noise Ratio (Signal Strength) [dBHz]
int8 elev             # Elevation in integer degrees [deg]
int16 azim            # Azimuth in integer degrees [deg]
int32 prRes           # Pseudo range residual in centimetres [cm]
"""
  # Pseudo-constants
  CLASS_ID = 1
  MESSAGE_ID = 48
  CHIPGEN_ANTARIS = 0
  CHIPGEN_UBLOX5 = 1
  CHIPGEN_UBLOX6 = 2
  CHIPGEN_UBLOX7 = 3
  CHIPGEN_UBLOX8 = 4

  __slots__ = ['iTOW','numCh','globalFlags','reserved2','sv']
  _slot_types = ['uint32','uint8','uint8','uint16','ublox_msgs/NavSVINFO_SV[]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       iTOW,numCh,globalFlags,reserved2,sv

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(NavSVINFO, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.iTOW is None:
        self.iTOW = 0
      if self.numCh is None:
        self.numCh = 0
      if self.globalFlags is None:
        self.globalFlags = 0
      if self.reserved2 is None:
        self.reserved2 = 0
      if self.sv is None:
        self.sv = []
    else:
      self.iTOW = 0
      self.numCh = 0
      self.globalFlags = 0
      self.reserved2 = 0
      self.sv = []

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
      _x = self
      buff.write(_struct_I2BH.pack(_x.iTOW, _x.numCh, _x.globalFlags, _x.reserved2))
      length = len(self.sv)
      buff.write(_struct_I.pack(length))
      for val1 in self.sv:
        _x = val1
        buff.write(_struct_5Bbhi.pack(_x.chn, _x.svid, _x.flags, _x.quality, _x.cno, _x.elev, _x.azim, _x.prRes))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.sv is None:
        self.sv = None
      end = 0
      _x = self
      start = end
      end += 8
      (_x.iTOW, _x.numCh, _x.globalFlags, _x.reserved2,) = _struct_I2BH.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.sv = []
      for i in range(0, length):
        val1 = ublox_msgs.msg.NavSVINFO_SV()
        _x = val1
        start = end
        end += 12
        (_x.chn, _x.svid, _x.flags, _x.quality, _x.cno, _x.elev, _x.azim, _x.prRes,) = _struct_5Bbhi.unpack(str[start:end])
        self.sv.append(val1)
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
      _x = self
      buff.write(_struct_I2BH.pack(_x.iTOW, _x.numCh, _x.globalFlags, _x.reserved2))
      length = len(self.sv)
      buff.write(_struct_I.pack(length))
      for val1 in self.sv:
        _x = val1
        buff.write(_struct_5Bbhi.pack(_x.chn, _x.svid, _x.flags, _x.quality, _x.cno, _x.elev, _x.azim, _x.prRes))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.sv is None:
        self.sv = None
      end = 0
      _x = self
      start = end
      end += 8
      (_x.iTOW, _x.numCh, _x.globalFlags, _x.reserved2,) = _struct_I2BH.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.sv = []
      for i in range(0, length):
        val1 = ublox_msgs.msg.NavSVINFO_SV()
        _x = val1
        start = end
        end += 12
        (_x.chn, _x.svid, _x.flags, _x.quality, _x.cno, _x.elev, _x.azim, _x.prRes,) = _struct_5Bbhi.unpack(str[start:end])
        self.sv.append(val1)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_5Bbhi = struct.Struct("<5Bbhi")
_struct_I2BH = struct.Struct("<I2BH")
