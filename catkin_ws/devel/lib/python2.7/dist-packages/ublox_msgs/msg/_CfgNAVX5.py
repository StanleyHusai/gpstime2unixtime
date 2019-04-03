# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from ublox_msgs/CfgNAVX5.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class CfgNAVX5(genpy.Message):
  _md5sum = "10b967e4bf2a0c03e74705b79c79a211"
  _type = "ublox_msgs/CfgNAVX5"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """# CFG-NAVX5 (0x06 0x23)
# Navigation Engine Expert Settings
#
# Warning: Refer to u-blox protocol spec before changing these settings.

uint8 CLASS_ID = 6
uint8 MESSAGE_ID = 35

uint16 version        # Message version (set to 0)

uint16 mask1          # First parameters bitmask (possible values below)
uint16 MASK1_MIN_MAX        = 4        # apply min/max SVs settings
uint16 MASK1_MIN_CNO        = 8        # apply minimum C/N0 setting
uint16 MASK1_INITIAL_FIX_3D = 64       # apply initial 3D fix settings
uint16 MASK1_WKN_ROLL       = 512      # apply GPS week number rollover settings
uint16 MASK1_ACK_AID        = 1024     # apply assistance acknowledgment 
                                       # settings
uint16 MASK1_PPP            = 8192     # apply usePPP flag
uint16 MASK1_AOP            = 16384    # apply aopCfg (useAOP flag) and 
                                       # aopOrbMaxErr settings
                                       # (AssistNow Autonomous)

uint32 mask2          # Second parameters bitmask (possible values below)
                      # Firmware >=8 only
uint32 MASK2_ADR = 64                    # Apply ADR sensor fusion on/off 
                                         # setting
uint32 MASK2_SIG_ATTEN_COMP_MODE = 128   # Apply signal attenuation 
                                         # compensation feature settings

uint8[2] reserved1      # Always set to zero

uint8 minSVs            # Minimum number of satellites for navigation
uint8 maxSVs            # Maximum number of satellites for navigation
uint8 minCNO            # Minimum satellite signal level for navigation [dBHz]

uint8 reserved2         # Always set to zero

uint8 iniFix3D          # If set to 1, initial fix must be 3D

uint8[2] reserved3      # Always set to zero

uint8 ackAiding         # If set to 1, issue acknowledgments for assistance
uint16 wknRollover      # GPS week rollover number, GPS week numbers will be set 
                        # correctly from this week up to 1024 weeks after this 
                        # week
uint8 sigAttenCompMode  # Permanently attenuated signal compensation [dBHz]
                        # 0 = disabled, 255 = automatic
                        # 1..63 = maximum expected C/N0 value
                        # Firmware 8 only

uint8[5] reserved4      # Always set to zero

uint8 usePPP            # Enable/disable PPP (on supported units)
uint8 aopCfg            # AssistNow Autonomous configuration, 1: enabled

uint8[2] reserved5      # Always set to zero

uint16 aopOrbMaxErr     # Maximum acceptable (modeled) autonomous orbit 
                        # error [m]
                        # valid range = 5..1000
                        # 0 = reset to firmware default

uint8[7] reserved6      # Always set to zero

uint8 useAdr            # Enable/disable ADR sensor fusion 
                        # 1: enabled, 0: disabled
                        # Only supported on certain products 
"""
  # Pseudo-constants
  CLASS_ID = 6
  MESSAGE_ID = 35
  MASK1_MIN_MAX = 4
  MASK1_MIN_CNO = 8
  MASK1_INITIAL_FIX_3D = 64
  MASK1_WKN_ROLL = 512
  MASK1_ACK_AID = 1024
  MASK1_PPP = 8192
  MASK1_AOP = 16384
  MASK2_ADR = 64
  MASK2_SIG_ATTEN_COMP_MODE = 128

  __slots__ = ['version','mask1','mask2','reserved1','minSVs','maxSVs','minCNO','reserved2','iniFix3D','reserved3','ackAiding','wknRollover','sigAttenCompMode','reserved4','usePPP','aopCfg','reserved5','aopOrbMaxErr','reserved6','useAdr']
  _slot_types = ['uint16','uint16','uint32','uint8[2]','uint8','uint8','uint8','uint8','uint8','uint8[2]','uint8','uint16','uint8','uint8[5]','uint8','uint8','uint8[2]','uint16','uint8[7]','uint8']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       version,mask1,mask2,reserved1,minSVs,maxSVs,minCNO,reserved2,iniFix3D,reserved3,ackAiding,wknRollover,sigAttenCompMode,reserved4,usePPP,aopCfg,reserved5,aopOrbMaxErr,reserved6,useAdr

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(CfgNAVX5, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.version is None:
        self.version = 0
      if self.mask1 is None:
        self.mask1 = 0
      if self.mask2 is None:
        self.mask2 = 0
      if self.reserved1 is None:
        self.reserved1 = chr(0)*2
      if self.minSVs is None:
        self.minSVs = 0
      if self.maxSVs is None:
        self.maxSVs = 0
      if self.minCNO is None:
        self.minCNO = 0
      if self.reserved2 is None:
        self.reserved2 = 0
      if self.iniFix3D is None:
        self.iniFix3D = 0
      if self.reserved3 is None:
        self.reserved3 = chr(0)*2
      if self.ackAiding is None:
        self.ackAiding = 0
      if self.wknRollover is None:
        self.wknRollover = 0
      if self.sigAttenCompMode is None:
        self.sigAttenCompMode = 0
      if self.reserved4 is None:
        self.reserved4 = chr(0)*5
      if self.usePPP is None:
        self.usePPP = 0
      if self.aopCfg is None:
        self.aopCfg = 0
      if self.reserved5 is None:
        self.reserved5 = chr(0)*2
      if self.aopOrbMaxErr is None:
        self.aopOrbMaxErr = 0
      if self.reserved6 is None:
        self.reserved6 = chr(0)*7
      if self.useAdr is None:
        self.useAdr = 0
    else:
      self.version = 0
      self.mask1 = 0
      self.mask2 = 0
      self.reserved1 = chr(0)*2
      self.minSVs = 0
      self.maxSVs = 0
      self.minCNO = 0
      self.reserved2 = 0
      self.iniFix3D = 0
      self.reserved3 = chr(0)*2
      self.ackAiding = 0
      self.wknRollover = 0
      self.sigAttenCompMode = 0
      self.reserved4 = chr(0)*5
      self.usePPP = 0
      self.aopCfg = 0
      self.reserved5 = chr(0)*2
      self.aopOrbMaxErr = 0
      self.reserved6 = chr(0)*7
      self.useAdr = 0

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
      buff.write(_struct_2HI.pack(_x.version, _x.mask1, _x.mask2))
      _x = self.reserved1
      # - if encoded as a list instead, serialize as bytes instead of string
      if type(_x) in [list, tuple]:
        buff.write(_struct_2B.pack(*_x))
      else:
        buff.write(_struct_2s.pack(_x))
      _x = self
      buff.write(_struct_5B.pack(_x.minSVs, _x.maxSVs, _x.minCNO, _x.reserved2, _x.iniFix3D))
      _x = self.reserved3
      # - if encoded as a list instead, serialize as bytes instead of string
      if type(_x) in [list, tuple]:
        buff.write(_struct_2B.pack(*_x))
      else:
        buff.write(_struct_2s.pack(_x))
      _x = self
      buff.write(_struct_BHB.pack(_x.ackAiding, _x.wknRollover, _x.sigAttenCompMode))
      _x = self.reserved4
      # - if encoded as a list instead, serialize as bytes instead of string
      if type(_x) in [list, tuple]:
        buff.write(_struct_5B.pack(*_x))
      else:
        buff.write(_struct_5s.pack(_x))
      _x = self
      buff.write(_struct_2B.pack(_x.usePPP, _x.aopCfg))
      _x = self.reserved5
      # - if encoded as a list instead, serialize as bytes instead of string
      if type(_x) in [list, tuple]:
        buff.write(_struct_2B.pack(*_x))
      else:
        buff.write(_struct_2s.pack(_x))
      buff.write(_struct_H.pack(self.aopOrbMaxErr))
      _x = self.reserved6
      # - if encoded as a list instead, serialize as bytes instead of string
      if type(_x) in [list, tuple]:
        buff.write(_struct_7B.pack(*_x))
      else:
        buff.write(_struct_7s.pack(_x))
      buff.write(_struct_B.pack(self.useAdr))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
      _x = self
      start = end
      end += 8
      (_x.version, _x.mask1, _x.mask2,) = _struct_2HI.unpack(str[start:end])
      start = end
      end += 2
      self.reserved1 = str[start:end]
      _x = self
      start = end
      end += 5
      (_x.minSVs, _x.maxSVs, _x.minCNO, _x.reserved2, _x.iniFix3D,) = _struct_5B.unpack(str[start:end])
      start = end
      end += 2
      self.reserved3 = str[start:end]
      _x = self
      start = end
      end += 4
      (_x.ackAiding, _x.wknRollover, _x.sigAttenCompMode,) = _struct_BHB.unpack(str[start:end])
      start = end
      end += 5
      self.reserved4 = str[start:end]
      _x = self
      start = end
      end += 2
      (_x.usePPP, _x.aopCfg,) = _struct_2B.unpack(str[start:end])
      start = end
      end += 2
      self.reserved5 = str[start:end]
      start = end
      end += 2
      (self.aopOrbMaxErr,) = _struct_H.unpack(str[start:end])
      start = end
      end += 7
      self.reserved6 = str[start:end]
      start = end
      end += 1
      (self.useAdr,) = _struct_B.unpack(str[start:end])
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
      buff.write(_struct_2HI.pack(_x.version, _x.mask1, _x.mask2))
      _x = self.reserved1
      # - if encoded as a list instead, serialize as bytes instead of string
      if type(_x) in [list, tuple]:
        buff.write(_struct_2B.pack(*_x))
      else:
        buff.write(_struct_2s.pack(_x))
      _x = self
      buff.write(_struct_5B.pack(_x.minSVs, _x.maxSVs, _x.minCNO, _x.reserved2, _x.iniFix3D))
      _x = self.reserved3
      # - if encoded as a list instead, serialize as bytes instead of string
      if type(_x) in [list, tuple]:
        buff.write(_struct_2B.pack(*_x))
      else:
        buff.write(_struct_2s.pack(_x))
      _x = self
      buff.write(_struct_BHB.pack(_x.ackAiding, _x.wknRollover, _x.sigAttenCompMode))
      _x = self.reserved4
      # - if encoded as a list instead, serialize as bytes instead of string
      if type(_x) in [list, tuple]:
        buff.write(_struct_5B.pack(*_x))
      else:
        buff.write(_struct_5s.pack(_x))
      _x = self
      buff.write(_struct_2B.pack(_x.usePPP, _x.aopCfg))
      _x = self.reserved5
      # - if encoded as a list instead, serialize as bytes instead of string
      if type(_x) in [list, tuple]:
        buff.write(_struct_2B.pack(*_x))
      else:
        buff.write(_struct_2s.pack(_x))
      buff.write(_struct_H.pack(self.aopOrbMaxErr))
      _x = self.reserved6
      # - if encoded as a list instead, serialize as bytes instead of string
      if type(_x) in [list, tuple]:
        buff.write(_struct_7B.pack(*_x))
      else:
        buff.write(_struct_7s.pack(_x))
      buff.write(_struct_B.pack(self.useAdr))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      end = 0
      _x = self
      start = end
      end += 8
      (_x.version, _x.mask1, _x.mask2,) = _struct_2HI.unpack(str[start:end])
      start = end
      end += 2
      self.reserved1 = str[start:end]
      _x = self
      start = end
      end += 5
      (_x.minSVs, _x.maxSVs, _x.minCNO, _x.reserved2, _x.iniFix3D,) = _struct_5B.unpack(str[start:end])
      start = end
      end += 2
      self.reserved3 = str[start:end]
      _x = self
      start = end
      end += 4
      (_x.ackAiding, _x.wknRollover, _x.sigAttenCompMode,) = _struct_BHB.unpack(str[start:end])
      start = end
      end += 5
      self.reserved4 = str[start:end]
      _x = self
      start = end
      end += 2
      (_x.usePPP, _x.aopCfg,) = _struct_2B.unpack(str[start:end])
      start = end
      end += 2
      self.reserved5 = str[start:end]
      start = end
      end += 2
      (self.aopOrbMaxErr,) = _struct_H.unpack(str[start:end])
      start = end
      end += 7
      self.reserved6 = str[start:end]
      start = end
      end += 1
      (self.useAdr,) = _struct_B.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_B = struct.Struct("<B")
_struct_7B = struct.Struct("<7B")
_struct_2s = struct.Struct("<2s")
_struct_H = struct.Struct("<H")
_struct_BHB = struct.Struct("<BHB")
_struct_5s = struct.Struct("<5s")
_struct_2HI = struct.Struct("<2HI")
_struct_7s = struct.Struct("<7s")
_struct_2B = struct.Struct("<2B")
_struct_5B = struct.Struct("<5B")
