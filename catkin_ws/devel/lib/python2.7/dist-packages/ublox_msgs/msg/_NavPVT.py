# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from ublox_msgs/NavPVT.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class NavPVT(genpy.Message):
  _md5sum = "10f57b0db1fa3679c06567492fa4e5f2"
  _type = "ublox_msgs/NavPVT"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """# NAV-PVT (0x01 0x07)
# Navigation Position Velocity Time Solution
#
# Note that during a leap second there may be more (or less) than 60 seconds in
# a minute; see the description of leap seconds for details.
#
# This message combines Position, velocity and time solution in LLH, 
# including accuracy figures
#
# WARNING: For firmware version 7, this message is a different length.
#

uint8 CLASS_ID = 1
uint8 MESSAGE_ID = 7

uint32 iTOW             # GPS Millisecond time of week [ms]
uint16 year             # Year (UTC)
uint8 month             # Month, range 1..12 (UTC)
uint8 day               # Day of month, range 1..31 (UTC)
uint8 hour              # Hour of day, range 0..23 (UTC)
uint8 min               # Minute of hour, range 0..59 (UTC)
uint8 sec               # Seconds of minute, range 0..60 (UTC)

uint8 valid             # Validity flags
uint8 VALID_DATE = 1            # Valid UTC Date
uint8 VALID_TIME = 2            # Valid 
uint8 VALID_FULLY_RESOLVED = 4  # UTC time of day has been fully resolved 
                                # (no seconds uncertainty)
uint8 VALID_MAG = 8             # Valid Magnetic Declination

uint32 tAcc             # time accuracy estimate [ns] (UTC)
int32 nano              # fraction of a second [ns], range -1e9 .. 1e9 (UTC)

uint8 fixType           # GNSS fix Type, range 0..5
uint8 FIX_TYPE_NO_FIX = 0
uint8 FIX_TYPE_DEAD_RECKONING_ONLY = 1
uint8 FIX_TYPE_2D = 2                           # Signal from only 3 SVs, 
                                                # constant altitude assumed
uint8 FIX_TYPE_3D = 3
uint8 FIX_TYPE_GNSS_DEAD_RECKONING_COMBINED = 4 # GNSS + Dead reckoning
uint8 FIX_TYPE_TIME_ONLY = 5                    # Time only fix (High precision 
                                                # devices)

uint8 flags             # Fix Status Flags
uint8 FLAGS_GNSS_FIX_OK = 1          # i.e. within DOP & accuracy masks
uint8 FLAGS_DIFF_SOLN = 2            # DGPS used
uint8 FLAGS_PSM_MASK = 28            # Power Save Mode
uint8 PSM_OFF = 0                       # PSM is off
uint8 PSM_ENABLED = 4                   # Enabled (state before acquisition)
uint8 PSM_ACQUIRED = 8                  # Acquisition
uint8 PSM_TRACKING = 12                 # Tracking
uint8 PSM_POWER_OPTIMIZED_TRACKING = 16 # Power Optimized Tracking
uint8 PSM_INACTIVE = 20                 # Inactive
uint8 FLAGS_HEAD_VEH_VALID = 32         # heading of vehicle is valid
uint8 FLAGS_CARRIER_PHASE_MASK = 192 # Carrier Phase Range Solution Status     
uint8 CARRIER_PHASE_NO_SOLUTION = 0     # no carrier phase range solution
uint8 CARRIER_PHASE_FLOAT = 64          # carrier phase float solution (no fixed 
                                        # integer measurements have been used to 
                                        # calculate the solution)
uint8 CARRIER_PHASE_FIXED = 128         # fixed solution (>=1 fixed integer 
                                        # carrier phase range measurements have 
                                        # been used to calculate  the solution)

uint8 flags2            # Additional Flags
uint8 FLAGS2_CONFIRMED_AVAILABLE = 32   # information about UTC Date and Time of 
                                        # Day validity confirmation is available
uint8 FLAGS2_CONFIRMED_DATE = 64        # UTC Date validity could be confirmed
uint8 FLAGS2_CONFIRMED_TIME = 128       # UTC Time of Day could be confirmed

uint8 numSV             # Number of SVs used in Nav Solution
int32 lon               # Longitude [deg / 1e-7]
int32 lat               # Latitude [deg / 1e-7]
int32 height            # Height above Ellipsoid [mm]
int32 hMSL              # Height above mean sea level [mm]
uint32 hAcc             # Horizontal Accuracy Estimate [mm]
uint32 vAcc             # Vertical Accuracy Estimate [mm]

int32 velN              # NED north velocity [mm/s]
int32 velE              # NED east velocity [mm/s]
int32 velD              # NED down velocity [mm/s]
int32 gSpeed            # Ground Speed (2-D) [mm/s]
int32 heading           # Heading of motion 2-D [deg / 1e-5]
uint32 sAcc             # Speed Accuracy Estimate [mm/s]
uint32 headAcc          # Heading Accuracy Estimate (both motion & vehicle) 
                        # [deg / 1e-5]

uint16 pDOP             # Position DOP [1 / 0.01]
uint8[6] reserved1      # Reserved

int32 headVeh           # Heading of vehicle (2-D) [deg / 1e-5]
int16 magDec            # Magnetic declination [deg / 1e-2]
uint16 magAcc           # Magnetic declination accuracy [deg / 1e-2]
"""
  # Pseudo-constants
  CLASS_ID = 1
  MESSAGE_ID = 7
  VALID_DATE = 1
  VALID_TIME = 2
  VALID_FULLY_RESOLVED = 4
  VALID_MAG = 8
  FIX_TYPE_NO_FIX = 0
  FIX_TYPE_DEAD_RECKONING_ONLY = 1
  FIX_TYPE_2D = 2
  FIX_TYPE_3D = 3
  FIX_TYPE_GNSS_DEAD_RECKONING_COMBINED = 4
  FIX_TYPE_TIME_ONLY = 5
  FLAGS_GNSS_FIX_OK = 1
  FLAGS_DIFF_SOLN = 2
  FLAGS_PSM_MASK = 28
  PSM_OFF = 0
  PSM_ENABLED = 4
  PSM_ACQUIRED = 8
  PSM_TRACKING = 12
  PSM_POWER_OPTIMIZED_TRACKING = 16
  PSM_INACTIVE = 20
  FLAGS_HEAD_VEH_VALID = 32
  FLAGS_CARRIER_PHASE_MASK = 192
  CARRIER_PHASE_NO_SOLUTION = 0
  CARRIER_PHASE_FLOAT = 64
  CARRIER_PHASE_FIXED = 128
  FLAGS2_CONFIRMED_AVAILABLE = 32
  FLAGS2_CONFIRMED_DATE = 64
  FLAGS2_CONFIRMED_TIME = 128

  __slots__ = ['iTOW','year','month','day','hour','min','sec','valid','tAcc','nano','fixType','flags','flags2','numSV','lon','lat','height','hMSL','hAcc','vAcc','velN','velE','velD','gSpeed','heading','sAcc','headAcc','pDOP','reserved1','headVeh','magDec','magAcc']
  _slot_types = ['uint32','uint16','uint8','uint8','uint8','uint8','uint8','uint8','uint32','int32','uint8','uint8','uint8','uint8','int32','int32','int32','int32','uint32','uint32','int32','int32','int32','int32','int32','uint32','uint32','uint16','uint8[6]','int32','int16','uint16']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       iTOW,year,month,day,hour,min,sec,valid,tAcc,nano,fixType,flags,flags2,numSV,lon,lat,height,hMSL,hAcc,vAcc,velN,velE,velD,gSpeed,heading,sAcc,headAcc,pDOP,reserved1,headVeh,magDec,magAcc

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(NavPVT, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.iTOW is None:
        self.iTOW = 0
      if self.year is None:
        self.year = 0
      if self.month is None:
        self.month = 0
      if self.day is None:
        self.day = 0
      if self.hour is None:
        self.hour = 0
      if self.min is None:
        self.min = 0
      if self.sec is None:
        self.sec = 0
      if self.valid is None:
        self.valid = 0
      if self.tAcc is None:
        self.tAcc = 0
      if self.nano is None:
        self.nano = 0
      if self.fixType is None:
        self.fixType = 0
      if self.flags is None:
        self.flags = 0
      if self.flags2 is None:
        self.flags2 = 0
      if self.numSV is None:
        self.numSV = 0
      if self.lon is None:
        self.lon = 0
      if self.lat is None:
        self.lat = 0
      if self.height is None:
        self.height = 0
      if self.hMSL is None:
        self.hMSL = 0
      if self.hAcc is None:
        self.hAcc = 0
      if self.vAcc is None:
        self.vAcc = 0
      if self.velN is None:
        self.velN = 0
      if self.velE is None:
        self.velE = 0
      if self.velD is None:
        self.velD = 0
      if self.gSpeed is None:
        self.gSpeed = 0
      if self.heading is None:
        self.heading = 0
      if self.sAcc is None:
        self.sAcc = 0
      if self.headAcc is None:
        self.headAcc = 0
      if self.pDOP is None:
        self.pDOP = 0
      if self.reserved1 is None:
        self.reserved1 = chr(0)*6
      if self.headVeh is None:
        self.headVeh = 0
      if self.magDec is None:
        self.magDec = 0
      if self.magAcc is None:
        self.magAcc = 0
    else:
      self.iTOW = 0
      self.year = 0
      self.month = 0
      self.day = 0
      self.hour = 0
      self.min = 0
      self.sec = 0
      self.valid = 0
      self.tAcc = 0
      self.nano = 0
      self.fixType = 0
      self.flags = 0
      self.flags2 = 0
      self.numSV = 0
      self.lon = 0
      self.lat = 0
      self.height = 0
      self.hMSL = 0
      self.hAcc = 0
      self.vAcc = 0
      self.velN = 0
      self.velE = 0
      self.velD = 0
      self.gSpeed = 0
      self.heading = 0
      self.sAcc = 0
      self.headAcc = 0
      self.pDOP = 0
      self.reserved1 = chr(0)*6
      self.headVeh = 0
      self.magDec = 0
      self.magAcc = 0

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
      buff.write(_struct_IH6BIi4B4i2I5i2IH.pack(_x.iTOW, _x.year, _x.month, _x.day, _x.hour, _x.min, _x.sec, _x.valid, _x.tAcc, _x.nano, _x.fixType, _x.flags, _x.flags2, _x.numSV, _x.lon, _x.lat, _x.height, _x.hMSL, _x.hAcc, _x.vAcc, _x.velN, _x.velE, _x.velD, _x.gSpeed, _x.heading, _x.sAcc, _x.headAcc, _x.pDOP))
      _x = self.reserved1
      # - if encoded as a list instead, serialize as bytes instead of string
      if type(_x) in [list, tuple]:
        buff.write(_struct_6B.pack(*_x))
      else:
        buff.write(_struct_6s.pack(_x))
      _x = self
      buff.write(_struct_ihH.pack(_x.headVeh, _x.magDec, _x.magAcc))
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
      end += 78
      (_x.iTOW, _x.year, _x.month, _x.day, _x.hour, _x.min, _x.sec, _x.valid, _x.tAcc, _x.nano, _x.fixType, _x.flags, _x.flags2, _x.numSV, _x.lon, _x.lat, _x.height, _x.hMSL, _x.hAcc, _x.vAcc, _x.velN, _x.velE, _x.velD, _x.gSpeed, _x.heading, _x.sAcc, _x.headAcc, _x.pDOP,) = _struct_IH6BIi4B4i2I5i2IH.unpack(str[start:end])
      start = end
      end += 6
      self.reserved1 = str[start:end]
      _x = self
      start = end
      end += 8
      (_x.headVeh, _x.magDec, _x.magAcc,) = _struct_ihH.unpack(str[start:end])
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
      buff.write(_struct_IH6BIi4B4i2I5i2IH.pack(_x.iTOW, _x.year, _x.month, _x.day, _x.hour, _x.min, _x.sec, _x.valid, _x.tAcc, _x.nano, _x.fixType, _x.flags, _x.flags2, _x.numSV, _x.lon, _x.lat, _x.height, _x.hMSL, _x.hAcc, _x.vAcc, _x.velN, _x.velE, _x.velD, _x.gSpeed, _x.heading, _x.sAcc, _x.headAcc, _x.pDOP))
      _x = self.reserved1
      # - if encoded as a list instead, serialize as bytes instead of string
      if type(_x) in [list, tuple]:
        buff.write(_struct_6B.pack(*_x))
      else:
        buff.write(_struct_6s.pack(_x))
      _x = self
      buff.write(_struct_ihH.pack(_x.headVeh, _x.magDec, _x.magAcc))
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
      end += 78
      (_x.iTOW, _x.year, _x.month, _x.day, _x.hour, _x.min, _x.sec, _x.valid, _x.tAcc, _x.nano, _x.fixType, _x.flags, _x.flags2, _x.numSV, _x.lon, _x.lat, _x.height, _x.hMSL, _x.hAcc, _x.vAcc, _x.velN, _x.velE, _x.velD, _x.gSpeed, _x.heading, _x.sAcc, _x.headAcc, _x.pDOP,) = _struct_IH6BIi4B4i2I5i2IH.unpack(str[start:end])
      start = end
      end += 6
      self.reserved1 = str[start:end]
      _x = self
      start = end
      end += 8
      (_x.headVeh, _x.magDec, _x.magAcc,) = _struct_ihH.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_IH6BIi4B4i2I5i2IH = struct.Struct("<IH6BIi4B4i2I5i2IH")
_struct_6B = struct.Struct("<6B")
_struct_6s = struct.Struct("<6s")
_struct_ihH = struct.Struct("<ihH")
