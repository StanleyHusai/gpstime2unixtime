// Generated by gencpp from file ublox_msgs/NavPVT.msg
// DO NOT EDIT!


#ifndef UBLOX_MSGS_MESSAGE_NAVPVT_H
#define UBLOX_MSGS_MESSAGE_NAVPVT_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace ublox_msgs
{
template <class ContainerAllocator>
struct NavPVT_
{
  typedef NavPVT_<ContainerAllocator> Type;

  NavPVT_()
    : iTOW(0)
    , year(0)
    , month(0)
    , day(0)
    , hour(0)
    , min(0)
    , sec(0)
    , valid(0)
    , tAcc(0)
    , nano(0)
    , fixType(0)
    , flags(0)
    , flags2(0)
    , numSV(0)
    , lon(0)
    , lat(0)
    , height(0)
    , hMSL(0)
    , hAcc(0)
    , vAcc(0)
    , velN(0)
    , velE(0)
    , velD(0)
    , gSpeed(0)
    , heading(0)
    , sAcc(0)
    , headAcc(0)
    , pDOP(0)
    , reserved1()
    , headVeh(0)
    , magDec(0)
    , magAcc(0)  {
      reserved1.assign(0);
  }
  NavPVT_(const ContainerAllocator& _alloc)
    : iTOW(0)
    , year(0)
    , month(0)
    , day(0)
    , hour(0)
    , min(0)
    , sec(0)
    , valid(0)
    , tAcc(0)
    , nano(0)
    , fixType(0)
    , flags(0)
    , flags2(0)
    , numSV(0)
    , lon(0)
    , lat(0)
    , height(0)
    , hMSL(0)
    , hAcc(0)
    , vAcc(0)
    , velN(0)
    , velE(0)
    , velD(0)
    , gSpeed(0)
    , heading(0)
    , sAcc(0)
    , headAcc(0)
    , pDOP(0)
    , reserved1()
    , headVeh(0)
    , magDec(0)
    , magAcc(0)  {
  (void)_alloc;
      reserved1.assign(0);
  }



   typedef uint32_t _iTOW_type;
  _iTOW_type iTOW;

   typedef uint16_t _year_type;
  _year_type year;

   typedef uint8_t _month_type;
  _month_type month;

   typedef uint8_t _day_type;
  _day_type day;

   typedef uint8_t _hour_type;
  _hour_type hour;

   typedef uint8_t _min_type;
  _min_type min;

   typedef uint8_t _sec_type;
  _sec_type sec;

   typedef uint8_t _valid_type;
  _valid_type valid;

   typedef uint32_t _tAcc_type;
  _tAcc_type tAcc;

   typedef int32_t _nano_type;
  _nano_type nano;

   typedef uint8_t _fixType_type;
  _fixType_type fixType;

   typedef uint8_t _flags_type;
  _flags_type flags;

   typedef uint8_t _flags2_type;
  _flags2_type flags2;

   typedef uint8_t _numSV_type;
  _numSV_type numSV;

   typedef int32_t _lon_type;
  _lon_type lon;

   typedef int32_t _lat_type;
  _lat_type lat;

   typedef int32_t _height_type;
  _height_type height;

   typedef int32_t _hMSL_type;
  _hMSL_type hMSL;

   typedef uint32_t _hAcc_type;
  _hAcc_type hAcc;

   typedef uint32_t _vAcc_type;
  _vAcc_type vAcc;

   typedef int32_t _velN_type;
  _velN_type velN;

   typedef int32_t _velE_type;
  _velE_type velE;

   typedef int32_t _velD_type;
  _velD_type velD;

   typedef int32_t _gSpeed_type;
  _gSpeed_type gSpeed;

   typedef int32_t _heading_type;
  _heading_type heading;

   typedef uint32_t _sAcc_type;
  _sAcc_type sAcc;

   typedef uint32_t _headAcc_type;
  _headAcc_type headAcc;

   typedef uint16_t _pDOP_type;
  _pDOP_type pDOP;

   typedef boost::array<uint8_t, 6>  _reserved1_type;
  _reserved1_type reserved1;

   typedef int32_t _headVeh_type;
  _headVeh_type headVeh;

   typedef int16_t _magDec_type;
  _magDec_type magDec;

   typedef uint16_t _magAcc_type;
  _magAcc_type magAcc;


    enum { CLASS_ID = 1u };
     enum { MESSAGE_ID = 7u };
     enum { VALID_DATE = 1u };
     enum { VALID_TIME = 2u };
     enum { VALID_FULLY_RESOLVED = 4u };
     enum { VALID_MAG = 8u };
     enum { FIX_TYPE_NO_FIX = 0u };
     enum { FIX_TYPE_DEAD_RECKONING_ONLY = 1u };
     enum { FIX_TYPE_2D = 2u };
     enum { FIX_TYPE_3D = 3u };
     enum { FIX_TYPE_GNSS_DEAD_RECKONING_COMBINED = 4u };
     enum { FIX_TYPE_TIME_ONLY = 5u };
     enum { FLAGS_GNSS_FIX_OK = 1u };
     enum { FLAGS_DIFF_SOLN = 2u };
     enum { FLAGS_PSM_MASK = 28u };
     enum { PSM_OFF = 0u };
     enum { PSM_ENABLED = 4u };
     enum { PSM_ACQUIRED = 8u };
     enum { PSM_TRACKING = 12u };
     enum { PSM_POWER_OPTIMIZED_TRACKING = 16u };
     enum { PSM_INACTIVE = 20u };
     enum { FLAGS_HEAD_VEH_VALID = 32u };
     enum { FLAGS_CARRIER_PHASE_MASK = 192u };
     enum { CARRIER_PHASE_NO_SOLUTION = 0u };
     enum { CARRIER_PHASE_FLOAT = 64u };
     enum { CARRIER_PHASE_FIXED = 128u };
     enum { FLAGS2_CONFIRMED_AVAILABLE = 32u };
     enum { FLAGS2_CONFIRMED_DATE = 64u };
     enum { FLAGS2_CONFIRMED_TIME = 128u };
 

  typedef boost::shared_ptr< ::ublox_msgs::NavPVT_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ublox_msgs::NavPVT_<ContainerAllocator> const> ConstPtr;

}; // struct NavPVT_

typedef ::ublox_msgs::NavPVT_<std::allocator<void> > NavPVT;

typedef boost::shared_ptr< ::ublox_msgs::NavPVT > NavPVTPtr;
typedef boost::shared_ptr< ::ublox_msgs::NavPVT const> NavPVTConstPtr;

// constants requiring out of line definition

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ublox_msgs::NavPVT_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ublox_msgs::NavPVT_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace ublox_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'sensor_msgs': ['/opt/ros/jade/share/sensor_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/jade/share/geometry_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/jade/share/std_msgs/cmake/../msg'], 'ublox_msgs': ['/home/husai/catkin_ws/src/ublox/ublox_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::ublox_msgs::NavPVT_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ublox_msgs::NavPVT_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ublox_msgs::NavPVT_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ublox_msgs::NavPVT_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ublox_msgs::NavPVT_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ublox_msgs::NavPVT_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ublox_msgs::NavPVT_<ContainerAllocator> >
{
  static const char* value()
  {
    return "10f57b0db1fa3679c06567492fa4e5f2";
  }

  static const char* value(const ::ublox_msgs::NavPVT_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x10f57b0db1fa3679ULL;
  static const uint64_t static_value2 = 0xc06567492fa4e5f2ULL;
};

template<class ContainerAllocator>
struct DataType< ::ublox_msgs::NavPVT_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ublox_msgs/NavPVT";
  }

  static const char* value(const ::ublox_msgs::NavPVT_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ublox_msgs::NavPVT_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# NAV-PVT (0x01 0x07)\n\
# Navigation Position Velocity Time Solution\n\
#\n\
# Note that during a leap second there may be more (or less) than 60 seconds in\n\
# a minute; see the description of leap seconds for details.\n\
#\n\
# This message combines Position, velocity and time solution in LLH, \n\
# including accuracy figures\n\
#\n\
# WARNING: For firmware version 7, this message is a different length.\n\
#\n\
\n\
uint8 CLASS_ID = 1\n\
uint8 MESSAGE_ID = 7\n\
\n\
uint32 iTOW             # GPS Millisecond time of week [ms]\n\
uint16 year             # Year (UTC)\n\
uint8 month             # Month, range 1..12 (UTC)\n\
uint8 day               # Day of month, range 1..31 (UTC)\n\
uint8 hour              # Hour of day, range 0..23 (UTC)\n\
uint8 min               # Minute of hour, range 0..59 (UTC)\n\
uint8 sec               # Seconds of minute, range 0..60 (UTC)\n\
\n\
uint8 valid             # Validity flags\n\
uint8 VALID_DATE = 1            # Valid UTC Date\n\
uint8 VALID_TIME = 2            # Valid \n\
uint8 VALID_FULLY_RESOLVED = 4  # UTC time of day has been fully resolved \n\
                                # (no seconds uncertainty)\n\
uint8 VALID_MAG = 8             # Valid Magnetic Declination\n\
\n\
uint32 tAcc             # time accuracy estimate [ns] (UTC)\n\
int32 nano              # fraction of a second [ns], range -1e9 .. 1e9 (UTC)\n\
\n\
uint8 fixType           # GNSS fix Type, range 0..5\n\
uint8 FIX_TYPE_NO_FIX = 0\n\
uint8 FIX_TYPE_DEAD_RECKONING_ONLY = 1\n\
uint8 FIX_TYPE_2D = 2                           # Signal from only 3 SVs, \n\
                                                # constant altitude assumed\n\
uint8 FIX_TYPE_3D = 3\n\
uint8 FIX_TYPE_GNSS_DEAD_RECKONING_COMBINED = 4 # GNSS + Dead reckoning\n\
uint8 FIX_TYPE_TIME_ONLY = 5                    # Time only fix (High precision \n\
                                                # devices)\n\
\n\
uint8 flags             # Fix Status Flags\n\
uint8 FLAGS_GNSS_FIX_OK = 1          # i.e. within DOP & accuracy masks\n\
uint8 FLAGS_DIFF_SOLN = 2            # DGPS used\n\
uint8 FLAGS_PSM_MASK = 28            # Power Save Mode\n\
uint8 PSM_OFF = 0                       # PSM is off\n\
uint8 PSM_ENABLED = 4                   # Enabled (state before acquisition)\n\
uint8 PSM_ACQUIRED = 8                  # Acquisition\n\
uint8 PSM_TRACKING = 12                 # Tracking\n\
uint8 PSM_POWER_OPTIMIZED_TRACKING = 16 # Power Optimized Tracking\n\
uint8 PSM_INACTIVE = 20                 # Inactive\n\
uint8 FLAGS_HEAD_VEH_VALID = 32         # heading of vehicle is valid\n\
uint8 FLAGS_CARRIER_PHASE_MASK = 192 # Carrier Phase Range Solution Status     \n\
uint8 CARRIER_PHASE_NO_SOLUTION = 0     # no carrier phase range solution\n\
uint8 CARRIER_PHASE_FLOAT = 64          # carrier phase float solution (no fixed \n\
                                        # integer measurements have been used to \n\
                                        # calculate the solution)\n\
uint8 CARRIER_PHASE_FIXED = 128         # fixed solution (>=1 fixed integer \n\
                                        # carrier phase range measurements have \n\
                                        # been used to calculate  the solution)\n\
\n\
uint8 flags2            # Additional Flags\n\
uint8 FLAGS2_CONFIRMED_AVAILABLE = 32   # information about UTC Date and Time of \n\
                                        # Day validity confirmation is available\n\
uint8 FLAGS2_CONFIRMED_DATE = 64        # UTC Date validity could be confirmed\n\
uint8 FLAGS2_CONFIRMED_TIME = 128       # UTC Time of Day could be confirmed\n\
\n\
uint8 numSV             # Number of SVs used in Nav Solution\n\
int32 lon               # Longitude [deg / 1e-7]\n\
int32 lat               # Latitude [deg / 1e-7]\n\
int32 height            # Height above Ellipsoid [mm]\n\
int32 hMSL              # Height above mean sea level [mm]\n\
uint32 hAcc             # Horizontal Accuracy Estimate [mm]\n\
uint32 vAcc             # Vertical Accuracy Estimate [mm]\n\
\n\
int32 velN              # NED north velocity [mm/s]\n\
int32 velE              # NED east velocity [mm/s]\n\
int32 velD              # NED down velocity [mm/s]\n\
int32 gSpeed            # Ground Speed (2-D) [mm/s]\n\
int32 heading           # Heading of motion 2-D [deg / 1e-5]\n\
uint32 sAcc             # Speed Accuracy Estimate [mm/s]\n\
uint32 headAcc          # Heading Accuracy Estimate (both motion & vehicle) \n\
                        # [deg / 1e-5]\n\
\n\
uint16 pDOP             # Position DOP [1 / 0.01]\n\
uint8[6] reserved1      # Reserved\n\
\n\
int32 headVeh           # Heading of vehicle (2-D) [deg / 1e-5]\n\
int16 magDec            # Magnetic declination [deg / 1e-2]\n\
uint16 magAcc           # Magnetic declination accuracy [deg / 1e-2]\n\
";
  }

  static const char* value(const ::ublox_msgs::NavPVT_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ublox_msgs::NavPVT_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.iTOW);
      stream.next(m.year);
      stream.next(m.month);
      stream.next(m.day);
      stream.next(m.hour);
      stream.next(m.min);
      stream.next(m.sec);
      stream.next(m.valid);
      stream.next(m.tAcc);
      stream.next(m.nano);
      stream.next(m.fixType);
      stream.next(m.flags);
      stream.next(m.flags2);
      stream.next(m.numSV);
      stream.next(m.lon);
      stream.next(m.lat);
      stream.next(m.height);
      stream.next(m.hMSL);
      stream.next(m.hAcc);
      stream.next(m.vAcc);
      stream.next(m.velN);
      stream.next(m.velE);
      stream.next(m.velD);
      stream.next(m.gSpeed);
      stream.next(m.heading);
      stream.next(m.sAcc);
      stream.next(m.headAcc);
      stream.next(m.pDOP);
      stream.next(m.reserved1);
      stream.next(m.headVeh);
      stream.next(m.magDec);
      stream.next(m.magAcc);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct NavPVT_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ublox_msgs::NavPVT_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::ublox_msgs::NavPVT_<ContainerAllocator>& v)
  {
    s << indent << "iTOW: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.iTOW);
    s << indent << "year: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.year);
    s << indent << "month: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.month);
    s << indent << "day: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.day);
    s << indent << "hour: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.hour);
    s << indent << "min: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.min);
    s << indent << "sec: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.sec);
    s << indent << "valid: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.valid);
    s << indent << "tAcc: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.tAcc);
    s << indent << "nano: ";
    Printer<int32_t>::stream(s, indent + "  ", v.nano);
    s << indent << "fixType: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.fixType);
    s << indent << "flags: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.flags);
    s << indent << "flags2: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.flags2);
    s << indent << "numSV: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.numSV);
    s << indent << "lon: ";
    Printer<int32_t>::stream(s, indent + "  ", v.lon);
    s << indent << "lat: ";
    Printer<int32_t>::stream(s, indent + "  ", v.lat);
    s << indent << "height: ";
    Printer<int32_t>::stream(s, indent + "  ", v.height);
    s << indent << "hMSL: ";
    Printer<int32_t>::stream(s, indent + "  ", v.hMSL);
    s << indent << "hAcc: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.hAcc);
    s << indent << "vAcc: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.vAcc);
    s << indent << "velN: ";
    Printer<int32_t>::stream(s, indent + "  ", v.velN);
    s << indent << "velE: ";
    Printer<int32_t>::stream(s, indent + "  ", v.velE);
    s << indent << "velD: ";
    Printer<int32_t>::stream(s, indent + "  ", v.velD);
    s << indent << "gSpeed: ";
    Printer<int32_t>::stream(s, indent + "  ", v.gSpeed);
    s << indent << "heading: ";
    Printer<int32_t>::stream(s, indent + "  ", v.heading);
    s << indent << "sAcc: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.sAcc);
    s << indent << "headAcc: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.headAcc);
    s << indent << "pDOP: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.pDOP);
    s << indent << "reserved1[]" << std::endl;
    for (size_t i = 0; i < v.reserved1.size(); ++i)
    {
      s << indent << "  reserved1[" << i << "]: ";
      Printer<uint8_t>::stream(s, indent + "  ", v.reserved1[i]);
    }
    s << indent << "headVeh: ";
    Printer<int32_t>::stream(s, indent + "  ", v.headVeh);
    s << indent << "magDec: ";
    Printer<int16_t>::stream(s, indent + "  ", v.magDec);
    s << indent << "magAcc: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.magAcc);
  }
};

} // namespace message_operations
} // namespace ros

#endif // UBLOX_MSGS_MESSAGE_NAVPVT_H
