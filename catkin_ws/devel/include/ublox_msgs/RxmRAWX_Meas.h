// Generated by gencpp from file ublox_msgs/RxmRAWX_Meas.msg
// DO NOT EDIT!


#ifndef UBLOX_MSGS_MESSAGE_RXMRAWX_MEAS_H
#define UBLOX_MSGS_MESSAGE_RXMRAWX_MEAS_H


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
struct RxmRAWX_Meas_
{
  typedef RxmRAWX_Meas_<ContainerAllocator> Type;

  RxmRAWX_Meas_()
    : prMes(0.0)
    , cpMes(0.0)
    , doMes(0.0)
    , gnssId(0)
    , svId(0)
    , reserved0(0)
    , freqId(0)
    , locktime(0)
    , cno(0)
    , prStdev(0)
    , cpStdev(0)
    , doStdev(0)
    , trkStat(0)
    , reserved1(0)  {
    }
  RxmRAWX_Meas_(const ContainerAllocator& _alloc)
    : prMes(0.0)
    , cpMes(0.0)
    , doMes(0.0)
    , gnssId(0)
    , svId(0)
    , reserved0(0)
    , freqId(0)
    , locktime(0)
    , cno(0)
    , prStdev(0)
    , cpStdev(0)
    , doStdev(0)
    , trkStat(0)
    , reserved1(0)  {
  (void)_alloc;
    }



   typedef double _prMes_type;
  _prMes_type prMes;

   typedef double _cpMes_type;
  _cpMes_type cpMes;

   typedef float _doMes_type;
  _doMes_type doMes;

   typedef uint8_t _gnssId_type;
  _gnssId_type gnssId;

   typedef uint8_t _svId_type;
  _svId_type svId;

   typedef uint8_t _reserved0_type;
  _reserved0_type reserved0;

   typedef uint8_t _freqId_type;
  _freqId_type freqId;

   typedef uint16_t _locktime_type;
  _locktime_type locktime;

   typedef int8_t _cno_type;
  _cno_type cno;

   typedef uint8_t _prStdev_type;
  _prStdev_type prStdev;

   typedef uint8_t _cpStdev_type;
  _cpStdev_type cpStdev;

   typedef uint8_t _doStdev_type;
  _doStdev_type doStdev;

   typedef uint8_t _trkStat_type;
  _trkStat_type trkStat;

   typedef uint8_t _reserved1_type;
  _reserved1_type reserved1;


    enum { TRK_STAT_PR_VALID = 1u };
     enum { TRK_STAT_CP_VALID = 2u };
     enum { TRK_STAT_HALF_CYC = 4u };
     enum { TRK_STAT_SUB_HALF_CYC = 8u };
 

  typedef boost::shared_ptr< ::ublox_msgs::RxmRAWX_Meas_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ublox_msgs::RxmRAWX_Meas_<ContainerAllocator> const> ConstPtr;

}; // struct RxmRAWX_Meas_

typedef ::ublox_msgs::RxmRAWX_Meas_<std::allocator<void> > RxmRAWX_Meas;

typedef boost::shared_ptr< ::ublox_msgs::RxmRAWX_Meas > RxmRAWX_MeasPtr;
typedef boost::shared_ptr< ::ublox_msgs::RxmRAWX_Meas const> RxmRAWX_MeasConstPtr;

// constants requiring out of line definition

   

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ublox_msgs::RxmRAWX_Meas_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ublox_msgs::RxmRAWX_Meas_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::ublox_msgs::RxmRAWX_Meas_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ublox_msgs::RxmRAWX_Meas_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ublox_msgs::RxmRAWX_Meas_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ublox_msgs::RxmRAWX_Meas_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ublox_msgs::RxmRAWX_Meas_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ublox_msgs::RxmRAWX_Meas_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ublox_msgs::RxmRAWX_Meas_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d6a580262875bf83a377ba14dcdd659f";
  }

  static const char* value(const ::ublox_msgs::RxmRAWX_Meas_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd6a580262875bf83ULL;
  static const uint64_t static_value2 = 0xa377ba14dcdd659fULL;
};

template<class ContainerAllocator>
struct DataType< ::ublox_msgs::RxmRAWX_Meas_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ublox_msgs/RxmRAWX_Meas";
  }

  static const char* value(const ::ublox_msgs::RxmRAWX_Meas_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ublox_msgs::RxmRAWX_Meas_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# see message RxmRAWX\n\
#\n\
\n\
float64 prMes             # Pseudorange measurement [m]. GLONASS inter frequency\n\
                          # channel delays are compensated with an internal\n\
                          # calibration table.\n\
float64 cpMes             # Carrier phase measurement [L1 cycles]. The carrier\n\
                          # phase initial ambiguity is initialized using an\n\
                          # approximate value to make the magnitude of\n\
                          # the phase close to the pseudorange\n\
                          # measurement. Clock resets are applied to both\n\
                          # phase and code measurements in accordance\n\
                          # with the RINEX specification.\n\
float32 doMes             # Doppler measurement [Hz] (positive sign for\n\
                          # approaching satellites)\n\
uint8 gnssId              # GNSS identifier (see CfgGNSS for constants)\n\
\n\
uint8 svId                # Satellite identifier (see Satellite Numbering)\n\
\n\
uint8 reserved0           # Reserved\n\
\n\
uint8 freqId              # Only used for GLONASS: This is the frequency\n\
                          # slot + 7 (range from 0 to 13)\n\
uint16 locktime           # Carrier phase locktime counter [ms] \n\
                          # (maximum 64500 ms)\n\
int8 cno                  # Carrier-to-noise density ratio (signal strength) \n\
                          # [dB-Hz]\n\
uint8 prStdev             # Estimated pseudorange measurement standard\n\
                          # deviation [m / 0.01*2^n]\n\
uint8 cpStdev             # Estimated carrier phase measurement standard\n\
                          # deviation (note a raw value of 0x0F indicates the\n\
                          # value is invalid) [cycles / 0.004]\n\
uint8 doStdev             # Estimated Doppler measurement standard deviation \n\
                          # [Hz / 0.002*2^n]\n\
\n\
uint8 trkStat             # Tracking status bitfield\n\
uint8 TRK_STAT_PR_VALID = 1       # Pseudorange valid\n\
uint8 TRK_STAT_CP_VALID = 2       # Carrier phase valid\n\
uint8 TRK_STAT_HALF_CYC = 4       # Half cycle valid\n\
uint8 TRK_STAT_SUB_HALF_CYC = 8   # Half cycle subtracted from phase\n\
\n\
uint8 reserved1           # Reserved\n\
";
  }

  static const char* value(const ::ublox_msgs::RxmRAWX_Meas_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ublox_msgs::RxmRAWX_Meas_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.prMes);
      stream.next(m.cpMes);
      stream.next(m.doMes);
      stream.next(m.gnssId);
      stream.next(m.svId);
      stream.next(m.reserved0);
      stream.next(m.freqId);
      stream.next(m.locktime);
      stream.next(m.cno);
      stream.next(m.prStdev);
      stream.next(m.cpStdev);
      stream.next(m.doStdev);
      stream.next(m.trkStat);
      stream.next(m.reserved1);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct RxmRAWX_Meas_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ublox_msgs::RxmRAWX_Meas_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::ublox_msgs::RxmRAWX_Meas_<ContainerAllocator>& v)
  {
    s << indent << "prMes: ";
    Printer<double>::stream(s, indent + "  ", v.prMes);
    s << indent << "cpMes: ";
    Printer<double>::stream(s, indent + "  ", v.cpMes);
    s << indent << "doMes: ";
    Printer<float>::stream(s, indent + "  ", v.doMes);
    s << indent << "gnssId: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.gnssId);
    s << indent << "svId: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.svId);
    s << indent << "reserved0: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.reserved0);
    s << indent << "freqId: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.freqId);
    s << indent << "locktime: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.locktime);
    s << indent << "cno: ";
    Printer<int8_t>::stream(s, indent + "  ", v.cno);
    s << indent << "prStdev: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.prStdev);
    s << indent << "cpStdev: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.cpStdev);
    s << indent << "doStdev: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.doStdev);
    s << indent << "trkStat: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.trkStat);
    s << indent << "reserved1: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.reserved1);
  }
};

} // namespace message_operations
} // namespace ros

#endif // UBLOX_MSGS_MESSAGE_RXMRAWX_MEAS_H