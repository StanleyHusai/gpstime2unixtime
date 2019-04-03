// Generated by gencpp from file ublox_msgs/NavSBAS_SV.msg
// DO NOT EDIT!


#ifndef UBLOX_MSGS_MESSAGE_NAVSBAS_SV_H
#define UBLOX_MSGS_MESSAGE_NAVSBAS_SV_H


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
struct NavSBAS_SV_
{
  typedef NavSBAS_SV_<ContainerAllocator> Type;

  NavSBAS_SV_()
    : svid(0)
    , flags(0)
    , udre(0)
    , svSys(0)
    , svService(0)
    , reserved1(0)
    , prc(0)
    , reserved2(0)
    , ic(0)  {
    }
  NavSBAS_SV_(const ContainerAllocator& _alloc)
    : svid(0)
    , flags(0)
    , udre(0)
    , svSys(0)
    , svService(0)
    , reserved1(0)
    , prc(0)
    , reserved2(0)
    , ic(0)  {
  (void)_alloc;
    }



   typedef uint8_t _svid_type;
  _svid_type svid;

   typedef uint8_t _flags_type;
  _flags_type flags;

   typedef uint8_t _udre_type;
  _udre_type udre;

   typedef uint8_t _svSys_type;
  _svSys_type svSys;

   typedef uint8_t _svService_type;
  _svService_type svService;

   typedef uint8_t _reserved1_type;
  _reserved1_type reserved1;

   typedef int16_t _prc_type;
  _prc_type prc;

   typedef uint16_t _reserved2_type;
  _reserved2_type reserved2;

   typedef int16_t _ic_type;
  _ic_type ic;




  typedef boost::shared_ptr< ::ublox_msgs::NavSBAS_SV_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ublox_msgs::NavSBAS_SV_<ContainerAllocator> const> ConstPtr;

}; // struct NavSBAS_SV_

typedef ::ublox_msgs::NavSBAS_SV_<std::allocator<void> > NavSBAS_SV;

typedef boost::shared_ptr< ::ublox_msgs::NavSBAS_SV > NavSBAS_SVPtr;
typedef boost::shared_ptr< ::ublox_msgs::NavSBAS_SV const> NavSBAS_SVConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ublox_msgs::NavSBAS_SV_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ublox_msgs::NavSBAS_SV_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::ublox_msgs::NavSBAS_SV_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ublox_msgs::NavSBAS_SV_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ublox_msgs::NavSBAS_SV_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ublox_msgs::NavSBAS_SV_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ublox_msgs::NavSBAS_SV_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ublox_msgs::NavSBAS_SV_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ublox_msgs::NavSBAS_SV_<ContainerAllocator> >
{
  static const char* value()
  {
    return "45259031fe19a4df2f5a4a667356a0bc";
  }

  static const char* value(const ::ublox_msgs::NavSBAS_SV_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x45259031fe19a4dfULL;
  static const uint64_t static_value2 = 0x2f5a4a667356a0bcULL;
};

template<class ContainerAllocator>
struct DataType< ::ublox_msgs::NavSBAS_SV_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ublox_msgs/NavSBAS_SV";
  }

  static const char* value(const ::ublox_msgs::NavSBAS_SV_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ublox_msgs::NavSBAS_SV_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# see message NavSBAS\n\
#\n\
\n\
uint8 svid              # SV Id\n\
uint8 flags             # Flags for this SV\n\
uint8 udre              # Monitoring status\n\
uint8 svSys             # System (WAAS/EGNOS/...), same as SYS\n\
uint8 svService         # Services available, same as SERVICE\n\
uint8 reserved1         # Reserved\n\
int16 prc               # Pseudo Range correction in [cm]\n\
uint16 reserved2        # Reserved\n\
int16 ic                # Ionosphere correction in [cm]\n\
";
  }

  static const char* value(const ::ublox_msgs::NavSBAS_SV_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ublox_msgs::NavSBAS_SV_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.svid);
      stream.next(m.flags);
      stream.next(m.udre);
      stream.next(m.svSys);
      stream.next(m.svService);
      stream.next(m.reserved1);
      stream.next(m.prc);
      stream.next(m.reserved2);
      stream.next(m.ic);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct NavSBAS_SV_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ublox_msgs::NavSBAS_SV_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::ublox_msgs::NavSBAS_SV_<ContainerAllocator>& v)
  {
    s << indent << "svid: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.svid);
    s << indent << "flags: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.flags);
    s << indent << "udre: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.udre);
    s << indent << "svSys: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.svSys);
    s << indent << "svService: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.svService);
    s << indent << "reserved1: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.reserved1);
    s << indent << "prc: ";
    Printer<int16_t>::stream(s, indent + "  ", v.prc);
    s << indent << "reserved2: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.reserved2);
    s << indent << "ic: ";
    Printer<int16_t>::stream(s, indent + "  ", v.ic);
  }
};

} // namespace message_operations
} // namespace ros

#endif // UBLOX_MSGS_MESSAGE_NAVSBAS_SV_H
