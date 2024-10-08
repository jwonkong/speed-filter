// Generated by gencpp from file brain_msgs/Wgs84.msg
// DO NOT EDIT!


#ifndef BRAIN_MSGS_MESSAGE_WGS84_H
#define BRAIN_MSGS_MESSAGE_WGS84_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace brain_msgs
{
template <class ContainerAllocator>
struct Wgs84_
{
  typedef Wgs84_<ContainerAllocator> Type;

  Wgs84_()
    : latitude(0.0)
    , longitude(0.0)
    , height(0.0)
    , heading(0.0)
    , status(0)  {
    }
  Wgs84_(const ContainerAllocator& _alloc)
    : latitude(0.0)
    , longitude(0.0)
    , height(0.0)
    , heading(0.0)
    , status(0)  {
  (void)_alloc;
    }



   typedef double _latitude_type;
  _latitude_type latitude;

   typedef double _longitude_type;
  _longitude_type longitude;

   typedef float _height_type;
  _height_type height;

   typedef float _heading_type;
  _heading_type heading;

   typedef uint8_t _status_type;
  _status_type status;



// reducing the odds to have name collisions with Windows.h 
#if defined(_WIN32) && defined(valid)
  #undef valid
#endif
#if defined(_WIN32) && defined(invalid)
  #undef invalid
#endif

  enum {
    valid = 0u,
    invalid = 1u,
  };


  typedef boost::shared_ptr< ::brain_msgs::Wgs84_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::brain_msgs::Wgs84_<ContainerAllocator> const> ConstPtr;

}; // struct Wgs84_

typedef ::brain_msgs::Wgs84_<std::allocator<void> > Wgs84;

typedef boost::shared_ptr< ::brain_msgs::Wgs84 > Wgs84Ptr;
typedef boost::shared_ptr< ::brain_msgs::Wgs84 const> Wgs84ConstPtr;

// constants requiring out of line definition

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::brain_msgs::Wgs84_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::brain_msgs::Wgs84_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::brain_msgs::Wgs84_<ContainerAllocator1> & lhs, const ::brain_msgs::Wgs84_<ContainerAllocator2> & rhs)
{
  return lhs.latitude == rhs.latitude &&
    lhs.longitude == rhs.longitude &&
    lhs.height == rhs.height &&
    lhs.heading == rhs.heading &&
    lhs.status == rhs.status;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::brain_msgs::Wgs84_<ContainerAllocator1> & lhs, const ::brain_msgs::Wgs84_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace brain_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::brain_msgs::Wgs84_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::brain_msgs::Wgs84_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::brain_msgs::Wgs84_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::brain_msgs::Wgs84_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::brain_msgs::Wgs84_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::brain_msgs::Wgs84_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::brain_msgs::Wgs84_<ContainerAllocator> >
{
  static const char* value()
  {
    return "155a87cab70ded072eeb22bb49579b4c";
  }

  static const char* value(const ::brain_msgs::Wgs84_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x155a87cab70ded07ULL;
  static const uint64_t static_value2 = 0x2eeb22bb49579b4cULL;
};

template<class ContainerAllocator>
struct DataType< ::brain_msgs::Wgs84_<ContainerAllocator> >
{
  static const char* value()
  {
    return "brain_msgs/Wgs84";
  }

  static const char* value(const ::brain_msgs::Wgs84_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::brain_msgs::Wgs84_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float64 latitude\n"
"float64 longitude\n"
"float32 height\n"
"float32 heading\n"
"uint8 status\n"
"uint8 valid=0\n"
"uint8 invalid=1\n"
;
  }

  static const char* value(const ::brain_msgs::Wgs84_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::brain_msgs::Wgs84_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.latitude);
      stream.next(m.longitude);
      stream.next(m.height);
      stream.next(m.heading);
      stream.next(m.status);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Wgs84_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::brain_msgs::Wgs84_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::brain_msgs::Wgs84_<ContainerAllocator>& v)
  {
    s << indent << "latitude: ";
    Printer<double>::stream(s, indent + "  ", v.latitude);
    s << indent << "longitude: ";
    Printer<double>::stream(s, indent + "  ", v.longitude);
    s << indent << "height: ";
    Printer<float>::stream(s, indent + "  ", v.height);
    s << indent << "heading: ";
    Printer<float>::stream(s, indent + "  ", v.heading);
    s << indent << "status: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.status);
  }
};

} // namespace message_operations
} // namespace ros

#endif // BRAIN_MSGS_MESSAGE_WGS84_H
