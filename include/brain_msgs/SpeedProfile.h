// Generated by gencpp from file brain_msgs/SpeedProfile.msg
// DO NOT EDIT!


#ifndef BRAIN_MSGS_MESSAGE_SPEEDPROFILE_H
#define BRAIN_MSGS_MESSAGE_SPEEDPROFILE_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <brain_msgs/SpeedProfileNode.h>
#include <geometry_msgs/Point.h>

namespace brain_msgs
{
template <class ContainerAllocator>
struct SpeedProfile_
{
  typedef SpeedProfile_<ContainerAllocator> Type;

  SpeedProfile_()
    : header()
    , speed_limit(0.0)
    , points()
    , stop_line()  {
    }
  SpeedProfile_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , speed_limit(0.0)
    , points(_alloc)
    , stop_line(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef float _speed_limit_type;
  _speed_limit_type speed_limit;

   typedef std::vector< ::brain_msgs::SpeedProfileNode_<ContainerAllocator> , typename std::allocator_traits<ContainerAllocator>::template rebind_alloc< ::brain_msgs::SpeedProfileNode_<ContainerAllocator> >> _points_type;
  _points_type points;

   typedef std::vector< ::geometry_msgs::Point_<ContainerAllocator> , typename std::allocator_traits<ContainerAllocator>::template rebind_alloc< ::geometry_msgs::Point_<ContainerAllocator> >> _stop_line_type;
  _stop_line_type stop_line;





  typedef boost::shared_ptr< ::brain_msgs::SpeedProfile_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::brain_msgs::SpeedProfile_<ContainerAllocator> const> ConstPtr;

}; // struct SpeedProfile_

typedef ::brain_msgs::SpeedProfile_<std::allocator<void> > SpeedProfile;

typedef boost::shared_ptr< ::brain_msgs::SpeedProfile > SpeedProfilePtr;
typedef boost::shared_ptr< ::brain_msgs::SpeedProfile const> SpeedProfileConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::brain_msgs::SpeedProfile_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::brain_msgs::SpeedProfile_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::brain_msgs::SpeedProfile_<ContainerAllocator1> & lhs, const ::brain_msgs::SpeedProfile_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.speed_limit == rhs.speed_limit &&
    lhs.points == rhs.points &&
    lhs.stop_line == rhs.stop_line;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::brain_msgs::SpeedProfile_<ContainerAllocator1> & lhs, const ::brain_msgs::SpeedProfile_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace brain_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::brain_msgs::SpeedProfile_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::brain_msgs::SpeedProfile_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::brain_msgs::SpeedProfile_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::brain_msgs::SpeedProfile_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::brain_msgs::SpeedProfile_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::brain_msgs::SpeedProfile_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::brain_msgs::SpeedProfile_<ContainerAllocator> >
{
  static const char* value()
  {
    return "5c3dfecc5d02d8a8eb8c8671fff6dd01";
  }

  static const char* value(const ::brain_msgs::SpeedProfile_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x5c3dfecc5d02d8a8ULL;
  static const uint64_t static_value2 = 0xeb8c8671fff6dd01ULL;
};

template<class ContainerAllocator>
struct DataType< ::brain_msgs::SpeedProfile_<ContainerAllocator> >
{
  static const char* value()
  {
    return "brain_msgs/SpeedProfile";
  }

  static const char* value(const ::brain_msgs::SpeedProfile_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::brain_msgs::SpeedProfile_<ContainerAllocator> >
{
  static const char* value()
  {
    return "std_msgs/Header                header\n"
"float32                        speed_limit\n"
"brain_msgs/SpeedProfileNode[]  points\n"
"geometry_msgs/Point[]          stop_line\n"
"================================================================================\n"
"MSG: std_msgs/Header\n"
"# Standard metadata for higher-level stamped data types.\n"
"# This is generally used to communicate timestamped data \n"
"# in a particular coordinate frame.\n"
"# \n"
"# sequence ID: consecutively increasing ID \n"
"uint32 seq\n"
"#Two-integer timestamp that is expressed as:\n"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
"# time-handling sugar is provided by the client library\n"
"time stamp\n"
"#Frame this data is associated with\n"
"string frame_id\n"
"\n"
"================================================================================\n"
"MSG: brain_msgs/SpeedProfileNode\n"
"float32 distance\n"
"float32 speed\n"
"float32 time\n"
"int8 direction\n"
"================================================================================\n"
"MSG: geometry_msgs/Point\n"
"# This contains the position of a point in free space\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
;
  }

  static const char* value(const ::brain_msgs::SpeedProfile_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::brain_msgs::SpeedProfile_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.speed_limit);
      stream.next(m.points);
      stream.next(m.stop_line);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SpeedProfile_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::brain_msgs::SpeedProfile_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::brain_msgs::SpeedProfile_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "speed_limit: ";
    Printer<float>::stream(s, indent + "  ", v.speed_limit);
    s << indent << "points[]" << std::endl;
    for (size_t i = 0; i < v.points.size(); ++i)
    {
      s << indent << "  points[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::brain_msgs::SpeedProfileNode_<ContainerAllocator> >::stream(s, indent + "    ", v.points[i]);
    }
    s << indent << "stop_line[]" << std::endl;
    for (size_t i = 0; i < v.stop_line.size(); ++i)
    {
      s << indent << "  stop_line[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::geometry_msgs::Point_<ContainerAllocator> >::stream(s, indent + "    ", v.stop_line[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // BRAIN_MSGS_MESSAGE_SPEEDPROFILE_H
