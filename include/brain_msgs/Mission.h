// Generated by gencpp from file brain_msgs/Mission.msg
// DO NOT EDIT!


#ifndef BRAIN_MSGS_MESSAGE_MISSION_H
#define BRAIN_MSGS_MESSAGE_MISSION_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace brain_msgs
{
template <class ContainerAllocator>
struct Mission_
{
  typedef Mission_<ContainerAllocator> Type;

  Mission_()
    : header()
    , mission(0)  {
    }
  Mission_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , mission(0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef uint8_t _mission_type;
  _mission_type mission;



// reducing the odds to have name collisions with Windows.h 
#if defined(_WIN32) && defined(unknown_mission)
  #undef unknown_mission
#endif
#if defined(_WIN32) && defined(start)
  #undef start
#endif
#if defined(_WIN32) && defined(wait_request)
  #undef wait_request
#endif
#if defined(_WIN32) && defined(drive)
  #undef drive
#endif
#if defined(_WIN32) && defined(aborted)
  #undef aborted
#endif
#if defined(_WIN32) && defined(complete)
  #undef complete
#endif

  enum {
    unknown_mission = 0u,
    start = 1u,
    wait_request = 2u,
    drive = 3u,
    aborted = 4u,
    complete = 5u,
  };


  typedef boost::shared_ptr< ::brain_msgs::Mission_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::brain_msgs::Mission_<ContainerAllocator> const> ConstPtr;

}; // struct Mission_

typedef ::brain_msgs::Mission_<std::allocator<void> > Mission;

typedef boost::shared_ptr< ::brain_msgs::Mission > MissionPtr;
typedef boost::shared_ptr< ::brain_msgs::Mission const> MissionConstPtr;

// constants requiring out of line definition

   

   

   

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::brain_msgs::Mission_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::brain_msgs::Mission_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::brain_msgs::Mission_<ContainerAllocator1> & lhs, const ::brain_msgs::Mission_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.mission == rhs.mission;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::brain_msgs::Mission_<ContainerAllocator1> & lhs, const ::brain_msgs::Mission_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace brain_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::brain_msgs::Mission_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::brain_msgs::Mission_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::brain_msgs::Mission_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::brain_msgs::Mission_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::brain_msgs::Mission_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::brain_msgs::Mission_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::brain_msgs::Mission_<ContainerAllocator> >
{
  static const char* value()
  {
    return "a8cb2c1b8ebfdd94e4ff207c627e069c";
  }

  static const char* value(const ::brain_msgs::Mission_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xa8cb2c1b8ebfdd94ULL;
  static const uint64_t static_value2 = 0xe4ff207c627e069cULL;
};

template<class ContainerAllocator>
struct DataType< ::brain_msgs::Mission_<ContainerAllocator> >
{
  static const char* value()
  {
    return "brain_msgs/Mission";
  }

  static const char* value(const ::brain_msgs::Mission_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::brain_msgs::Mission_<ContainerAllocator> >
{
  static const char* value()
  {
    return "std_msgs/Header     header \n"
"\n"
"uint8               mission\n"
"uint8               unknown_mission=0\n"
"uint8               start=1\n"
"uint8               wait_request=2\n"
"uint8               drive=3\n"
"uint8               aborted=4\n"
"uint8               complete=5\n"
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
;
  }

  static const char* value(const ::brain_msgs::Mission_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::brain_msgs::Mission_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.mission);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Mission_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::brain_msgs::Mission_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::brain_msgs::Mission_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "mission: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.mission);
  }
};

} // namespace message_operations
} // namespace ros

#endif // BRAIN_MSGS_MESSAGE_MISSION_H
