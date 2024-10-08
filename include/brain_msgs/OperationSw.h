// Generated by gencpp from file brain_msgs/OperationSw.msg
// DO NOT EDIT!


#ifndef BRAIN_MSGS_MESSAGE_OPERATIONSW_H
#define BRAIN_MSGS_MESSAGE_OPERATIONSW_H


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
struct OperationSw_
{
  typedef OperationSw_<ContainerAllocator> Type;

  OperationSw_()
    : header()
    , operation_sw(false)
    , autonomous_sw(false)
    , emergency_sw(false)  {
    }
  OperationSw_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , operation_sw(false)
    , autonomous_sw(false)
    , emergency_sw(false)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef uint8_t _operation_sw_type;
  _operation_sw_type operation_sw;

   typedef uint8_t _autonomous_sw_type;
  _autonomous_sw_type autonomous_sw;

   typedef uint8_t _emergency_sw_type;
  _emergency_sw_type emergency_sw;





  typedef boost::shared_ptr< ::brain_msgs::OperationSw_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::brain_msgs::OperationSw_<ContainerAllocator> const> ConstPtr;

}; // struct OperationSw_

typedef ::brain_msgs::OperationSw_<std::allocator<void> > OperationSw;

typedef boost::shared_ptr< ::brain_msgs::OperationSw > OperationSwPtr;
typedef boost::shared_ptr< ::brain_msgs::OperationSw const> OperationSwConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::brain_msgs::OperationSw_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::brain_msgs::OperationSw_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::brain_msgs::OperationSw_<ContainerAllocator1> & lhs, const ::brain_msgs::OperationSw_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.operation_sw == rhs.operation_sw &&
    lhs.autonomous_sw == rhs.autonomous_sw &&
    lhs.emergency_sw == rhs.emergency_sw;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::brain_msgs::OperationSw_<ContainerAllocator1> & lhs, const ::brain_msgs::OperationSw_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace brain_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::brain_msgs::OperationSw_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::brain_msgs::OperationSw_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::brain_msgs::OperationSw_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::brain_msgs::OperationSw_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::brain_msgs::OperationSw_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::brain_msgs::OperationSw_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::brain_msgs::OperationSw_<ContainerAllocator> >
{
  static const char* value()
  {
    return "e8d7351a44c805f9203b1c0379785b37";
  }

  static const char* value(const ::brain_msgs::OperationSw_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xe8d7351a44c805f9ULL;
  static const uint64_t static_value2 = 0x203b1c0379785b37ULL;
};

template<class ContainerAllocator>
struct DataType< ::brain_msgs::OperationSw_<ContainerAllocator> >
{
  static const char* value()
  {
    return "brain_msgs/OperationSw";
  }

  static const char* value(const ::brain_msgs::OperationSw_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::brain_msgs::OperationSw_<ContainerAllocator> >
{
  static const char* value()
  {
    return "std_msgs/Header        header \n"
"\n"
"bool operation_sw\n"
"bool autonomous_sw\n"
"bool emergency_sw\n"
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

  static const char* value(const ::brain_msgs::OperationSw_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::brain_msgs::OperationSw_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.operation_sw);
      stream.next(m.autonomous_sw);
      stream.next(m.emergency_sw);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct OperationSw_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::brain_msgs::OperationSw_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::brain_msgs::OperationSw_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "operation_sw: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.operation_sw);
    s << indent << "autonomous_sw: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.autonomous_sw);
    s << indent << "emergency_sw: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.emergency_sw);
  }
};

} // namespace message_operations
} // namespace ros

#endif // BRAIN_MSGS_MESSAGE_OPERATIONSW_H
