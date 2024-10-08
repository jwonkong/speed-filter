// Generated by gencpp from file brain_msgs/Route.msg
// DO NOT EDIT!


#ifndef BRAIN_MSGS_MESSAGE_ROUTE_H
#define BRAIN_MSGS_MESSAGE_ROUTE_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <brain_msgs/Relation.h>

namespace brain_msgs
{
template <class ContainerAllocator>
struct Route_
{
  typedef Route_<ContainerAllocator> Type;

  Route_()
    : route()  {
    }
  Route_(const ContainerAllocator& _alloc)
    : route(_alloc)  {
  (void)_alloc;
    }



   typedef std::vector< ::brain_msgs::Relation_<ContainerAllocator> , typename std::allocator_traits<ContainerAllocator>::template rebind_alloc< ::brain_msgs::Relation_<ContainerAllocator> >> _route_type;
  _route_type route;





  typedef boost::shared_ptr< ::brain_msgs::Route_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::brain_msgs::Route_<ContainerAllocator> const> ConstPtr;

}; // struct Route_

typedef ::brain_msgs::Route_<std::allocator<void> > Route;

typedef boost::shared_ptr< ::brain_msgs::Route > RoutePtr;
typedef boost::shared_ptr< ::brain_msgs::Route const> RouteConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::brain_msgs::Route_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::brain_msgs::Route_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::brain_msgs::Route_<ContainerAllocator1> & lhs, const ::brain_msgs::Route_<ContainerAllocator2> & rhs)
{
  return lhs.route == rhs.route;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::brain_msgs::Route_<ContainerAllocator1> & lhs, const ::brain_msgs::Route_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace brain_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::brain_msgs::Route_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::brain_msgs::Route_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::brain_msgs::Route_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::brain_msgs::Route_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::brain_msgs::Route_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::brain_msgs::Route_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::brain_msgs::Route_<ContainerAllocator> >
{
  static const char* value()
  {
    return "5e4e1b84b6e99c6f3a5ca0cae88f1bc9";
  }

  static const char* value(const ::brain_msgs::Route_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x5e4e1b84b6e99c6fULL;
  static const uint64_t static_value2 = 0x3a5ca0cae88f1bc9ULL;
};

template<class ContainerAllocator>
struct DataType< ::brain_msgs::Route_<ContainerAllocator> >
{
  static const char* value()
  {
    return "brain_msgs/Route";
  }

  static const char* value(const ::brain_msgs::Route_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::brain_msgs::Route_<ContainerAllocator> >
{
  static const char* value()
  {
    return "brain_msgs/Relation[] route\n"
"================================================================================\n"
"MSG: brain_msgs/Relation\n"
"int64 id \n"
"uint8 relation\n"
"uint8 unknown=0\n"
"uint8 lane_keeping=1\n"
"uint8 change_left=2\n"
"uint8 change_right=3\n"
"uint8 adjacent_left=4\n"
"uint8 adjacent_right=5\n"
"uint8 conflict=6\n"
"uint8 area=7\n"
;
  }

  static const char* value(const ::brain_msgs::Route_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::brain_msgs::Route_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.route);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Route_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::brain_msgs::Route_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::brain_msgs::Route_<ContainerAllocator>& v)
  {
    s << indent << "route[]" << std::endl;
    for (size_t i = 0; i < v.route.size(); ++i)
    {
      s << indent << "  route[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::brain_msgs::Relation_<ContainerAllocator> >::stream(s, indent + "    ", v.route[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // BRAIN_MSGS_MESSAGE_ROUTE_H
