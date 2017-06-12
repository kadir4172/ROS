// Generated by gencpp from file lama_msgs/SetCrossingRequest.msg
// DO NOT EDIT!


#ifndef LAMA_MSGS_MESSAGE_SETCROSSINGREQUEST_H
#define LAMA_MSGS_MESSAGE_SETCROSSINGREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <lama_msgs/Crossing.h>

namespace lama_msgs
{
template <class ContainerAllocator>
struct SetCrossingRequest_
{
  typedef SetCrossingRequest_<ContainerAllocator> Type;

  SetCrossingRequest_()
    : descriptor()  {
    }
  SetCrossingRequest_(const ContainerAllocator& _alloc)
    : descriptor(_alloc)  {
  (void)_alloc;
    }



   typedef  ::lama_msgs::Crossing_<ContainerAllocator>  _descriptor_type;
  _descriptor_type descriptor;




  typedef boost::shared_ptr< ::lama_msgs::SetCrossingRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::lama_msgs::SetCrossingRequest_<ContainerAllocator> const> ConstPtr;

}; // struct SetCrossingRequest_

typedef ::lama_msgs::SetCrossingRequest_<std::allocator<void> > SetCrossingRequest;

typedef boost::shared_ptr< ::lama_msgs::SetCrossingRequest > SetCrossingRequestPtr;
typedef boost::shared_ptr< ::lama_msgs::SetCrossingRequest const> SetCrossingRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::lama_msgs::SetCrossingRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::lama_msgs::SetCrossingRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace lama_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'lama_msgs': ['/home/kadir/Desktop/Freelancer/Project4_ROS/ROS/catkin_ws/src/lama_msgs/msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::lama_msgs::SetCrossingRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::lama_msgs::SetCrossingRequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::lama_msgs::SetCrossingRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::lama_msgs::SetCrossingRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::lama_msgs::SetCrossingRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::lama_msgs::SetCrossingRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::lama_msgs::SetCrossingRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "753047a0f3e0e7bbdd77b36b85512c99";
  }

  static const char* value(const ::lama_msgs::SetCrossingRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x753047a0f3e0e7bbULL;
  static const uint64_t static_value2 = 0xdd77b36b85512c99ULL;
};

template<class ContainerAllocator>
struct DataType< ::lama_msgs::SetCrossingRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "lama_msgs/SetCrossingRequest";
  }

  static const char* value(const ::lama_msgs::SetCrossingRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::lama_msgs::SetCrossingRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Crossing descriptor\n\
\n\
================================================================================\n\
MSG: lama_msgs/Crossing\n\
# The main aim of a crossing is to store frontier information.\n\
\n\
# Crossing center.\n\
geometry_msgs/Point center\n\
\n\
# Crossing radius.\n\
float32 radius\n\
\n\
# Max. range of the data used to compute the crossing.\n\
float32 max_range\n\
\n\
Frontier[] frontiers\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Point\n\
# This contains the position of a point in free space\n\
float64 x\n\
float64 y\n\
float64 z\n\
\n\
================================================================================\n\
MSG: lama_msgs/Frontier\n\
# Frontier is a line segment through which a robot can go.\n\
\n\
# First point\n\
geometry_msgs/Point p1\n\
\n\
# Second point, so that angle(r-p1, r-p2) is positive, where r is crossing center.\n\
geometry_msgs/Point p2\n\
\n\
# Segment length, i.e. width of free space\n\
float32 width\n\
\n\
# Angle between x-axis and line between origin and frontier middle. Where x-axis\n\
# and origin refer to the data from which the frontier is computed, in general a LaserScan.\n\
float32 angle\n\
";
  }

  static const char* value(const ::lama_msgs::SetCrossingRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::lama_msgs::SetCrossingRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.descriptor);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SetCrossingRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::lama_msgs::SetCrossingRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::lama_msgs::SetCrossingRequest_<ContainerAllocator>& v)
  {
    s << indent << "descriptor: ";
    s << std::endl;
    Printer< ::lama_msgs::Crossing_<ContainerAllocator> >::stream(s, indent + "  ", v.descriptor);
  }
};

} // namespace message_operations
} // namespace ros

#endif // LAMA_MSGS_MESSAGE_SETCROSSINGREQUEST_H
