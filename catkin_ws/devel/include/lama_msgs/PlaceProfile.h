// Generated by gencpp from file lama_msgs/PlaceProfile.msg
// DO NOT EDIT!


#ifndef LAMA_MSGS_MESSAGE_PLACEPROFILE_H
#define LAMA_MSGS_MESSAGE_PLACEPROFILE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <geometry_msgs/Polygon.h>

namespace lama_msgs
{
template <class ContainerAllocator>
struct PlaceProfile_
{
  typedef PlaceProfile_<ContainerAllocator> Type;

  PlaceProfile_()
    : header()
    , polygon()
    , exclude_segments()  {
    }
  PlaceProfile_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , polygon(_alloc)
    , exclude_segments(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef  ::geometry_msgs::Polygon_<ContainerAllocator>  _polygon_type;
  _polygon_type polygon;

   typedef std::vector<int32_t, typename ContainerAllocator::template rebind<int32_t>::other >  _exclude_segments_type;
  _exclude_segments_type exclude_segments;




  typedef boost::shared_ptr< ::lama_msgs::PlaceProfile_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::lama_msgs::PlaceProfile_<ContainerAllocator> const> ConstPtr;

}; // struct PlaceProfile_

typedef ::lama_msgs::PlaceProfile_<std::allocator<void> > PlaceProfile;

typedef boost::shared_ptr< ::lama_msgs::PlaceProfile > PlaceProfilePtr;
typedef boost::shared_ptr< ::lama_msgs::PlaceProfile const> PlaceProfileConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::lama_msgs::PlaceProfile_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::lama_msgs::PlaceProfile_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace lama_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'lama_msgs': ['/home/kadir/Desktop/Freelancer/Project4_ROS/ROS/catkin_ws/src/lama_msgs/msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::lama_msgs::PlaceProfile_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::lama_msgs::PlaceProfile_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::lama_msgs::PlaceProfile_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::lama_msgs::PlaceProfile_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::lama_msgs::PlaceProfile_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::lama_msgs::PlaceProfile_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::lama_msgs::PlaceProfile_<ContainerAllocator> >
{
  static const char* value()
  {
    return "c55b56133425a18c250b94124f4feaa1";
  }

  static const char* value(const ::lama_msgs::PlaceProfile_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xc55b56133425a18cULL;
  static const uint64_t static_value2 = 0x250b94124f4feaa1ULL;
};

template<class ContainerAllocator>
struct DataType< ::lama_msgs::PlaceProfile_<ContainerAllocator> >
{
  static const char* value()
  {
    return "lama_msgs/PlaceProfile";
  }

  static const char* value(const ::lama_msgs::PlaceProfile_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::lama_msgs::PlaceProfile_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# 2D Description of a place by some points at obstacle borders\n\
# The difference with a LaserScan is that some segments can\n\
# be marked as to be excluded. Such segments will be the segments\n\
# where the range is the maximum range of the LaserSCan or\n\
# borders between free and unknown pixels of a costmap.\n\
\n\
Header header\n\
\n\
geometry_msgs/Polygon polygon\n\
\n\
# Segments to exclude, segment i is between point i and i+1.\n\
# The last segment is between points (n-1) and 0, where n is the number of points.\n\
int32[] exclude_segments\n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n\
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Polygon\n\
#A specification of a polygon where the first and last points are assumed to be connected\n\
Point32[] points\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Point32\n\
# This contains the position of a point in free space(with 32 bits of precision).\n\
# It is recommeded to use Point wherever possible instead of Point32.  \n\
# \n\
# This recommendation is to promote interoperability.  \n\
#\n\
# This message is designed to take up less space when sending\n\
# lots of points at once, as in the case of a PointCloud.  \n\
\n\
float32 x\n\
float32 y\n\
float32 z\n\
";
  }

  static const char* value(const ::lama_msgs::PlaceProfile_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::lama_msgs::PlaceProfile_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.polygon);
      stream.next(m.exclude_segments);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct PlaceProfile_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::lama_msgs::PlaceProfile_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::lama_msgs::PlaceProfile_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "polygon: ";
    s << std::endl;
    Printer< ::geometry_msgs::Polygon_<ContainerAllocator> >::stream(s, indent + "  ", v.polygon);
    s << indent << "exclude_segments[]" << std::endl;
    for (size_t i = 0; i < v.exclude_segments.size(); ++i)
    {
      s << indent << "  exclude_segments[" << i << "]: ";
      Printer<int32_t>::stream(s, indent + "  ", v.exclude_segments[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // LAMA_MSGS_MESSAGE_PLACEPROFILE_H
