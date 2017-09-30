// Generated by gencpp from file create_fundamentals/DiffDriveResponse.msg
// DO NOT EDIT!


#ifndef CREATE_FUNDAMENTALS_MESSAGE_DIFFDRIVERESPONSE_H
#define CREATE_FUNDAMENTALS_MESSAGE_DIFFDRIVERESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace create_fundamentals
{
template <class ContainerAllocator>
struct DiffDriveResponse_
{
  typedef DiffDriveResponse_<ContainerAllocator> Type;

  DiffDriveResponse_()
    : success(false)  {
    }
  DiffDriveResponse_(const ContainerAllocator& _alloc)
    : success(false)  {
    }



   typedef uint8_t _success_type;
  _success_type success;




  typedef boost::shared_ptr< ::create_fundamentals::DiffDriveResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::create_fundamentals::DiffDriveResponse_<ContainerAllocator> const> ConstPtr;

}; // struct DiffDriveResponse_

typedef ::create_fundamentals::DiffDriveResponse_<std::allocator<void> > DiffDriveResponse;

typedef boost::shared_ptr< ::create_fundamentals::DiffDriveResponse > DiffDriveResponsePtr;
typedef boost::shared_ptr< ::create_fundamentals::DiffDriveResponse const> DiffDriveResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::create_fundamentals::DiffDriveResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::create_fundamentals::DiffDriveResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace create_fundamentals

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'sensor_msgs': ['/opt/ros/indigo/share/sensor_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg'], 'create_fundamentals': ['/home/liliac/catkin_ws/src/create_fundamentals/msg'], 'geometry_msgs': ['/opt/ros/indigo/share/geometry_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::create_fundamentals::DiffDriveResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::create_fundamentals::DiffDriveResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::create_fundamentals::DiffDriveResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::create_fundamentals::DiffDriveResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::create_fundamentals::DiffDriveResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::create_fundamentals::DiffDriveResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::create_fundamentals::DiffDriveResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "358e233cde0c8a8bcfea4ce193f8fc15";
  }

  static const char* value(const ::create_fundamentals::DiffDriveResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x358e233cde0c8a8bULL;
  static const uint64_t static_value2 = 0xcfea4ce193f8fc15ULL;
};

template<class ContainerAllocator>
struct DataType< ::create_fundamentals::DiffDriveResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "create_fundamentals/DiffDriveResponse";
  }

  static const char* value(const ::create_fundamentals::DiffDriveResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::create_fundamentals::DiffDriveResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bool success\n\
\n\
";
  }

  static const char* value(const ::create_fundamentals::DiffDriveResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::create_fundamentals::DiffDriveResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.success);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct DiffDriveResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::create_fundamentals::DiffDriveResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::create_fundamentals::DiffDriveResponse_<ContainerAllocator>& v)
  {
    s << indent << "success: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.success);
  }
};

} // namespace message_operations
} // namespace ros

#endif // CREATE_FUNDAMENTALS_MESSAGE_DIFFDRIVERESPONSE_H