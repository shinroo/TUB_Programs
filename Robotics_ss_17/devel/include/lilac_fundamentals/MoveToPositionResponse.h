// Generated by gencpp from file lilac_fundamentals/MoveToPositionResponse.msg
// DO NOT EDIT!


#ifndef LILAC_FUNDAMENTALS_MESSAGE_MOVETOPOSITIONRESPONSE_H
#define LILAC_FUNDAMENTALS_MESSAGE_MOVETOPOSITIONRESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace lilac_fundamentals
{
template <class ContainerAllocator>
struct MoveToPositionResponse_
{
  typedef MoveToPositionResponse_<ContainerAllocator> Type;

  MoveToPositionResponse_()
    : success(false)  {
    }
  MoveToPositionResponse_(const ContainerAllocator& _alloc)
    : success(false)  {
    }



   typedef uint8_t _success_type;
  _success_type success;




  typedef boost::shared_ptr< ::lilac_fundamentals::MoveToPositionResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::lilac_fundamentals::MoveToPositionResponse_<ContainerAllocator> const> ConstPtr;

}; // struct MoveToPositionResponse_

typedef ::lilac_fundamentals::MoveToPositionResponse_<std::allocator<void> > MoveToPositionResponse;

typedef boost::shared_ptr< ::lilac_fundamentals::MoveToPositionResponse > MoveToPositionResponsePtr;
typedef boost::shared_ptr< ::lilac_fundamentals::MoveToPositionResponse const> MoveToPositionResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::lilac_fundamentals::MoveToPositionResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::lilac_fundamentals::MoveToPositionResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace lilac_fundamentals

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'lilac_fundamentals': ['/home/liliac/lilac/src/lilac_fundamentals/msg', '/home/liliac/lilac/src/lilac_fundamentals/msg'], 'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::lilac_fundamentals::MoveToPositionResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::lilac_fundamentals::MoveToPositionResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::lilac_fundamentals::MoveToPositionResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::lilac_fundamentals::MoveToPositionResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::lilac_fundamentals::MoveToPositionResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::lilac_fundamentals::MoveToPositionResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::lilac_fundamentals::MoveToPositionResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "358e233cde0c8a8bcfea4ce193f8fc15";
  }

  static const char* value(const ::lilac_fundamentals::MoveToPositionResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x358e233cde0c8a8bULL;
  static const uint64_t static_value2 = 0xcfea4ce193f8fc15ULL;
};

template<class ContainerAllocator>
struct DataType< ::lilac_fundamentals::MoveToPositionResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "lilac_fundamentals/MoveToPositionResponse";
  }

  static const char* value(const ::lilac_fundamentals::MoveToPositionResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::lilac_fundamentals::MoveToPositionResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n\
bool success\n\
\n\
";
  }

  static const char* value(const ::lilac_fundamentals::MoveToPositionResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::lilac_fundamentals::MoveToPositionResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.success);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct MoveToPositionResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::lilac_fundamentals::MoveToPositionResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::lilac_fundamentals::MoveToPositionResponse_<ContainerAllocator>& v)
  {
    s << indent << "success: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.success);
  }
};

} // namespace message_operations
} // namespace ros

#endif // LILAC_FUNDAMENTALS_MESSAGE_MOVETOPOSITIONRESPONSE_H
