// Generated by gencpp from file lilac_fundamentals/ActualLocalization.msg
// DO NOT EDIT!


#ifndef LILAC_FUNDAMENTALS_MESSAGE_ACTUALLOCALIZATION_H
#define LILAC_FUNDAMENTALS_MESSAGE_ACTUALLOCALIZATION_H


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
struct ActualLocalization_
{
  typedef ActualLocalization_<ContainerAllocator> Type;

  ActualLocalization_()
    : row(0)
    , column(0)
    , orientation(0)
    , localized(false)  {
    }
  ActualLocalization_(const ContainerAllocator& _alloc)
    : row(0)
    , column(0)
    , orientation(0)
    , localized(false)  {
    }



   typedef int16_t _row_type;
  _row_type row;

   typedef int16_t _column_type;
  _column_type column;

   typedef int16_t _orientation_type;
  _orientation_type orientation;

   typedef uint8_t _localized_type;
  _localized_type localized;




  typedef boost::shared_ptr< ::lilac_fundamentals::ActualLocalization_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::lilac_fundamentals::ActualLocalization_<ContainerAllocator> const> ConstPtr;

}; // struct ActualLocalization_

typedef ::lilac_fundamentals::ActualLocalization_<std::allocator<void> > ActualLocalization;

typedef boost::shared_ptr< ::lilac_fundamentals::ActualLocalization > ActualLocalizationPtr;
typedef boost::shared_ptr< ::lilac_fundamentals::ActualLocalization const> ActualLocalizationConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::lilac_fundamentals::ActualLocalization_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::lilac_fundamentals::ActualLocalization_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::lilac_fundamentals::ActualLocalization_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::lilac_fundamentals::ActualLocalization_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::lilac_fundamentals::ActualLocalization_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::lilac_fundamentals::ActualLocalization_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::lilac_fundamentals::ActualLocalization_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::lilac_fundamentals::ActualLocalization_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::lilac_fundamentals::ActualLocalization_<ContainerAllocator> >
{
  static const char* value()
  {
    return "893e067a4f69fb209f2b2e037a5aa78b";
  }

  static const char* value(const ::lilac_fundamentals::ActualLocalization_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x893e067a4f69fb20ULL;
  static const uint64_t static_value2 = 0x9f2b2e037a5aa78bULL;
};

template<class ContainerAllocator>
struct DataType< ::lilac_fundamentals::ActualLocalization_<ContainerAllocator> >
{
  static const char* value()
  {
    return "lilac_fundamentals/ActualLocalization";
  }

  static const char* value(const ::lilac_fundamentals::ActualLocalization_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::lilac_fundamentals::ActualLocalization_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int16 row\n\
int16 column\n\
int16 orientation\n\
bool localized\n\
";
  }

  static const char* value(const ::lilac_fundamentals::ActualLocalization_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::lilac_fundamentals::ActualLocalization_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.row);
      stream.next(m.column);
      stream.next(m.orientation);
      stream.next(m.localized);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct ActualLocalization_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::lilac_fundamentals::ActualLocalization_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::lilac_fundamentals::ActualLocalization_<ContainerAllocator>& v)
  {
    s << indent << "row: ";
    Printer<int16_t>::stream(s, indent + "  ", v.row);
    s << indent << "column: ";
    Printer<int16_t>::stream(s, indent + "  ", v.column);
    s << indent << "orientation: ";
    Printer<int16_t>::stream(s, indent + "  ", v.orientation);
    s << indent << "localized: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.localized);
  }
};

} // namespace message_operations
} // namespace ros

#endif // LILAC_FUNDAMENTALS_MESSAGE_ACTUALLOCALIZATION_H