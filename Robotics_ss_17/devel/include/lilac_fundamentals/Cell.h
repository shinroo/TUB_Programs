// Generated by gencpp from file lilac_fundamentals/Cell.msg
// DO NOT EDIT!


#ifndef LILAC_FUNDAMENTALS_MESSAGE_CELL_H
#define LILAC_FUNDAMENTALS_MESSAGE_CELL_H


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
struct Cell_
{
  typedef Cell_<ContainerAllocator> Type;

  Cell_()
    : walls()  {
    }
  Cell_(const ContainerAllocator& _alloc)
    : walls(_alloc)  {
    }



   typedef std::vector<int32_t, typename ContainerAllocator::template rebind<int32_t>::other >  _walls_type;
  _walls_type walls;


    enum { RIGHT = 0 };
     enum { TOP = 1 };
     enum { LEFT = 2 };
     enum { BOTTOM = 3 };
 

  typedef boost::shared_ptr< ::lilac_fundamentals::Cell_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::lilac_fundamentals::Cell_<ContainerAllocator> const> ConstPtr;

}; // struct Cell_

typedef ::lilac_fundamentals::Cell_<std::allocator<void> > Cell;

typedef boost::shared_ptr< ::lilac_fundamentals::Cell > CellPtr;
typedef boost::shared_ptr< ::lilac_fundamentals::Cell const> CellConstPtr;

// constants requiring out of line definition

   

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::lilac_fundamentals::Cell_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::lilac_fundamentals::Cell_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace lilac_fundamentals

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'lilac_fundamentals': ['/home/liliac/lilac/src/lilac_fundamentals/msg', '/home/liliac/lilac/src/lilac_fundamentals/msg'], 'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::lilac_fundamentals::Cell_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::lilac_fundamentals::Cell_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::lilac_fundamentals::Cell_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::lilac_fundamentals::Cell_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::lilac_fundamentals::Cell_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::lilac_fundamentals::Cell_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::lilac_fundamentals::Cell_<ContainerAllocator> >
{
  static const char* value()
  {
    return "95bb061dc101cae41f56cabb3faafc66";
  }

  static const char* value(const ::lilac_fundamentals::Cell_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x95bb061dc101cae4ULL;
  static const uint64_t static_value2 = 0x1f56cabb3faafc66ULL;
};

template<class ContainerAllocator>
struct DataType< ::lilac_fundamentals::Cell_<ContainerAllocator> >
{
  static const char* value()
  {
    return "lilac_fundamentals/Cell";
  }

  static const char* value(const ::lilac_fundamentals::Cell_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::lilac_fundamentals::Cell_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# constants for walls\n\
int32 RIGHT = 0\n\
int32 TOP = 1\n\
int32 LEFT = 2\n\
int32 BOTTOM = 3\n\
# walls that are present in this cell\n\
int32[] walls\n\
";
  }

  static const char* value(const ::lilac_fundamentals::Cell_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::lilac_fundamentals::Cell_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.walls);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct Cell_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::lilac_fundamentals::Cell_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::lilac_fundamentals::Cell_<ContainerAllocator>& v)
  {
    s << indent << "walls[]" << std::endl;
    for (size_t i = 0; i < v.walls.size(); ++i)
    {
      s << indent << "  walls[" << i << "]: ";
      Printer<int32_t>::stream(s, indent + "  ", v.walls[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // LILAC_FUNDAMENTALS_MESSAGE_CELL_H
