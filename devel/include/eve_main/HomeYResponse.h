// Generated by gencpp from file eve_main/HomeYResponse.msg
// DO NOT EDIT!


#ifndef EVE_MAIN_MESSAGE_HOMEYRESPONSE_H
#define EVE_MAIN_MESSAGE_HOMEYRESPONSE_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace eve_main
{
template <class ContainerAllocator>
struct HomeYResponse_
{
  typedef HomeYResponse_<ContainerAllocator> Type;

  HomeYResponse_()
    {
    }
  HomeYResponse_(const ContainerAllocator& _alloc)
    {
  (void)_alloc;
    }







  typedef boost::shared_ptr< ::eve_main::HomeYResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::eve_main::HomeYResponse_<ContainerAllocator> const> ConstPtr;

}; // struct HomeYResponse_

typedef ::eve_main::HomeYResponse_<std::allocator<void> > HomeYResponse;

typedef boost::shared_ptr< ::eve_main::HomeYResponse > HomeYResponsePtr;
typedef boost::shared_ptr< ::eve_main::HomeYResponse const> HomeYResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::eve_main::HomeYResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::eve_main::HomeYResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


} // namespace eve_main

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::eve_main::HomeYResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::eve_main::HomeYResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::eve_main::HomeYResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::eve_main::HomeYResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::eve_main::HomeYResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::eve_main::HomeYResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::eve_main::HomeYResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const ::eve_main::HomeYResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::eve_main::HomeYResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "eve_main/HomeYResponse";
  }

  static const char* value(const ::eve_main::HomeYResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::eve_main::HomeYResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
;
  }

  static const char* value(const ::eve_main::HomeYResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::eve_main::HomeYResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream&, T)
    {}

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct HomeYResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::eve_main::HomeYResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream&, const std::string&, const ::eve_main::HomeYResponse_<ContainerAllocator>&)
  {}
};

} // namespace message_operations
} // namespace ros

#endif // EVE_MAIN_MESSAGE_HOMEYRESPONSE_H
