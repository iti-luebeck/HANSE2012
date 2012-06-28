/* Auto-generated by genmsg_cpp for file /home/hanse/hanse-ros/hanse_ros/hanse_pidcontrol/srv/Enable.srv */
#ifndef HANSE_PIDCONTROL_SERVICE_ENABLE_H
#define HANSE_PIDCONTROL_SERVICE_ENABLE_H
#include <string>
#include <vector>
#include <map>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/time.h"

#include "ros/macros.h"

#include "ros/assert.h"

#include "ros/service_traits.h"




namespace hanse_pidcontrol
{
template <class ContainerAllocator>
struct EnableRequest_ {
  typedef EnableRequest_<ContainerAllocator> Type;

  EnableRequest_()
  : enable(false)
  {
  }

  EnableRequest_(const ContainerAllocator& _alloc)
  : enable(false)
  {
  }

  typedef uint8_t _enable_type;
  uint8_t enable;


  typedef boost::shared_ptr< ::hanse_pidcontrol::EnableRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::hanse_pidcontrol::EnableRequest_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct EnableRequest
typedef  ::hanse_pidcontrol::EnableRequest_<std::allocator<void> > EnableRequest;

typedef boost::shared_ptr< ::hanse_pidcontrol::EnableRequest> EnableRequestPtr;
typedef boost::shared_ptr< ::hanse_pidcontrol::EnableRequest const> EnableRequestConstPtr;


template <class ContainerAllocator>
struct EnableResponse_ {
  typedef EnableResponse_<ContainerAllocator> Type;

  EnableResponse_()
  {
  }

  EnableResponse_(const ContainerAllocator& _alloc)
  {
  }


  typedef boost::shared_ptr< ::hanse_pidcontrol::EnableResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::hanse_pidcontrol::EnableResponse_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct EnableResponse
typedef  ::hanse_pidcontrol::EnableResponse_<std::allocator<void> > EnableResponse;

typedef boost::shared_ptr< ::hanse_pidcontrol::EnableResponse> EnableResponsePtr;
typedef boost::shared_ptr< ::hanse_pidcontrol::EnableResponse const> EnableResponseConstPtr;

struct Enable
{

typedef EnableRequest Request;
typedef EnableResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct Enable
} // namespace hanse_pidcontrol

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::hanse_pidcontrol::EnableRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::hanse_pidcontrol::EnableRequest_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::hanse_pidcontrol::EnableRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "8c1211af706069c994c06e00eb59ac9e";
  }

  static const char* value(const  ::hanse_pidcontrol::EnableRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x8c1211af706069c9ULL;
  static const uint64_t static_value2 = 0x94c06e00eb59ac9eULL;
};

template<class ContainerAllocator>
struct DataType< ::hanse_pidcontrol::EnableRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "hanse_pidcontrol/EnableRequest";
  }

  static const char* value(const  ::hanse_pidcontrol::EnableRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::hanse_pidcontrol::EnableRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "bool enable\n\
\n\
";
  }

  static const char* value(const  ::hanse_pidcontrol::EnableRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::hanse_pidcontrol::EnableRequest_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::hanse_pidcontrol::EnableResponse_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::hanse_pidcontrol::EnableResponse_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::hanse_pidcontrol::EnableResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const  ::hanse_pidcontrol::EnableResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::hanse_pidcontrol::EnableResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "hanse_pidcontrol/EnableResponse";
  }

  static const char* value(const  ::hanse_pidcontrol::EnableResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::hanse_pidcontrol::EnableResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "\n\
\n\
";
  }

  static const char* value(const  ::hanse_pidcontrol::EnableResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::hanse_pidcontrol::EnableResponse_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::hanse_pidcontrol::EnableRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.enable);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct EnableRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::hanse_pidcontrol::EnableResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct EnableResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<hanse_pidcontrol::Enable> {
  static const char* value() 
  {
    return "8c1211af706069c994c06e00eb59ac9e";
  }

  static const char* value(const hanse_pidcontrol::Enable&) { return value(); } 
};

template<>
struct DataType<hanse_pidcontrol::Enable> {
  static const char* value() 
  {
    return "hanse_pidcontrol/Enable";
  }

  static const char* value(const hanse_pidcontrol::Enable&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<hanse_pidcontrol::EnableRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "8c1211af706069c994c06e00eb59ac9e";
  }

  static const char* value(const hanse_pidcontrol::EnableRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<hanse_pidcontrol::EnableRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "hanse_pidcontrol/Enable";
  }

  static const char* value(const hanse_pidcontrol::EnableRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<hanse_pidcontrol::EnableResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "8c1211af706069c994c06e00eb59ac9e";
  }

  static const char* value(const hanse_pidcontrol::EnableResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<hanse_pidcontrol::EnableResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "hanse_pidcontrol/Enable";
  }

  static const char* value(const hanse_pidcontrol::EnableResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // HANSE_PIDCONTROL_SERVICE_ENABLE_H

