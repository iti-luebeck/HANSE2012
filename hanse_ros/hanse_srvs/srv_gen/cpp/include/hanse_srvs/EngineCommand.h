/* Auto-generated by genmsg_cpp for file /home/hanse/ros/ROS/hanse_ros/hanse_srvs/srv/EngineCommand.srv */
#ifndef HANSE_SRVS_SERVICE_ENGINECOMMAND_H
#define HANSE_SRVS_SERVICE_ENGINECOMMAND_H
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




namespace hanse_srvs
{
template <class ContainerAllocator>
struct EngineCommandRequest_ {
  typedef EngineCommandRequest_<ContainerAllocator> Type;

  EngineCommandRequest_()
  : enableDepthPid(false)
  , enableOrientationPid(false)
  , enableMotors(false)
  , resetZeroPressure(false)
  , setEmergencyStop(false)
  {
  }

  EngineCommandRequest_(const ContainerAllocator& _alloc)
  : enableDepthPid(false)
  , enableOrientationPid(false)
  , enableMotors(false)
  , resetZeroPressure(false)
  , setEmergencyStop(false)
  {
  }

  typedef uint8_t _enableDepthPid_type;
  uint8_t enableDepthPid;

  typedef uint8_t _enableOrientationPid_type;
  uint8_t enableOrientationPid;

  typedef uint8_t _enableMotors_type;
  uint8_t enableMotors;

  typedef uint8_t _resetZeroPressure_type;
  uint8_t resetZeroPressure;

  typedef uint8_t _setEmergencyStop_type;
  uint8_t setEmergencyStop;


  typedef boost::shared_ptr< ::hanse_srvs::EngineCommandRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::hanse_srvs::EngineCommandRequest_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct EngineCommandRequest
typedef  ::hanse_srvs::EngineCommandRequest_<std::allocator<void> > EngineCommandRequest;

typedef boost::shared_ptr< ::hanse_srvs::EngineCommandRequest> EngineCommandRequestPtr;
typedef boost::shared_ptr< ::hanse_srvs::EngineCommandRequest const> EngineCommandRequestConstPtr;


template <class ContainerAllocator>
struct EngineCommandResponse_ {
  typedef EngineCommandResponse_<ContainerAllocator> Type;

  EngineCommandResponse_()
  {
  }

  EngineCommandResponse_(const ContainerAllocator& _alloc)
  {
  }


  typedef boost::shared_ptr< ::hanse_srvs::EngineCommandResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::hanse_srvs::EngineCommandResponse_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct EngineCommandResponse
typedef  ::hanse_srvs::EngineCommandResponse_<std::allocator<void> > EngineCommandResponse;

typedef boost::shared_ptr< ::hanse_srvs::EngineCommandResponse> EngineCommandResponsePtr;
typedef boost::shared_ptr< ::hanse_srvs::EngineCommandResponse const> EngineCommandResponseConstPtr;

struct EngineCommand
{

typedef EngineCommandRequest Request;
typedef EngineCommandResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct EngineCommand
} // namespace hanse_srvs

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::hanse_srvs::EngineCommandRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::hanse_srvs::EngineCommandRequest_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::hanse_srvs::EngineCommandRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "c82b14de8f0a636eab798dc4793bf104";
  }

  static const char* value(const  ::hanse_srvs::EngineCommandRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xc82b14de8f0a636eULL;
  static const uint64_t static_value2 = 0xab798dc4793bf104ULL;
};

template<class ContainerAllocator>
struct DataType< ::hanse_srvs::EngineCommandRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "hanse_srvs/EngineCommandRequest";
  }

  static const char* value(const  ::hanse_srvs::EngineCommandRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::hanse_srvs::EngineCommandRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "bool enableDepthPid\n\
bool enableOrientationPid\n\
bool enableMotors\n\
bool resetZeroPressure\n\
bool setEmergencyStop\n\
\n\
";
  }

  static const char* value(const  ::hanse_srvs::EngineCommandRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::hanse_srvs::EngineCommandRequest_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::hanse_srvs::EngineCommandResponse_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::hanse_srvs::EngineCommandResponse_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::hanse_srvs::EngineCommandResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const  ::hanse_srvs::EngineCommandResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::hanse_srvs::EngineCommandResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "hanse_srvs/EngineCommandResponse";
  }

  static const char* value(const  ::hanse_srvs::EngineCommandResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::hanse_srvs::EngineCommandResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "\n\
";
  }

  static const char* value(const  ::hanse_srvs::EngineCommandResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::hanse_srvs::EngineCommandResponse_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::hanse_srvs::EngineCommandRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.enableDepthPid);
    stream.next(m.enableOrientationPid);
    stream.next(m.enableMotors);
    stream.next(m.resetZeroPressure);
    stream.next(m.setEmergencyStop);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct EngineCommandRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::hanse_srvs::EngineCommandResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct EngineCommandResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<hanse_srvs::EngineCommand> {
  static const char* value() 
  {
    return "c82b14de8f0a636eab798dc4793bf104";
  }

  static const char* value(const hanse_srvs::EngineCommand&) { return value(); } 
};

template<>
struct DataType<hanse_srvs::EngineCommand> {
  static const char* value() 
  {
    return "hanse_srvs/EngineCommand";
  }

  static const char* value(const hanse_srvs::EngineCommand&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<hanse_srvs::EngineCommandRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "c82b14de8f0a636eab798dc4793bf104";
  }

  static const char* value(const hanse_srvs::EngineCommandRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<hanse_srvs::EngineCommandRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "hanse_srvs/EngineCommand";
  }

  static const char* value(const hanse_srvs::EngineCommandRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<hanse_srvs::EngineCommandResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "c82b14de8f0a636eab798dc4793bf104";
  }

  static const char* value(const hanse_srvs::EngineCommandResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<hanse_srvs::EngineCommandResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "hanse_srvs/EngineCommand";
  }

  static const char* value(const hanse_srvs::EngineCommandResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // HANSE_SRVS_SERVICE_ENGINECOMMAND_H
