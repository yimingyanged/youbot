/* Auto-generated by genmsg_cpp for file /media/sf_youbot_edinburgh/workspaces/andrew/src/cyphy-vis-slam/image_cache/srv/GetNeighbours.srv */
#ifndef IMAGE_CACHE_SERVICE_GETNEIGHBOURS_H
#define IMAGE_CACHE_SERVICE_GETNEIGHBOURS_H
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




namespace image_cache
{
template <class ContainerAllocator>
struct GetNeighboursRequest_ {
  typedef GetNeighboursRequest_<ContainerAllocator> Type;

  GetNeighboursRequest_()
  : nodeID(0)
  {
  }

  GetNeighboursRequest_(const ContainerAllocator& _alloc)
  : nodeID(0)
  {
  }

  typedef int32_t _nodeID_type;
  int32_t nodeID;


  typedef boost::shared_ptr< ::image_cache::GetNeighboursRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::image_cache::GetNeighboursRequest_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct GetNeighboursRequest
typedef  ::image_cache::GetNeighboursRequest_<std::allocator<void> > GetNeighboursRequest;

typedef boost::shared_ptr< ::image_cache::GetNeighboursRequest> GetNeighboursRequestPtr;
typedef boost::shared_ptr< ::image_cache::GetNeighboursRequest const> GetNeighboursRequestConstPtr;



template <class ContainerAllocator>
struct GetNeighboursResponse_ {
  typedef GetNeighboursResponse_<ContainerAllocator> Type;

  GetNeighboursResponse_()
  : node1(0)
  , node2(0)
  , frac(0.0)
  , inter_time()
  {
  }

  GetNeighboursResponse_(const ContainerAllocator& _alloc)
  : node1(0)
  , node2(0)
  , frac(0.0)
  , inter_time()
  {
  }

  typedef int32_t _node1_type;
  int32_t node1;

  typedef int32_t _node2_type;
  int32_t node2;

  typedef float _frac_type;
  float frac;

  typedef ros::Time _inter_time_type;
  ros::Time inter_time;


  typedef boost::shared_ptr< ::image_cache::GetNeighboursResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::image_cache::GetNeighboursResponse_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct GetNeighboursResponse
typedef  ::image_cache::GetNeighboursResponse_<std::allocator<void> > GetNeighboursResponse;

typedef boost::shared_ptr< ::image_cache::GetNeighboursResponse> GetNeighboursResponsePtr;
typedef boost::shared_ptr< ::image_cache::GetNeighboursResponse const> GetNeighboursResponseConstPtr;


struct GetNeighbours
{

typedef GetNeighboursRequest Request;
typedef GetNeighboursResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct GetNeighbours
} // namespace image_cache

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::image_cache::GetNeighboursRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::image_cache::GetNeighboursRequest_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::image_cache::GetNeighboursRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "83f1d0e77402d46c26211c4c64e0f71c";
  }

  static const char* value(const  ::image_cache::GetNeighboursRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x83f1d0e77402d46cULL;
  static const uint64_t static_value2 = 0x26211c4c64e0f71cULL;
};

template<class ContainerAllocator>
struct DataType< ::image_cache::GetNeighboursRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "image_cache/GetNeighboursRequest";
  }

  static const char* value(const  ::image_cache::GetNeighboursRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::image_cache::GetNeighboursRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "\n\
int32 nodeID\n\
\n\
";
  }

  static const char* value(const  ::image_cache::GetNeighboursRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::image_cache::GetNeighboursRequest_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::image_cache::GetNeighboursResponse_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::image_cache::GetNeighboursResponse_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::image_cache::GetNeighboursResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "15fb40c1365b06d426b1f08def48bff0";
  }

  static const char* value(const  ::image_cache::GetNeighboursResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x15fb40c1365b06d4ULL;
  static const uint64_t static_value2 = 0x26b1f08def48bff0ULL;
};

template<class ContainerAllocator>
struct DataType< ::image_cache::GetNeighboursResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "image_cache/GetNeighboursResponse";
  }

  static const char* value(const  ::image_cache::GetNeighboursResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::image_cache::GetNeighboursResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "int32 node1\n\
int32 node2\n\
float32 frac\n\
time inter_time\n\
\n\
\n\
";
  }

  static const char* value(const  ::image_cache::GetNeighboursResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::image_cache::GetNeighboursResponse_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::image_cache::GetNeighboursRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.nodeID);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct GetNeighboursRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::image_cache::GetNeighboursResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.node1);
    stream.next(m.node2);
    stream.next(m.frac);
    stream.next(m.inter_time);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct GetNeighboursResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<image_cache::GetNeighbours> {
  static const char* value() 
  {
    return "ccd112ae98ddab13d42702fa6352dc06";
  }

  static const char* value(const image_cache::GetNeighbours&) { return value(); } 
};

template<>
struct DataType<image_cache::GetNeighbours> {
  static const char* value() 
  {
    return "image_cache/GetNeighbours";
  }

  static const char* value(const image_cache::GetNeighbours&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<image_cache::GetNeighboursRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "ccd112ae98ddab13d42702fa6352dc06";
  }

  static const char* value(const image_cache::GetNeighboursRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<image_cache::GetNeighboursRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "image_cache/GetNeighbours";
  }

  static const char* value(const image_cache::GetNeighboursRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<image_cache::GetNeighboursResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "ccd112ae98ddab13d42702fa6352dc06";
  }

  static const char* value(const image_cache::GetNeighboursResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<image_cache::GetNeighboursResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "image_cache/GetNeighbours";
  }

  static const char* value(const image_cache::GetNeighboursResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // IMAGE_CACHE_SERVICE_GETNEIGHBOURS_H

