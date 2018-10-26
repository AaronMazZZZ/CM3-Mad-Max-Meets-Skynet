// Generated by gencpp from file trajectory_tracking/TrajectoryPoint.msg
// DO NOT EDIT!


#ifndef TRAJECTORY_TRACKING_MESSAGE_TRAJECTORYPOINT_H
#define TRAJECTORY_TRACKING_MESSAGE_TRAJECTORYPOINT_H

#include <ros/service_traits.h>


#include <trajectory_tracking/TrajectoryPointRequest.h>
#include <trajectory_tracking/TrajectoryPointResponse.h>


namespace trajectory_tracking
{

struct TrajectoryPoint
{

typedef TrajectoryPointRequest Request;
typedef TrajectoryPointResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct TrajectoryPoint
} // namespace trajectory_tracking


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::trajectory_tracking::TrajectoryPoint > {
  static const char* value()
  {
    return "cd4edd24fcfc9c8a5ef4de64d28e7e2c";
  }

  static const char* value(const ::trajectory_tracking::TrajectoryPoint&) { return value(); }
};

template<>
struct DataType< ::trajectory_tracking::TrajectoryPoint > {
  static const char* value()
  {
    return "trajectory_tracking/TrajectoryPoint";
  }

  static const char* value(const ::trajectory_tracking::TrajectoryPoint&) { return value(); }
};


// service_traits::MD5Sum< ::trajectory_tracking::TrajectoryPointRequest> should match 
// service_traits::MD5Sum< ::trajectory_tracking::TrajectoryPoint > 
template<>
struct MD5Sum< ::trajectory_tracking::TrajectoryPointRequest>
{
  static const char* value()
  {
    return MD5Sum< ::trajectory_tracking::TrajectoryPoint >::value();
  }
  static const char* value(const ::trajectory_tracking::TrajectoryPointRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::trajectory_tracking::TrajectoryPointRequest> should match 
// service_traits::DataType< ::trajectory_tracking::TrajectoryPoint > 
template<>
struct DataType< ::trajectory_tracking::TrajectoryPointRequest>
{
  static const char* value()
  {
    return DataType< ::trajectory_tracking::TrajectoryPoint >::value();
  }
  static const char* value(const ::trajectory_tracking::TrajectoryPointRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::trajectory_tracking::TrajectoryPointResponse> should match 
// service_traits::MD5Sum< ::trajectory_tracking::TrajectoryPoint > 
template<>
struct MD5Sum< ::trajectory_tracking::TrajectoryPointResponse>
{
  static const char* value()
  {
    return MD5Sum< ::trajectory_tracking::TrajectoryPoint >::value();
  }
  static const char* value(const ::trajectory_tracking::TrajectoryPointResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::trajectory_tracking::TrajectoryPointResponse> should match 
// service_traits::DataType< ::trajectory_tracking::TrajectoryPoint > 
template<>
struct DataType< ::trajectory_tracking::TrajectoryPointResponse>
{
  static const char* value()
  {
    return DataType< ::trajectory_tracking::TrajectoryPoint >::value();
  }
  static const char* value(const ::trajectory_tracking::TrajectoryPointResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // TRAJECTORY_TRACKING_MESSAGE_TRAJECTORYPOINT_H
