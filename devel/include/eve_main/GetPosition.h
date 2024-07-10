// Generated by gencpp from file eve_main/GetPosition.msg
// DO NOT EDIT!


#ifndef EVE_MAIN_MESSAGE_GETPOSITION_H
#define EVE_MAIN_MESSAGE_GETPOSITION_H

#include <ros/service_traits.h>


#include <eve_main/GetPositionRequest.h>
#include <eve_main/GetPositionResponse.h>


namespace eve_main
{

struct GetPosition
{

typedef GetPositionRequest Request;
typedef GetPositionResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct GetPosition
} // namespace eve_main


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::eve_main::GetPosition > {
  static const char* value()
  {
    return "f98d79514754ade5731789b7227e61f3";
  }

  static const char* value(const ::eve_main::GetPosition&) { return value(); }
};

template<>
struct DataType< ::eve_main::GetPosition > {
  static const char* value()
  {
    return "eve_main/GetPosition";
  }

  static const char* value(const ::eve_main::GetPosition&) { return value(); }
};


// service_traits::MD5Sum< ::eve_main::GetPositionRequest> should match
// service_traits::MD5Sum< ::eve_main::GetPosition >
template<>
struct MD5Sum< ::eve_main::GetPositionRequest>
{
  static const char* value()
  {
    return MD5Sum< ::eve_main::GetPosition >::value();
  }
  static const char* value(const ::eve_main::GetPositionRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::eve_main::GetPositionRequest> should match
// service_traits::DataType< ::eve_main::GetPosition >
template<>
struct DataType< ::eve_main::GetPositionRequest>
{
  static const char* value()
  {
    return DataType< ::eve_main::GetPosition >::value();
  }
  static const char* value(const ::eve_main::GetPositionRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::eve_main::GetPositionResponse> should match
// service_traits::MD5Sum< ::eve_main::GetPosition >
template<>
struct MD5Sum< ::eve_main::GetPositionResponse>
{
  static const char* value()
  {
    return MD5Sum< ::eve_main::GetPosition >::value();
  }
  static const char* value(const ::eve_main::GetPositionResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::eve_main::GetPositionResponse> should match
// service_traits::DataType< ::eve_main::GetPosition >
template<>
struct DataType< ::eve_main::GetPositionResponse>
{
  static const char* value()
  {
    return DataType< ::eve_main::GetPosition >::value();
  }
  static const char* value(const ::eve_main::GetPositionResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // EVE_MAIN_MESSAGE_GETPOSITION_H