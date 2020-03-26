// Generated by gencpp from file simple_car_model/VehicleMoveCommand.msg
// DO NOT EDIT!


#ifndef SIMPLE_CAR_MODEL_MESSAGE_VEHICLEMOVECOMMAND_H
#define SIMPLE_CAR_MODEL_MESSAGE_VEHICLEMOVECOMMAND_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace simple_car_model
{
template <class ContainerAllocator>
struct VehicleMoveCommand_
{
  typedef VehicleMoveCommand_<ContainerAllocator> Type;

  VehicleMoveCommand_()
    : header()
    , linear_vel(0.0)
    , steering_angle_vel(0.0)  {
    }
  VehicleMoveCommand_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , linear_vel(0.0)
    , steering_angle_vel(0.0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef float _linear_vel_type;
  _linear_vel_type linear_vel;

   typedef float _steering_angle_vel_type;
  _steering_angle_vel_type steering_angle_vel;




  typedef boost::shared_ptr< ::simple_car_model::VehicleMoveCommand_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::simple_car_model::VehicleMoveCommand_<ContainerAllocator> const> ConstPtr;

}; // struct VehicleMoveCommand_

typedef ::simple_car_model::VehicleMoveCommand_<std::allocator<void> > VehicleMoveCommand;

typedef boost::shared_ptr< ::simple_car_model::VehicleMoveCommand > VehicleMoveCommandPtr;
typedef boost::shared_ptr< ::simple_car_model::VehicleMoveCommand const> VehicleMoveCommandConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::simple_car_model::VehicleMoveCommand_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::simple_car_model::VehicleMoveCommand_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace simple_car_model

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'geometry_msgs': ['/opt/ros/indigo/share/geometry_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg'], 'simple_car_model': ['/home/oavillanueva/AutonomyProject/ros/src/simple_car_model/../../msgs/']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::simple_car_model::VehicleMoveCommand_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::simple_car_model::VehicleMoveCommand_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::simple_car_model::VehicleMoveCommand_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::simple_car_model::VehicleMoveCommand_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::simple_car_model::VehicleMoveCommand_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::simple_car_model::VehicleMoveCommand_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::simple_car_model::VehicleMoveCommand_<ContainerAllocator> >
{
  static const char* value()
  {
    return "10971e0348cab38ff52d6137bdf2a0a6";
  }

  static const char* value(const ::simple_car_model::VehicleMoveCommand_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x10971e0348cab38fULL;
  static const uint64_t static_value2 = 0xf52d6137bdf2a0a6ULL;
};

template<class ContainerAllocator>
struct DataType< ::simple_car_model::VehicleMoveCommand_<ContainerAllocator> >
{
  static const char* value()
  {
    return "simple_car_model/VehicleMoveCommand";
  }

  static const char* value(const ::simple_car_model::VehicleMoveCommand_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::simple_car_model::VehicleMoveCommand_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n\
float32 linear_vel\n\
float32 steering_angle_vel\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n\
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
";
  }

  static const char* value(const ::simple_car_model::VehicleMoveCommand_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::simple_car_model::VehicleMoveCommand_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.linear_vel);
      stream.next(m.steering_angle_vel);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct VehicleMoveCommand_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::simple_car_model::VehicleMoveCommand_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::simple_car_model::VehicleMoveCommand_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "linear_vel: ";
    Printer<float>::stream(s, indent + "  ", v.linear_vel);
    s << indent << "steering_angle_vel: ";
    Printer<float>::stream(s, indent + "  ", v.steering_angle_vel);
  }
};

} // namespace message_operations
} // namespace ros

#endif // SIMPLE_CAR_MODEL_MESSAGE_VEHICLEMOVECOMMAND_H
