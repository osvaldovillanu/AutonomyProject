// Generated by gencpp from file simple_car_model/VehicleState.msg
// DO NOT EDIT!


#ifndef SIMPLE_CAR_MODEL_MESSAGE_VEHICLESTATE_H
#define SIMPLE_CAR_MODEL_MESSAGE_VEHICLESTATE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3.h>

namespace simple_car_model
{
template <class ContainerAllocator>
struct VehicleState_
{
  typedef VehicleState_<ContainerAllocator> Type;

  VehicleState_()
    : header()
    , position()
    , vel()
    , accel()
    , steering_angle(0.0)
    , vehicle_angle(0.0)
    , vehicle_width(0.0)
    , vehicle_length(0.0)  {
    }
  VehicleState_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , position(_alloc)
    , vel(_alloc)
    , accel(_alloc)
    , steering_angle(0.0)
    , vehicle_angle(0.0)
    , vehicle_width(0.0)
    , vehicle_length(0.0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef  ::geometry_msgs::Vector3_<ContainerAllocator>  _position_type;
  _position_type position;

   typedef  ::geometry_msgs::Vector3_<ContainerAllocator>  _vel_type;
  _vel_type vel;

   typedef  ::geometry_msgs::Vector3_<ContainerAllocator>  _accel_type;
  _accel_type accel;

   typedef float _steering_angle_type;
  _steering_angle_type steering_angle;

   typedef float _vehicle_angle_type;
  _vehicle_angle_type vehicle_angle;

   typedef float _vehicle_width_type;
  _vehicle_width_type vehicle_width;

   typedef float _vehicle_length_type;
  _vehicle_length_type vehicle_length;





  typedef boost::shared_ptr< ::simple_car_model::VehicleState_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::simple_car_model::VehicleState_<ContainerAllocator> const> ConstPtr;

}; // struct VehicleState_

typedef ::simple_car_model::VehicleState_<std::allocator<void> > VehicleState;

typedef boost::shared_ptr< ::simple_car_model::VehicleState > VehicleStatePtr;
typedef boost::shared_ptr< ::simple_car_model::VehicleState const> VehicleStateConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::simple_car_model::VehicleState_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::simple_car_model::VehicleState_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace simple_car_model

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'geometry_msgs': ['/opt/ros/melodic/share/geometry_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/melodic/share/std_msgs/cmake/../msg'], 'simple_car_model': ['/home/suli/git/AutonomyProject/ros/src/simple_car_model/../../msgs/']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::simple_car_model::VehicleState_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::simple_car_model::VehicleState_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::simple_car_model::VehicleState_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::simple_car_model::VehicleState_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::simple_car_model::VehicleState_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::simple_car_model::VehicleState_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::simple_car_model::VehicleState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ee5f58c0c027a7069a3e20bf33a427ae";
  }

  static const char* value(const ::simple_car_model::VehicleState_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xee5f58c0c027a706ULL;
  static const uint64_t static_value2 = 0x9a3e20bf33a427aeULL;
};

template<class ContainerAllocator>
struct DataType< ::simple_car_model::VehicleState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "simple_car_model/VehicleState";
  }

  static const char* value(const ::simple_car_model::VehicleState_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::simple_car_model::VehicleState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n"
"geometry_msgs/Vector3 position\n"
"geometry_msgs/Vector3 vel\n"
"geometry_msgs/Vector3 accel\n"
"float32 steering_angle\n"
"float32 vehicle_angle\n"
"float32 vehicle_width\n"
"float32 vehicle_length\n"
"================================================================================\n"
"MSG: std_msgs/Header\n"
"# Standard metadata for higher-level stamped data types.\n"
"# This is generally used to communicate timestamped data \n"
"# in a particular coordinate frame.\n"
"# \n"
"# sequence ID: consecutively increasing ID \n"
"uint32 seq\n"
"#Two-integer timestamp that is expressed as:\n"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
"# time-handling sugar is provided by the client library\n"
"time stamp\n"
"#Frame this data is associated with\n"
"string frame_id\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Vector3\n"
"# This represents a vector in free space. \n"
"# It is only meant to represent a direction. Therefore, it does not\n"
"# make sense to apply a translation to it (e.g., when applying a \n"
"# generic rigid transformation to a Vector3, tf2 will only apply the\n"
"# rotation). If you want your data to be translatable too, use the\n"
"# geometry_msgs/Point message instead.\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
;
  }

  static const char* value(const ::simple_car_model::VehicleState_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::simple_car_model::VehicleState_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.position);
      stream.next(m.vel);
      stream.next(m.accel);
      stream.next(m.steering_angle);
      stream.next(m.vehicle_angle);
      stream.next(m.vehicle_width);
      stream.next(m.vehicle_length);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct VehicleState_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::simple_car_model::VehicleState_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::simple_car_model::VehicleState_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "position: ";
    s << std::endl;
    Printer< ::geometry_msgs::Vector3_<ContainerAllocator> >::stream(s, indent + "  ", v.position);
    s << indent << "vel: ";
    s << std::endl;
    Printer< ::geometry_msgs::Vector3_<ContainerAllocator> >::stream(s, indent + "  ", v.vel);
    s << indent << "accel: ";
    s << std::endl;
    Printer< ::geometry_msgs::Vector3_<ContainerAllocator> >::stream(s, indent + "  ", v.accel);
    s << indent << "steering_angle: ";
    Printer<float>::stream(s, indent + "  ", v.steering_angle);
    s << indent << "vehicle_angle: ";
    Printer<float>::stream(s, indent + "  ", v.vehicle_angle);
    s << indent << "vehicle_width: ";
    Printer<float>::stream(s, indent + "  ", v.vehicle_width);
    s << indent << "vehicle_length: ";
    Printer<float>::stream(s, indent + "  ", v.vehicle_length);
  }
};

} // namespace message_operations
} // namespace ros

#endif // SIMPLE_CAR_MODEL_MESSAGE_VEHICLESTATE_H
