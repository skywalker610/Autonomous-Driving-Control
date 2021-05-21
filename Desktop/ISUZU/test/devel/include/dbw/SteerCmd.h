// Generated by gencpp from file dbw/SteerCmd.msg
// DO NOT EDIT!


#ifndef DBW_MESSAGE_STEERCMD_H
#define DBW_MESSAGE_STEERCMD_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace dbw
{
template <class ContainerAllocator>
struct SteerCmd_
{
  typedef SteerCmd_<ContainerAllocator> Type;

  SteerCmd_()
    : steering_wheel_angle_cmd(0.0)
    , steering_wheel_angle_velocity(0.0)
    , steering_wheel_torque_cmd(0.0)
    , cmd_type(0)
    , enable(false)
    , clear(false)
    , ignore(false)
    , quiet(false)
    , count(0)  {
    }
  SteerCmd_(const ContainerAllocator& _alloc)
    : steering_wheel_angle_cmd(0.0)
    , steering_wheel_angle_velocity(0.0)
    , steering_wheel_torque_cmd(0.0)
    , cmd_type(0)
    , enable(false)
    , clear(false)
    , ignore(false)
    , quiet(false)
    , count(0)  {
  (void)_alloc;
    }



   typedef float _steering_wheel_angle_cmd_type;
  _steering_wheel_angle_cmd_type steering_wheel_angle_cmd;

   typedef float _steering_wheel_angle_velocity_type;
  _steering_wheel_angle_velocity_type steering_wheel_angle_velocity;

   typedef float _steering_wheel_torque_cmd_type;
  _steering_wheel_torque_cmd_type steering_wheel_torque_cmd;

   typedef uint8_t _cmd_type_type;
  _cmd_type_type cmd_type;

   typedef uint8_t _enable_type;
  _enable_type enable;

   typedef uint8_t _clear_type;
  _clear_type clear;

   typedef uint8_t _ignore_type;
  _ignore_type ignore;

   typedef uint8_t _quiet_type;
  _quiet_type quiet;

   typedef uint8_t _count_type;
  _count_type count;



// reducing the odds to have name collisions with Windows.h 
#if defined(_WIN32) && defined(CMD_ANGLE)
  #undef CMD_ANGLE
#endif
#if defined(_WIN32) && defined(CMD_TORQUE)
  #undef CMD_TORQUE
#endif
#if defined(_WIN32) && defined(ANGLE_MAX)
  #undef ANGLE_MAX
#endif
#if defined(_WIN32) && defined(VELOCITY_MAX)
  #undef VELOCITY_MAX
#endif
#if defined(_WIN32) && defined(TORQUE_MAX)
  #undef TORQUE_MAX
#endif

  enum {
    CMD_ANGLE = 0u,
    CMD_TORQUE = 1u,
  };

  static const float ANGLE_MAX;
  static const float VELOCITY_MAX;
  static const float TORQUE_MAX;

  typedef boost::shared_ptr< ::dbw::SteerCmd_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::dbw::SteerCmd_<ContainerAllocator> const> ConstPtr;

}; // struct SteerCmd_

typedef ::dbw::SteerCmd_<std::allocator<void> > SteerCmd;

typedef boost::shared_ptr< ::dbw::SteerCmd > SteerCmdPtr;
typedef boost::shared_ptr< ::dbw::SteerCmd const> SteerCmdConstPtr;

// constants requiring out of line definition

   

   

   
   template<typename ContainerAllocator> const float
      SteerCmd_<ContainerAllocator>::ANGLE_MAX =
        
          9.6
        
        ;
   

   
   template<typename ContainerAllocator> const float
      SteerCmd_<ContainerAllocator>::VELOCITY_MAX =
        
          17.5
        
        ;
   

   
   template<typename ContainerAllocator> const float
      SteerCmd_<ContainerAllocator>::TORQUE_MAX =
        
          8.0
        
        ;
   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::dbw::SteerCmd_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::dbw::SteerCmd_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::dbw::SteerCmd_<ContainerAllocator1> & lhs, const ::dbw::SteerCmd_<ContainerAllocator2> & rhs)
{
  return lhs.steering_wheel_angle_cmd == rhs.steering_wheel_angle_cmd &&
    lhs.steering_wheel_angle_velocity == rhs.steering_wheel_angle_velocity &&
    lhs.steering_wheel_torque_cmd == rhs.steering_wheel_torque_cmd &&
    lhs.cmd_type == rhs.cmd_type &&
    lhs.enable == rhs.enable &&
    lhs.clear == rhs.clear &&
    lhs.ignore == rhs.ignore &&
    lhs.quiet == rhs.quiet &&
    lhs.count == rhs.count;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::dbw::SteerCmd_<ContainerAllocator1> & lhs, const ::dbw::SteerCmd_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace dbw

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::dbw::SteerCmd_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::dbw::SteerCmd_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::dbw::SteerCmd_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::dbw::SteerCmd_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dbw::SteerCmd_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dbw::SteerCmd_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::dbw::SteerCmd_<ContainerAllocator> >
{
  static const char* value()
  {
    return "076be04e947ae855b3617e6190e078b9";
  }

  static const char* value(const ::dbw::SteerCmd_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x076be04e947ae855ULL;
  static const uint64_t static_value2 = 0xb3617e6190e078b9ULL;
};

template<class ContainerAllocator>
struct DataType< ::dbw::SteerCmd_<ContainerAllocator> >
{
  static const char* value()
  {
    return "dbw/SteerCmd";
  }

  static const char* value(const ::dbw::SteerCmd_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::dbw::SteerCmd_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# Steering Wheel\n"
"float32 steering_wheel_angle_cmd        # rad, range -9.6 to 9.6\n"
"float32 steering_wheel_angle_velocity   # rad/s, range 0 to 17.5, 0 = maximum\n"
"float32 steering_wheel_torque_cmd       # Nm, range -8.0 to 8.0\n"
"uint8 cmd_type\n"
"\n"
"# Enable\n"
"bool enable\n"
"\n"
"# Clear driver overrides\n"
"bool clear\n"
"\n"
"# Ignore driver overrides\n"
"bool ignore\n"
"\n"
"# Disable the driver override audible warning\n"
"bool quiet\n"
"\n"
"# Watchdog counter (optional)\n"
"uint8 count\n"
"\n"
"# Command types\n"
"uint8 CMD_ANGLE=0\n"
"uint8 CMD_TORQUE=1\n"
"\n"
"# Maximum values\n"
"float32 ANGLE_MAX=9.6     # rad, maximum angle\n"
"float32 VELOCITY_MAX=17.5 # rad/s, maximum velocity\n"
"float32 TORQUE_MAX=8.0    # Nm, maximum torque\n"
"\n"
;
  }

  static const char* value(const ::dbw::SteerCmd_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::dbw::SteerCmd_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.steering_wheel_angle_cmd);
      stream.next(m.steering_wheel_angle_velocity);
      stream.next(m.steering_wheel_torque_cmd);
      stream.next(m.cmd_type);
      stream.next(m.enable);
      stream.next(m.clear);
      stream.next(m.ignore);
      stream.next(m.quiet);
      stream.next(m.count);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SteerCmd_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::dbw::SteerCmd_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::dbw::SteerCmd_<ContainerAllocator>& v)
  {
    s << indent << "steering_wheel_angle_cmd: ";
    Printer<float>::stream(s, indent + "  ", v.steering_wheel_angle_cmd);
    s << indent << "steering_wheel_angle_velocity: ";
    Printer<float>::stream(s, indent + "  ", v.steering_wheel_angle_velocity);
    s << indent << "steering_wheel_torque_cmd: ";
    Printer<float>::stream(s, indent + "  ", v.steering_wheel_torque_cmd);
    s << indent << "cmd_type: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.cmd_type);
    s << indent << "enable: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.enable);
    s << indent << "clear: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.clear);
    s << indent << "ignore: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.ignore);
    s << indent << "quiet: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.quiet);
    s << indent << "count: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.count);
  }
};

} // namespace message_operations
} // namespace ros

#endif // DBW_MESSAGE_STEERCMD_H