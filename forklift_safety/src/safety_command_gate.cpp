#include "forklift_safety/collision_core.hpp"

#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <rclcpp/rclcpp.hpp>
#include <forklift_msgs/msg/forklift_vehicle_state.hpp>
#include <forklift_msgs/msg/forklift_fault_state.hpp>
#include <forklift_msgs/srv/set_emergency_stop.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav2_msgs/msg/costmap.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <yaml-cpp/yaml.h>

namespace forklift_safety
{
using Scan=sensor_msgs::msg::LaserScan;
using Odom=nav_msgs::msg::Odometry;
using Vehicle=forklift_msgs::msg::ForkliftVehicleState;
using Fault=forklift_msgs::msg::ForkliftFaultState;
using Target=geometry_msgs::msg::PoseStamped;
using Twist=geometry_msgs::msg::Twist;
using Steady=std::chrono::steady_clock;
inline double seconds(const builtin_interfaces::msg::Time & t) {return t.sec+t.nanosec*1e-9;}
inline double yaw(const geometry_msgs::msg::Quaternion & q)
{return std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));}
inline Pose planar(const geometry_msgs::msg::Pose & p) {return {p.position.x,p.position.y,yaw(p.orientation)};}
inline bool finite(Pose p) {return std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.yaw);}
inline bool validCommand(const Command & c)
{
  for (double v : {c.velocity_mps,c.drive_rpm,c.steering_angle_rad,c.accel_time_sec,
    c.decel_time_sec,c.pump_rpm,c.lift_valve_ma,c.lower_valve_ma,c.side_shift_left_valve_ma,
    c.side_shift_right_valve_ma,c.tilt_forward_valve_ma,c.tilt_backward_valve_ma}) {
    if (!std::isfinite(v)) {return false;}
  }
  return true;
}
inline bool steeringOnly(const Command & c)
{
  if (!c.enable || c.brake || direction(c) || c.horn || !validCommand(c)) {return false;}
  for (double v : {c.velocity_mps,c.drive_rpm,c.pump_rpm,c.lift_valve_ma,c.lower_valve_ma,
    c.side_shift_left_valve_ma,c.side_shift_right_valve_ma,c.tilt_forward_valve_ma,c.tilt_backward_valve_ma}) {
    if (std::abs(v)>1e-6) {return false;}
  }
  return true;
}
template<class T> struct Sample
{
  std::shared_ptr<const T> msg;
  double received{};
};
struct Inputs
{
  Sample<Command> raw;
  Sample<Twist> recovery;
  Sample<Vehicle> vehicle;
  Sample<Fault> fault;
  Sample<Odom> odom;
  Sample<Scan> scan;
  std::shared_ptr<const Grid> grid;
  std::shared_ptr<const Target> target;
  bool exemption{}, emergency{}, degraded{};
  double exemption_received{}, fresh_since{-1};
  uint64_t stop_generation{};
};

class SafetyCommandGate : public rclcpp::Node
{
public:
  explicit SafetyCommandGate(const rclcpp::NodeOptions & options=rclcpp::NodeOptions())
  : Node("safety_command_gate",options), buffer_(get_clock()), listener_(buffer_)
  {
    parameters();
    const auto polygon=YAML::Load(s("footprint"));
    if (!polygon.IsSequence() || polygon.size()<3) {throw std::runtime_error("invalid footprint");}
    geometry_.footprint.clear();
    for (auto p : polygon) {
      if (!p.IsSequence() || p.size()!=2) {throw std::runtime_error("invalid footprint point");}
      Point point{p[0].as<double>(),p[1].as<double>()};
      if (!std::isfinite(point.x)||!std::isfinite(point.y)) {throw std::runtime_error("invalid footprint point");}
      geometry_.footprint.push_back(point);
    }
    geometry_.wheel_base=d("wheel_base"); geometry_.pivot_radius=d("pivot_turn_radius");
    geometry_.axle_offset=d("rear_axle_x_offset"); geometry_.pivot_angle=d("pivot_steering_angle_rad");
    geometry_.scan_spacing=d("scan_collision_sample_spacing_m"); geometry_.edge_spacing=d("footprint_sample_spacing");
    geometry_.padding=d("scan_collision_padding_m"); geometry_.reaction=d("dynamic_stop_reaction_time_sec");
    geometry_.deceleration=d("dynamic_stop_brake_deceleration_mps2"); geometry_.clearance=d("dynamic_stop_clearance_m");
    geometry_.pivot_deceleration=d("pivot_brake_deceleration_radps2"); geometry_.pivot_margin=d("pivot_stop_margin_rad");
    geometry_.threshold=threshold_; geometry_.unknown_collision=b("unknown_is_collision");
    geometry_.reverse_escape=b("allow_reverse_collision_escape");
    geometry_.escape_speed=d("reverse_collision_escape_max_speed_mps");
    geometry_.escape_steering=d("reverse_collision_escape_max_steering_angle_rad");
    geometry_.escape_min_x=d("reverse_collision_escape_obstacle_min_x_m");
    const auto self_filter=YAML::Load(s("scan_self_filter_rectangles"));
    if (!self_filter.IsSequence()) {throw std::runtime_error("invalid scan self-filter rectangles");}
    for (const auto & rectangle : self_filter) {
      if (!rectangle.IsSequence() || rectangle.size()!=4) {
        throw std::runtime_error("scan self-filter rectangle must be [min_x,max_x,min_y,max_y]");
      }
      Rectangle filter{rectangle[0].as<double>(), rectangle[1].as<double>(),
        rectangle[2].as<double>(), rectangle[3].as<double>()};
      if (!std::isfinite(filter.min_x) || !std::isfinite(filter.max_x) ||
        !std::isfinite(filter.min_y) || !std::isfinite(filter.max_y) ||
        filter.min_x >= filter.max_x || filter.min_y >= filter.max_y)
      {
        throw std::runtime_error("invalid scan self-filter rectangle");
      }
      geometry_.scan_self_filter_rectangles.push_back(filter);
    }
    inputs_.emergency=b("emergency_stop_active");
    started_=now().seconds();
    pub_=create_publisher<Command>(s("gated_command_topic"),1);
    status_=create_publisher<std_msgs::msg::String>(s("status_topic"),10);
    subscribe<Command>(s("raw_command_topic"),[this](Command::ConstSharedPtr m) {raw(m);});
    subscribe<Twist>(s("recovery_twist_topic"),[this](Twist::ConstSharedPtr m) {
      std::lock_guard<std::mutex> lock(mutex_); inputs_.recovery={m,now().seconds()};
    });
    subscribe<Vehicle>(s("vehicle_state_topic"),[this](Vehicle::ConstSharedPtr m) {
      std::lock_guard<std::mutex> lock(mutex_); inputs_.vehicle={m,now().seconds()};
      if (m->emergency_stopped||m->soft_emergency_stop ||
        (b("require_vehicle_state") && (m->parking_brake||!m->interlock))) {priorityStop();}
    });
    subscribe<Fault>(s("fault_state_topic"),[this](Fault::ConstSharedPtr m) {
      std::lock_guard<std::mutex> lock(mutex_); inputs_.fault={m,now().seconds()};
      if (m->has_fault) {priorityStop();}
    });
    if (s("localization_message_type")=="odometry") {
      subscribe<Odom>(s("localization_topic"),[this](Odom::ConstSharedPtr m) {localization(m);});
    } else if (s("localization_message_type")=="pose_with_covariance_stamped" ||
      s("localization_message_type")=="amcl_pose") {
      subscribe<geometry_msgs::msg::PoseWithCovarianceStamped>(s("localization_topic"),
        [this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr m) {
          auto odom=std::make_shared<Odom>(); odom->header=m->header; odom->pose=m->pose;
          odom->child_frame_id=s("base_frame_id");
          odom->twist.twist.angular.z=std::numeric_limits<double>::quiet_NaN(); localization(odom);
        });
    } else {throw std::runtime_error("unsupported localization_message_type");}
    if (s("costmap_message_type")=="costmap_raw" || s("costmap_message_type")=="nav2_costmap") {
      subscribe<nav2_msgs::msg::Costmap>(s("costmap_topic"),[this](nav2_msgs::msg::Costmap::ConstSharedPtr m) {
        auto grid=std::make_shared<Grid>();
        grid->width=m->metadata.size_x; grid->height=m->metadata.size_y;
        grid->resolution=m->metadata.resolution; grid->origin=planar(m->metadata.origin);
        grid->frame=m->header.frame_id; grid->source=seconds(m->header.stamp);
        grid->data.assign(m->data.begin(),m->data.end()); map(grid);
      });
    } else if (s("costmap_message_type")=="occupancy_grid") {
      subscribe<nav_msgs::msg::OccupancyGrid>(s("costmap_topic"),[this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr m) {
        auto grid=std::make_shared<Grid>(); grid->width=m->info.width; grid->height=m->info.height;
        grid->resolution=m->info.resolution; grid->origin=planar(m->info.origin);
        grid->frame=m->header.frame_id; grid->source=seconds(m->header.stamp);
        grid->data.assign(m->data.begin(),m->data.end()); map(grid);
      });
    } else {throw std::runtime_error("unsupported costmap_message_type");}
    if (b("scan_protection_enabled")) {
      subscribe<Scan>(s("scan_topic"),[this](Scan::ConstSharedPtr m) {scan(m);});
    }
    if (b("pallet_exemption_enabled")) {
      subscribe<Target>(s("pallet_exemption_pose_topic"),[this](Target::ConstSharedPtr m) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (inputs_.target && inputs_.exemption &&
          (inputs_.target->header.frame_id!=m->header.frame_id || inputs_.target->pose!=m->pose)) {priorityStop();}
        inputs_.target=m;
      });
      subscribe<std_msgs::msg::Bool>(s("pallet_exemption_active_topic"),[this](std_msgs::msg::Bool::ConstSharedPtr m) {
        std::lock_guard<std::mutex> lock(mutex_);
        const bool revoked=inputs_.exemption && !m->data;
        inputs_.exemption=m->data;
        inputs_.exemption_received=now().seconds();
        if (revoked) {priorityStop();}
      });
    }
    auto group=create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    groups_.push_back(group);
    emergency_service_=create_service<forklift_msgs::srv::SetEmergencyStop>(
      "/forklift_safety/set_emergency_stop",[this](
        const std::shared_ptr<forklift_msgs::srv::SetEmergencyStop::Request> request,
        std::shared_ptr<forklift_msgs::srv::SetEmergencyStop::Response> response) {
        std::lock_guard<std::mutex> lock(mutex_); inputs_.emergency=request->emergency_stop;
        if (inputs_.emergency) {priorityStop();}
        response->success=true; response->message=inputs_.emergency ?
          "safety gate emergency stop enabled" : "safety gate emergency stop cleared";
      },rmw_qos_profile_services_default,group);
    timer_=create_wall_timer(std::chrono::duration<double>(1/d("control_rate_hz")),[this]() {tick();});
    RCLCPP_INFO(get_logger(),"C++ safety_command_gate ready: latest-only independent odom/scan/costmap; rate=%.1f Hz budget=%.3fs scan_timeout=%.3fs costmap_timeout=%.3fs",
      d("control_rate_hz"),d("collision_compute_budget_sec"),d("scan_timeout_sec"),d("costmap_timeout_sec"));
  }

private:
  void parameters()
  {
    for (auto entry : std::map<std::string,bool>{
      {"enabled",true},{"require_vehicle_state",false},{"require_fault_state",false},
      {"require_localization",false},{"costmap_monitor_enabled",true},{"collision_check_enabled",true},
      {"costmap_collision_check_enabled",true},
      {"unknown_is_collision",true},{"scan_protection_enabled",true},{"scan_require_motion_fov_coverage",true},
      {"pivot_costmap_collision_check_enabled",true},
      {"allow_reverse_collision_escape",true},{"pallet_exemption_enabled",true},{"pallet_exemption_reverse_only",true},
      {"emergency_stop_active",false},{"allow_recovery_twist",true},{"allow_recovery_backoff",true},{"allow_recovery_pivot",true}}) {
      bools_[entry.first]=declare_parameter<bool>(entry.first,entry.second);
    }
    for (auto entry : std::map<std::string,double>{
      {"command_timeout_sec",.5},{"recovery_timeout_sec",.5},{"vehicle_state_timeout_sec",.5},
      {"fault_state_timeout_sec",.5},{"localization_timeout_sec",.5},{"costmap_timeout_sec",.5},
      {"footprint_sample_spacing",.05},{"collision_check_horizon_sec",1.},{"collision_check_time_step_sec",.1},
      {"dynamic_stop_reaction_time_sec",.9},{"dynamic_stop_brake_deceleration_mps2",1.5},{"dynamic_stop_clearance_m",.5},
      {"pivot_brake_deceleration_radps2",.15},{"pivot_stop_margin_rad",.05},{"collision_compute_budget_sec",.15},
      {"scan_timeout_sec",.7},{"scan_high_speed_freshness_timeout_sec",.25},{"scan_degraded_max_speed_mps",1.},
      {"scan_fresh_recovery_duration_sec",1.},{"obstacle_release_clear_sec",.5},{"obstacle_release_steering_change_rad",.15},
      {"scan_required_range_m",8.},{"scan_collision_sample_spacing_m",.05},{"scan_collision_padding_m",.05},
      {"reverse_collision_escape_max_speed_mps",.15},{"reverse_collision_escape_max_steering_angle_rad",.05},
      {"reverse_collision_escape_obstacle_min_x_m",0.},{"pallet_exemption_timeout_sec",.5},
      {"pallet_exemption_length_m",.50},{"pallet_exemption_width_m",1.30},
      {"max_forward_velocity_mps",.45},{"max_reverse_velocity_mps",.15},{"max_recovery_velocity_mps",.10},
      {"max_recovery_angular_velocity_radps",.30},{"max_steering_angle_rad",pi/2},{"max_drive_rpm",2485.},
      {"drive_accel_time_sec",5.},{"drive_decel_time_sec",3.},{"wheel_base",1.4},{"pivot_turn_radius",.6},
      {"rear_axle_x_offset",0.},{"pivot_steering_angle_rad",pi/2},{"control_rate_hz",20.}}) {
      doubles_[entry.first]=declare_parameter<double>(entry.first,entry.second);
      const bool signed_value=entry.first=="rear_axle_x_offset" || entry.first=="reverse_collision_escape_obstacle_min_x_m";
      const bool allow_zero=entry.first=="scan_collision_padding_m" || entry.first=="dynamic_stop_clearance_m";
      const double v=doubles_[entry.first];
      if (!std::isfinite(v) || (!signed_value && (allow_zero ? v<0 : v<=0))) {
        throw std::runtime_error("invalid parameter: "+entry.first);
      }
    }
    for (auto entry : std::map<std::string,std::string>{
      {"raw_command_topic","/forklift/control_cmd_raw"},{"gated_command_topic","/forklift/control_cmd"},
      {"recovery_twist_topic","/cmd_vel"},{"vehicle_state_topic","/forklift/vehicle_state"},
      {"fault_state_topic","/forklift/fault_state"},{"localization_topic","/odom"},
      {"localization_message_type","odometry"},{"base_frame_id","base_link"},
      {"costmap_topic","/local_costmap/costmap_raw"},{"costmap_message_type","costmap_raw"},
      {"status_topic","/forklift/safety_gate/status"},{"scan_topic","/scan"},
      {"scan_self_filter_rectangles","[]"},
      {"pallet_exemption_pose_topic","/forklift/pallet_approach/exemption_pose"},
      {"pallet_exemption_active_topic","/forklift/pallet_approach/exemption_active"},
      {"footprint","[[1.709,0.610],[1.709,-0.610],[-1.590,-0.610],[-1.590,0.610]]"}}) {
      strings_[entry.first]=declare_parameter<std::string>(entry.first,entry.second);
    }
    threshold_=declare_parameter<int>("footprint_collision_cost_threshold",253);
    exemption_threshold_=declare_parameter<int>("pallet_exemption_cost_threshold",254);
  }
  double d(const std::string & name) const {return doubles_.at(name);}
  bool b(const std::string & name) const {return bools_.at(name);}
  const std::string & s(const std::string & name) const {return strings_.at(name);}
  template<class T,class F> void subscribe(const std::string & topic,F callback)
  {
    if (topic.empty()) {return;}
    auto group=create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    groups_.push_back(group);
    rclcpp::SubscriptionOptions options; options.callback_group=group;
    subscriptions_.push_back(create_subscription<T>(topic,rclcpp::QoS(1),callback,options));
  }
  Inputs snapshot() {std::lock_guard<std::mutex> lock(mutex_); return inputs_;}
  Command stop() {Command c; c.header.stamp=now(); c.brake=true; return c;}
  void priorityStop() {++inputs_.stop_generation; pub_->publish(stop());}
  void localization(Odom::ConstSharedPtr m)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    // Reject delayed/repeated source samples without refreshing their receipt age.
    if (inputs_.odom.msg && seconds(m->header.stamp)<=seconds(inputs_.odom.msg->header.stamp) &&
      now().seconds()>=inputs_.odom.received) {return;}
    inputs_.odom={m,now().seconds()};
  }
  void map(std::shared_ptr<Grid> grid)
  {
    if (!grid->width || !grid->height) {grid->error="empty dimensions";}
    else if (!std::isfinite(grid->resolution) || grid->resolution<=0) {grid->error="invalid resolution";}
    else if (!finite(grid->origin)) {grid->error="invalid origin";}
    else if (grid->data.size()/grid->width<grid->height) {grid->error="truncated data";}
    std::lock_guard<std::mutex> lock(mutex_); grid->received=now().seconds(); inputs_.grid=grid;
  }
  void scan(Scan::ConstSharedPtr m)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const double t=now().seconds();
    if (sourceAge(seconds(m->header.stamp),t)>d("scan_high_speed_freshness_timeout_sec") ||
      (inputs_.scan.msg && t-inputs_.scan.received>d("scan_high_speed_freshness_timeout_sec"))) {
      inputs_.degraded=true; inputs_.fresh_since=-1;
    } else if (inputs_.degraded) {
      if (inputs_.fresh_since<0) {inputs_.fresh_since=t;}
      else if (t-inputs_.fresh_since>=d("scan_fresh_recovery_duration_sec")) {inputs_.degraded=false; inputs_.fresh_since=-1;}
    }
    inputs_.scan={m,t};
  }
  void raw(Command::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const double t=now().seconds(); inputs_.raw={msg,t};
    const bool expired=sourceAge(seconds(msg->header.stamp),t)>d("command_timeout_sec");
    if (!msg->enable || msg->brake || expired || !validCommand(*msg)) {
      ++inputs_.stop_generation;
      auto out=stop();
      if (!expired && validCommand(*msg) && health(inputs_,t).empty()) {
        if (msg->enable && msg->brake) {out=*msg; out.header.stamp=now();}
        out.forward=out.reverse=false; out.velocity_mps=out.drive_rpm=0;
        out.enable=msg->enable; out.brake=true;
        out.steering_angle_rad=std::clamp(msg->steering_angle_rad,-d("max_steering_angle_rad"),d("max_steering_angle_rad"));
        out.steering_angle_deg=out.steering_angle_rad*180/pi;
      }
      pub_->publish(out);
    }
  }
  std::string mapHealth(const std::shared_ptr<const Grid> & grid,double t) const
  {
    if (!b("costmap_monitor_enabled")) {return "";}
    if (!grid) {return t-started_>d("costmap_timeout_sec") ? "costmap missing" : "";}
    if (!grid->error.empty()) {return "costmap invalid: "+grid->error;}
    return t-grid->received>d("costmap_timeout_sec") || t<grid->received ? "costmap timeout" : "";
  }
  std::string scanHealth(const Sample<Scan> & scan,double t) const
  {
    if (!b("scan_protection_enabled")) {return "";}
    if (!scan.msg) {return "scan missing";}
    if (std::max(t-scan.received,sourceAge(seconds(scan.msg->header.stamp),t))>d("scan_timeout_sec")) {return "scan timeout";}
    if (!std::isfinite(scan.msg->range_max) || scan.msg->range_max<d("scan_required_range_m")) {
      return "scan range_max below required range";
    }
    return "";
  }
  std::string health(const Inputs & in,double t) const
  {
    if (in.emergency) {return "emergency stop";}
    if (!in.vehicle.msg) {if (b("require_vehicle_state")) {return "vehicle state missing";}}
    else {
      const auto & v=*in.vehicle.msg;
      if (b("require_vehicle_state") && t-in.vehicle.received>d("vehicle_state_timeout_sec")) {return "vehicle state timeout";}
      if (v.emergency_stopped||v.soft_emergency_stop) {return "vehicle emergency stop";}
      if (b("require_vehicle_state") && v.parking_brake) {return "parking brake";}
      if (b("require_vehicle_state") && !v.interlock) {return "vehicle interlock open";}
    }
    if (!in.fault.msg) {if (b("require_fault_state")) {return "vehicle fault state missing";}}
    else {
      if (b("require_fault_state") && t-in.fault.received>d("fault_state_timeout_sec")) {return "fault state timeout";}
      if (in.fault.msg->has_fault) {return "vehicle fault";}
    }
    if (b("require_localization") && (!in.odom.msg || t-in.odom.received>d("localization_timeout_sec"))) {return "localization timeout";}
    return mapHealth(in.grid,t);
  }
  Pose lookup(const std::string & target,const std::string & source,const builtin_interfaces::msg::Time & stamp)
  {
    if (source.empty()||target.empty()) {throw std::runtime_error("frame missing");}
    if (source==target) {return {};}
    const auto tr=buffer_.lookupTransform(target,source,rclcpp::Time(stamp)).transform;
    Pose p{tr.translation.x,tr.translation.y,yaw(tr.rotation)};
    if (!finite(p)) {throw std::runtime_error("transform invalid");}
    return p;
  }
  Pose poseInGrid(const Inputs & in)
  {
    if (!in.odom.msg) {throw std::runtime_error("costmap pose missing");}
    const auto & msg=*in.odom.msg;
    const double t=now().seconds();
    if (sourceAge(seconds(msg.header.stamp),t)>d("localization_timeout_sec") ||
      t-in.odom.received>d("localization_timeout_sec")) {
      RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),2000,
        "Costmap pose stale: receive_age=%.4f source_age=%.4f frame=%s child=%s",
        t-in.odom.received,sourceAge(seconds(msg.header.stamp),t),msg.header.frame_id.c_str(),msg.child_frame_id.c_str());
      throw std::runtime_error("costmap pose source stale");
    }
    auto p=planar(msg.pose.pose);
    if (!finite(p)) {throw std::runtime_error("costmap pose invalid");}
    try {
      p=compose(p,lookup(msg.child_frame_id,s("base_frame_id"),msg.header.stamp));
      return compose(lookup(in.grid->frame,msg.header.frame_id,msg.header.stamp),p);
    } catch (const std::exception &) {throw std::runtime_error("costmap pose transform unavailable");}
  }
  std::optional<Zone> zone(const Inputs & in,const Command & cmd,const std::string & frame)
  {
    if (!b("pallet_exemption_enabled") || !in.exemption || !in.target ||
      now().seconds()-in.exemption_received>d("pallet_exemption_timeout_sec") ||
      (b("pallet_exemption_reverse_only") && direction(cmd)!=-1)) {return {};}
    try {
      auto p=compose(lookup(frame,in.target->header.frame_id,builtin_interfaces::msg::Time()),planar(in.target->pose));
      if (!finite(p)) {return {};}
      exemption_used_=true;
      return Zone{p,d("pallet_exemption_length_m")/2,d("pallet_exemption_width_m")/2};
    } catch (const std::exception &) {return {};}
  }
  double protectedSpeed(const Inputs & in,const Command & cmd) const
  {
    const double v=in.vehicle.msg ? std::abs(in.vehicle.msg->velocity_mps) : 0.;
    if (!std::isfinite(v)) {throw std::runtime_error("vehicle speed invalid");}
    return std::max({v,std::abs(cmd.velocity_mps),release_.active ? release_.floor : 0.});
  }
  std::string collision(const Inputs & in,const Command & command)
  {
    if (!b("collision_check_enabled") || !command.enable || command.brake || !direction(command)) {return "";}
    release_.prepare(direction(command),command.steering_angle_rad,d("obstacle_release_steering_change_rad"));
    Command probe=command; probe.velocity_mps=protectedSpeed(in,command);
    const double speed=probe.velocity_mps;
    std::optional<std::vector<Pose>> pivot;
    if (std::abs(probe.steering_angle_rad)>=geometry_.pivot_angle-1e-3) {
      if (!in.odom.msg || sourceAge(seconds(in.odom.msg->header.stamp),now().seconds())>d("localization_timeout_sec")) {
        return "pivot angular feedback stale or unavailable";
      }
      pivot=pivotPoses(std::copysign(speed/geometry_.pivot_radius,probe.steering_angle_rad),
        in.odom.msg->twist.twist.angular.z,geometry_);
    }
    bool escape=false;
    const auto scan_begin=Steady::now();
    if (b("scan_protection_enabled")) {
      const auto failure=scanHealth(in.scan,now().seconds()); if (!failure.empty()) {return failure;}
      const auto & scan=*in.scan.msg;
      Pose offset;
      try {offset=lookup(s("base_frame_id"),scan.header.frame_id,scan.header.stamp);}
      catch (const std::exception &) {return "scan transform unavailable";}
      const double angle=std::atan2(std::sin((direction(probe)>0 ? 0 : pi)-offset.yaw),
        std::cos((direction(probe)>0 ? 0 : pi)-offset.yaw));
      bool covered=false;
      for (double a : {angle,angle-2*pi,angle+2*pi}) {
        covered=covered || (a>=scan.angle_min-1e-6 && a<=scan.angle_max+1e-6);
      }
      if (b("scan_require_motion_fov_coverage") && !covered) {return "scan blind motion direction";}
      if (!std::isfinite(scan.angle_min)||!std::isfinite(scan.angle_increment)) {return "scan invalid angles";}
      std::vector<Point> points; points.reserve(scan.ranges.size());
      double a=scan.angle_min;
      for (double r : scan.ranges) {
        if (std::isfinite(r) && r>=scan.range_min && r<=scan.range_max) {points.push_back(transform({r*std::cos(a),r*std::sin(a)},offset));}
        a+=scan.angle_increment;
      }
      const double horizon=stoppingDistance(speed,geometry_.reaction,geometry_.deceleration,geometry_.clearance)/std::max(speed,1e-6);
      auto poses=pivot ? *pivot : predict(probe,geometry_,horizon,spatialStep(speed,geometry_.scan_spacing,horizon));
      const auto result=scanSweep(points,probe,geometry_,poses,zone(in,probe,s("base_frame_id")));
      scan_elapsed_=std::chrono::duration<double>(Steady::now()-scan_begin).count();
      if (result.blocked) {release_.blocked(speed); return result.reason;}
      escape=result.reason=="scan reverse escape clear";
    }
    const auto map_reason=mapHealth(in.grid,now().seconds()); if (!map_reason.empty()) {return map_reason;}
    if (in.grid && b("costmap_collision_check_enabled") &&
      (!pivot || b("pivot_costmap_collision_check_enabled"))) {
      const auto pose_begin=Steady::now();
      const auto pose=poseInGrid(in);
      pose_elapsed_=std::chrono::duration<double>(Steady::now()-pose_begin).count();
      const double horizon=speed<=1e-6 ? d("collision_check_horizon_sec") :
        std::max(d("collision_check_time_step_sec"),stoppingDistance(speed,geometry_.reaction,geometry_.deceleration,0)/speed);
      auto poses=pivot ? *pivot : predict(probe,geometry_,horizon,spatialStep(speed,geometry_.scan_spacing,horizon));
      for (auto & p : poses) {p=compose(pose,p);}
      auto config=geometry_;
      const auto exemption=zone(in,probe,in.grid->frame);
      if (exemption) {config.threshold=std::max(config.threshold,exemption_threshold_);}
      const auto grid_begin=Steady::now();
      const auto result=gridSweep(*in.grid,config,poses,exemption,escape);
      grid_elapsed_=std::chrono::duration<double>(Steady::now()-grid_begin).count();
      if (result.blocked) {
        if (obstacleReason(result.reason)) {release_.blocked(speed);}
        if (result.reason.rfind("costmap coverage",0)==0) {
          RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),2000,
            "%s frame=%s size=%zux%zu resolution=%.3f origin=(%.3f,%.3f,%.3f) pose=(%.3f,%.3f,%.3f) receive_age=%.3f source_age=%.3f pose_age=%.3f speed=%.3f horizon=%.3f",
            result.reason.c_str(),in.grid->frame.c_str(),in.grid->width,in.grid->height,in.grid->resolution,
            in.grid->origin.x,in.grid->origin.y,in.grid->origin.yaw,pose.x,pose.y,pose.yaw,
            now().seconds()-in.grid->received,sourceAge(in.grid->source,now().seconds()),
            sourceAge(seconds(in.odom.msg->header.stamp),now().seconds()),speed,horizon);
        }
        return result.reason;
      }
    }
    if (!release_.clear(now().seconds(),b("scan_protection_enabled") ? seconds(in.scan.msg->header.stamp) : -1.,
      in.grid && b("costmap_collision_check_enabled") ? in.grid->received : -1.,d("obstacle_release_clear_sec"))) {return "obstacle release waiting for stable clearance";}
    return "";
  }
  bool cap(Command & cmd,double limit)
  {
    const double v=std::abs(cmd.velocity_mps);
    if (v<=limit+1e-9) {return false;}
    cmd.velocity_mps=limit; cmd.drive_rpm=std::abs(cmd.drive_rpm)*limit/v; return true;
  }
  Command clampCommand(Command cmd)
  {
    cmd.velocity_mps=std::abs(cmd.velocity_mps); cmd.drive_rpm=std::abs(cmd.drive_rpm);
    cap(cmd,d(direction(cmd)<0 ? "max_reverse_velocity_mps" : "max_forward_velocity_mps"));
    cmd.drive_rpm=std::min(cmd.drive_rpm,d("max_drive_rpm"));
    cmd.steering_angle_rad=std::clamp(cmd.steering_angle_rad,-d("max_steering_angle_rad"),d("max_steering_angle_rad"));
    cmd.steering_angle_deg=cmd.steering_angle_rad*180/pi;
    if (cmd.accel_time_sec<=0) {cmd.accel_time_sec=d("drive_accel_time_sec");}
    if (cmd.decel_time_sec<=0) {cmd.decel_time_sec=d("drive_decel_time_sec");}
    cmd.header.stamp=now(); return cmd;
  }
  bool degraded(const Inputs & in,double t) const
  {
    return b("scan_protection_enabled") && (in.degraded || !in.scan.msg ||
      std::max(t-in.scan.received,sourceAge(seconds(in.scan.msg->header.stamp),t))>d("scan_high_speed_freshness_timeout_sec"));
  }
  Command recovery(const Twist & twist,std::string & reason)
  {
    const double v=std::clamp(twist.linear.x,-d("max_recovery_velocity_mps"),d("max_recovery_velocity_mps"));
    const double w=std::clamp(twist.angular.z,-d("max_recovery_angular_velocity_radps"),d("max_recovery_angular_velocity_radps"));
    if (!std::isfinite(v)||!std::isfinite(w)) {reason="invalid recovery command"; return stop();}
    if (std::abs(v)<=1e-6 && std::abs(w)<=1e-6) {reason="recovery wait"; return stop();}
    Command c; c.enable=true; c.header.stamp=now();
    if (std::abs(v)>1e-6) {
      if (v<0 && !b("allow_recovery_backoff")) {reason="recovery backoff disabled"; return stop();}
      c.forward=v>0; c.reverse=v<0; c.velocity_mps=std::abs(v);
      if (std::abs(w)>1e-6) {c.steering_angle_rad=std::clamp(std::atan2(w*geometry_.wheel_base,v),-geometry_.pivot_angle,geometry_.pivot_angle);}
      reason=v>0 ? "recovery forward" : "recovery backoff";
    } else {
      if (!b("allow_recovery_pivot")) {reason="recovery pivot disabled"; return stop();}
      c.forward=true; c.velocity_mps=std::min(d("max_recovery_velocity_mps"),std::abs(w)*geometry_.pivot_radius);
      c.steering_angle_rad=std::copysign(geometry_.pivot_angle,w); reason="recovery pivot";
    }
    return clampCommand(c);
  }
  std::pair<Command,std::string> evaluate(const Inputs & in)
  {
    const double t=now().seconds();
    auto reason=health(in,t); if (!reason.empty()) {return {stop(),reason};}
    Command cmd;
    if (in.raw.msg) {
      if (sourceAge(seconds(in.raw.msg->header.stamp),t)>d("command_timeout_sec")) {return {stop(),"command source timeout"};}
      if (!validCommand(*in.raw.msg)) {return {stop(),"invalid command"};}
    }
    if (in.raw.msg && t-in.raw.received<=d("command_timeout_sec")) {
      cmd=clampCommand(*in.raw.msg); reason="raw command";
      if (!b("enabled")) {return {cmd,"bypass"};}
      if (!cmd.enable || cmd.brake) {cmd.forward=cmd.reverse=false; cmd.velocity_mps=cmd.drive_rpm=0; return {cmd,"raw stop"};}
      if (!direction(cmd)) {return steeringOnly(cmd) ? std::make_pair(cmd,std::string("steering center")) : std::make_pair(stop(),std::string("invalid direction"));}
    } else if (b("allow_recovery_twist") && in.recovery.msg && t-in.recovery.received<=d("recovery_timeout_sec")) {
      cmd=recovery(*in.recovery.msg,reason);
    } else {return {stop(),in.raw.msg ? "command timeout" : "waiting for first command"};}
    if (!cmd.enable || cmd.brake) {return {cmd,reason};}
    if (degraded(in,t) && cap(cmd,d("scan_degraded_max_speed_mps"))) {reason="raw command: scan freshness speed cap";}
    const auto failure=collision(in,cmd);
    if (!failure.empty()) {
      auto stopped=stop();
      if (in.vehicle.msg && t-in.vehicle.received<=d("vehicle_state_timeout_sec") &&
        std::isfinite(in.vehicle.msg->steering_angle_rad) &&
        (obstacleReason(failure)||failure.rfind("costmap coverage",0)==0||failure=="obstacle release waiting for stable clearance")) {
        stopped.steering_angle_rad=std::clamp(in.vehicle.msg->steering_angle_rad,-d("max_steering_angle_rad"),d("max_steering_angle_rad"));
        stopped.steering_angle_deg=stopped.steering_angle_rad*180/pi;
      }
      return {stopped,failure};
    }
    return {cmd,reason};
  }
  void tick()
  {
    const auto begin=Steady::now();
    const auto in=snapshot();
    exemption_used_=false;
    scan_elapsed_=pose_elapsed_=grid_elapsed_=0;
    Command cmd; std::string reason;
    try {std::tie(cmd,reason)=evaluate(in);}
    catch (const std::exception & e) {cmd=stop(); reason=e.what();}
    const double elapsed=std::chrono::duration<double>(Steady::now()-begin).count();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (in.stop_generation!=inputs_.stop_generation) {release_.interrupt(); return;}
      const double t=now().seconds();
      if (cmd.enable && !cmd.brake) {
        auto failure=health(inputs_,t);
        if (failure.empty() && in.raw.msg && sourceAge(seconds(in.raw.msg->header.stamp),t)>d("command_timeout_sec")) {failure="command source timeout";}
        if (failure.empty() && elapsed>d("collision_compute_budget_sec")) {failure="collision check deadline exceeded";}
        const bool checked=b("enabled") && b("collision_check_enabled") && direction(cmd)!=0;
        if (checked && failure.empty()) {failure=mapHealth(in.grid,t);}
        if (checked && failure.empty()) {failure=scanHealth(in.scan,t);}
        if (checked && failure.empty()) {failure=scanHealth(inputs_.scan,t);}
        if (checked && b("costmap_collision_check_enabled") && failure.empty() && in.grid && (!in.odom.msg ||
          sourceAge(seconds(in.odom.msg->header.stamp),t)>d("localization_timeout_sec"))) {failure="costmap pose source stale";}
        if (checked && failure.empty() && exemption_used_ &&
          t-in.exemption_received>d("pallet_exemption_timeout_sec")) {failure="pallet exemption expired during check";}
        if (!failure.empty()) {cmd=stop(); reason=failure;}
        else if (checked && degraded(inputs_,t) && cap(cmd,d("scan_degraded_max_speed_mps"))) {reason="raw command: scan freshness speed cap";}
      }
      if (cmd.enable && !cmd.brake) {release_.commit();}
      else if (reason!="obstacle release waiting for stable clearance") {release_.interrupt();}
      pub_->publish(cmd);
    }
    std_msgs::msg::String status; status.data=reason; status_->publish(status);
    if (reason!=last_reason_) {RCLCPP_INFO(get_logger(),"Safety gate state: %s.",reason.c_str()); last_reason_=reason;}
    if (elapsed>.05) {RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),2000,
      "Collision timing: total=%.4fs scan=%.4fs pose_tf=%.4fs costmap=%.4fs result=%s",
      elapsed,scan_elapsed_,pose_elapsed_,grid_elapsed_,reason.c_str());}
  }
  std::map<std::string,double> doubles_;
  std::map<std::string,bool> bools_;
  std::map<std::string,std::string> strings_;
  int threshold_{}, exemption_threshold_{};
  Geometry geometry_;
  double started_{};
  std::mutex mutex_;
  Inputs inputs_;
  ReleaseState release_;
  std::string last_reason_;
  bool exemption_used_{};
  double scan_elapsed_{}, pose_elapsed_{}, grid_elapsed_{};
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  rclcpp::Publisher<Command>::SharedPtr pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
  std::vector<rclcpp::CallbackGroup::SharedPtr> groups_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
  rclcpp::Service<forklift_msgs::srv::SetEmergencyStop>::SharedPtr emergency_service_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace forklift_safety

#ifndef FORKLIFT_SAFETY_NO_MAIN
int main(int argc,char ** argv)
{
  rclcpp::init(argc,argv);
  try {
    auto node=std::make_shared<forklift_safety::SafetyCommandGate>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),6);
    executor.add_node(node); executor.spin();
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("safety_command_gate"),"Safety gate failed: %s",e.what());
    rclcpp::shutdown(); return 1;
  }
  rclcpp::shutdown(); return 0;
}
#endif
