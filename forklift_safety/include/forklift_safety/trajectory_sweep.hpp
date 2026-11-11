#pragma once

#include "forklift_safety/collision_core.hpp"

namespace forklift_safety
{
inline double angleDifference(double a, double b)
{return std::atan2(std::sin(a-b),std::cos(a-b));}

inline std::vector<Pose> brakingSweepPoses(
  const Command & command,const Geometry & geometry,double measured_steering)
{
  const double speed=std::abs(command.velocity_mps);
  const double horizon=stoppingDistance(speed,geometry.reaction,geometry.deceleration,0.)/
    std::max(speed,1e-6);
  double radius=0;
  for (auto p:geometry.footprint) {radius=std::max(radius,std::hypot(p.x,p.y));}
  std::vector<Pose> poses;
  for (const double steering:{command.steering_angle_rad,measured_steering}) {
    auto probe=command; probe.steering_angle_rad=steering;
    const double curvature=std::abs(std::tan(steering)/geometry.wheel_base);
    const double spacing=geometry.scan_spacing/(1.+radius*curvature);
    const double dt=std::max(1e-4,std::min(horizon,spacing/std::max(speed,1e-6)));
    const auto arc=predict(probe,geometry,horizon,dt);
    poses.insert(poses.end(),arc.begin(),arc.end());
  }
  return poses;
}

// Returns no reference if it cannot cover the braking distance plus clearance.
// The caller must then retain the original, conservative command-arc sweep.
inline std::optional<std::vector<Pose>> trajectorySweepPoses(
  const std::vector<Pose> & reference, const Command & command,
  const Geometry & geometry, double speed, double measured_steering,
  double steering_rate)
{
  if (reference.size()<2 || reference.size()>512 || !direction(command) ||
    !std::isfinite(speed) || speed<=1e-6 || !std::isfinite(measured_steering) ||
    !std::isfinite(steering_rate) || steering_rate<=0 ||
    std::abs(measured_steering)>=geometry.pivot_angle-.01 ||
    std::abs(command.steering_angle_rad)>=geometry.pivot_angle-1e-3)
  {return {};}
  for (const auto & p : reference) {
    if (!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.yaw)) {return {};}
  }
  if (std::hypot(reference.front().x,reference.front().y)>.15 ||
    std::abs(angleDifference(reference.front().yaw,0.))>.10) {return {};}
  const double required=stoppingDistance(speed,geometry.reaction,
    geometry.deceleration,geometry.clearance);
  double length=0;
  for (size_t i=1;i<reference.size() && length<required-1e-9;++i) {
    const auto & a=reference[i-1]; const auto & b=reference[i];
    const double ds=std::hypot(b.x-a.x,b.y-a.y);
    const double dh=angleDifference(b.yaw,a.yaw);
    const double along=(b.x-a.x)*std::cos(a.yaw)+(b.y-a.y)*std::sin(a.yaw);
    if (ds<1e-6 || ds>.15 || along*direction(command)<=0 ||
      std::abs(dh)>ds*2.5+.01) {return {};}
    length+=ds;
  }
  if (length+1e-6<required) {return {};}
  double radius=0;
  for (const auto & p : geometry.footprint) {radius=std::max(radius,std::hypot(p.x,p.y));}
  std::vector<Pose> poses{{}};
  Pose actual;
  double steering=measured_steering, covered=0;
  for (size_t i=1;i<reference.size() && covered<required-1e-9;++i) {
    const auto & a=reference[i-1]; const auto & b=reference[i];
    const double ds=std::hypot(b.x-a.x,b.y-a.y);
    const double dh=angleDifference(b.yaw,a.yaw);
    const double used=std::min(ds,required-covered);
    // Bound the movement of the outermost corner, not only the rear axle.
    const double corner_motion=used*(1.+radius*std::max(std::abs(dh/ds),
      std::abs(std::tan(steering)/geometry.wheel_base)));
    const int samples=std::max(1,static_cast<int>(std::ceil(
      corner_motion/std::min(.05,geometry.scan_spacing))));
    const double step=used/samples;
    const double desired=std::clamp(std::atan(geometry.wheel_base*dh/ds*direction(command)),
      -geometry.pivot_angle+.01,geometry.pivot_angle-.01);
    for (int j=1;j<=samples;++j) {
      const double u=step*j/ds;
      poses.push_back({a.x+(b.x-a.x)*u,a.y+(b.y-a.y)*u,a.yaw+dh*u});
      const double previous=steering;
      steering+=std::clamp(desired-steering,-steering_rate*step/speed,steering_rate*step/speed);
      const double delta=direction(command)*step;
      const double curvature=std::tan(.5*(previous+steering))/geometry.wheel_base;
      const double next=actual.yaw+curvature*delta;
      if (std::abs(curvature)<1e-6) {
        actual.x+=delta*std::cos(actual.yaw); actual.y+=delta*std::sin(actual.yaw);
      } else {
        actual.x+=(std::sin(next)-std::sin(actual.yaw))/curvature;
        actual.y-=(std::cos(next)-std::cos(actual.yaw))/curvature;
      }
      actual.yaw=next;
      poses.push_back(actual);
    }
    covered+=used;
    if (poses.size()>4096) {return {};}
  }
  return poses;
}
}  // namespace forklift_safety
