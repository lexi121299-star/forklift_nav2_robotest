#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>
#include <forklift_msgs/msg/forklift_control_command.hpp>

namespace forklift_safety
{
constexpr double pi = 3.14159265358979323846;
using Command = forklift_msgs::msg::ForkliftControlCommand;
struct Point {double x{}, y{};};
struct Pose {double x{}, y{}, yaw{};};
using Polygon = std::vector<Point>;
struct Zone {Pose pose; double half_length{}, half_width{};};
struct Rectangle {double min_x{}, max_x{}, min_y{}, max_y{};};
struct Grid
{
  size_t width{}, height{};
  double resolution{}, received{}, source{};
  Pose origin;
  std::string frame;
  std::vector<int16_t> data;
  std::string error;
};
struct Geometry
{
  Polygon footprint{{1.709, .610}, {1.709, -.610}, {-1.590, -.610}, {-1.590, .610}};
  double wheel_base{1.4}, pivot_radius{.6}, axle_offset{}, pivot_angle{pi/2};
  double scan_spacing{.05}, edge_spacing{.05}, padding{.05};
  double reaction{.9}, deceleration{1.5}, clearance{.5};
  double pivot_deceleration{.15}, pivot_margin{.05};
  double horizon{1.}, minimum_step{.1};
  int threshold{253};
  bool unknown_collision{true}, reverse_escape{true};
  double escape_speed{.15}, escape_steering{.05}, escape_min_x{};
  // Static returns from the forklift body are fixed in base_link and must not
  // be treated as external obstacles while predicting a pivot sweep.
  std::vector<Rectangle> scan_self_filter_rectangles;
};
struct Result
{
  bool blocked{};
  std::string reason;
};
inline int direction(const Command & c) {return c.forward != c.reverse ? (c.forward ? 1 : -1) : 0;}
inline Point transform(Point p, Pose t)
{
  const double c = std::cos(t.yaw), s = std::sin(t.yaw);
  return {t.x + p.x*c - p.y*s, t.y + p.x*s + p.y*c};
}
inline Pose compose(Pose a, Pose b)
{
  const auto p = transform({b.x, b.y}, a);
  return {p.x, p.y, a.yaw + b.yaw};
}
inline double stoppingDistance(double v, double reaction, double deceleration, double margin)
{return v * reaction + v*v/(2*deceleration) + margin;}
inline double sourceAge(double stamp, double now)
{
  const double age = now - stamp;
  return stamp <= 0 || !std::isfinite(age) || age < -.1 ?
         std::numeric_limits<double>::infinity() : std::max(0., age);
}
inline bool exempt(Point p, const std::optional<Zone> & z)
{
  if (!z) {return false;}
  const double dx = p.x-z->pose.x, dy = p.y-z->pose.y;
  const double c = std::cos(z->pose.yaw), s = std::sin(z->pose.yaw);
  return std::abs(dx*c+dy*s) <= z->half_length && std::abs(-dx*s+dy*c) <= z->half_width;
}
inline bool selfReturn(Point p, const Geometry & g)
{
  for (const auto & rectangle : g.scan_self_filter_rectangles) {
    if (p.x >= rectangle.min_x && p.x <= rectangle.max_x &&
      p.y >= rectangle.min_y && p.y <= rectangle.max_y)
    {
      return true;
    }
  }
  return false;
}
inline Polygon atPose(const Polygon & f, Pose p)
{
  Polygon out; out.reserve(f.size());
  const double c = std::cos(p.yaw), s = std::sin(p.yaw);
  for (auto q : f) {out.push_back({p.x+q.x*c-q.y*s, p.y+q.x*s+q.y*c});}
  return out;
}
inline bool inside(Point p, const Polygon & poly, double padding)
{
  bool in = false;
  for (size_t i=0; i<poly.size(); ++i) {
    const auto a=poly[i], b=poly[(i+1)%poly.size()];
    const double dx=b.x-a.x, dy=b.y-a.y, l2=dx*dx+dy*dy;
    const double r=l2 <= 1e-12 ? 0 : std::clamp(((p.x-a.x)*dx+(p.y-a.y)*dy)/l2, 0., 1.);
    if (std::hypot(p.x-a.x-r*dx, p.y-a.y-r*dy) <= padding) {return true;}
    if ((a.y>p.y)!=(b.y>p.y) && p.x < dx*(p.y-a.y)/dy+a.x) {in=!in;}
  }
  return in;
}
inline std::vector<Pose> predict(const Command & cmd, const Geometry & g, double horizon, double dt)
{
  std::vector<Pose> poses{{}};
  const double v=direction(cmd)*std::abs(cmd.velocity_mps);
  if (!cmd.enable || cmd.brake || std::abs(v)<=1e-6) {return poses;}
  const int count=std::max(1, static_cast<int>(std::ceil(horizon/dt)));
  Pose p;
  for (int i=0; i<count; ++i) {
    if (std::abs(cmd.steering_angle_rad)>=g.pivot_angle-1e-3) {
      const double rx=p.x+g.axle_offset*std::cos(p.yaw), ry=p.y+g.axle_offset*std::sin(p.yaw);
      p.yaw+=std::copysign(std::abs(v)/g.pivot_radius, cmd.steering_angle_rad)*dt;
      p.x=rx-g.axle_offset*std::cos(p.yaw); p.y=ry-g.axle_offset*std::sin(p.yaw);
    } else {
      p.x+=v*std::cos(p.yaw)*dt; p.y+=v*std::sin(p.yaw)*dt;
      p.yaw+=v*std::tan(cmd.steering_angle_rad)/g.wheel_base*dt;
    }
    poses.push_back(p);
  }
  return poses;
}
inline double spatialStep(double v, double spacing, double horizon)
{return v<=1e-6 ? horizon : std::max(.01, std::min(horizon, spacing/v));}
inline std::vector<Pose> pivotPoses(double commanded, double measured, const Geometry & g)
{
  if (!std::isfinite(commanded) || !std::isfinite(measured)) {throw std::runtime_error("pivot angular feedback unavailable");}
  double radius=0;
  for (auto p : g.footprint) {radius=std::max(radius, std::hypot(p.x-g.axle_offset,p.y));}
  const double step=std::min(.03, g.scan_spacing/std::max(radius,.01));
  std::vector<Pose> poses{{}};
  for (double sign : {-1.,1.}) {
    const double rate=std::max({0.,sign*commanded,sign*measured});
    if (rate<=1e-6) {continue;}
    const double angle=std::min(2*pi,stoppingDistance(rate,g.reaction,g.pivot_deceleration,g.pivot_margin));
    const int count=std::max(1,static_cast<int>(std::ceil(angle/step)));
    for (int i=1; i<=count; ++i) {
      const double yaw=sign*angle*i/count;
      poses.push_back({g.axle_offset*(1-std::cos(yaw)),-g.axle_offset*std::sin(yaw),yaw});
    }
  }
  return poses;
}
inline Result scanSweep(const std::vector<Point> & input, const Command & cmd,
  const Geometry & g, const std::vector<Pose> & poses, const std::optional<Zone> & zone={})
{
  if (!direction(cmd) || std::abs(cmd.velocity_mps)<=1e-6) {return {false,"scan sweep clear"};}
  double radius=0, distance=0;
  for (auto p : g.footprint) {radius=std::max(radius,std::hypot(p.x,p.y));}
  for (auto p : poses) {distance=std::max(distance,std::hypot(p.x,p.y));}
  const double bound=radius+g.padding+distance;
  std::vector<Point> points;
  const bool pivot_motion = std::abs(cmd.steering_angle_rad) >= g.pivot_angle - 1e-3;
  for (auto p : input) {
    if (p.x*p.x+p.y*p.y<=bound*bound && !exempt(p,zone) &&
      (!pivot_motion || !selfReturn(p,g))) {
      points.push_back(p);
    }
  }
  auto hits=[&](Pose pose) {
      const auto polygon=atPose(g.footprint,pose);
      std::set<size_t> result;
      for (size_t i=0;i<points.size();++i) {if (inside(points[i],polygon,g.padding)) {result.insert(i);}}
      return result;
    };
  const auto initial=hits(poses.front());
  bool escape=g.reverse_escape && direction(cmd)<0 && cmd.velocity_mps<=g.escape_speed+1e-9 &&
    std::abs(cmd.steering_angle_rad)<=g.escape_steering+1e-9 && !initial.empty();
  for (auto i : initial) {if (points[i].x<g.escape_min_x) {escape=false;}}
  if (escape) {
    size_t previous=initial.size();
    for (size_t i=1;i<poses.size();++i) {
      const auto current=hits(poses[i]);
      if (!std::includes(initial.begin(),initial.end(),current.begin(),current.end())) {
        return {true,"scan reverse escape would hit a new obstacle"};
      }
      if (current.size()>previous) {return {true,"scan reverse escape overlap is increasing"};}
      previous=current.size();
    }
    return previous ? Result{true,"scan reverse escape does not clear current overlap"} :
           Result{false,"scan reverse escape clear"};
  }
  if (!initial.empty()) {return {true,"scan footprint sweep collision"};}
  for (size_t i=1;i<poses.size();++i) {if (!hits(poses[i]).empty()) {return {true,"scan footprint sweep collision"};}}
  return {false,"scan sweep clear"};
}
inline Result gridSweep(const Grid & grid, const Geometry & g, const std::vector<Pose> & poses,
  const std::optional<Zone> & zone={}, bool escape=false)
{
  if (!grid.error.empty()) {return {true,"costmap invalid: "+grid.error};}
  if (!grid.width || !grid.height || !std::isfinite(grid.resolution) || grid.resolution<=0 ||
    grid.data.size()/grid.width<grid.height) {return {true,"costmap invalid: dimensions or data"};}
  const double c=std::cos(grid.origin.yaw), s=std::sin(grid.origin.yaw);
  bool initial=false, cleared=false;
  for (size_t k=0; k<poses.size(); ++k) {
    const auto polygon=atPose(g.footprint,poses[k]);
    Result result;
    for (size_t i=0;i<polygon.size() && !result.blocked;++i) {
      const auto a=polygon[i], b=polygon[(i+1)%polygon.size()];
      const int steps=std::max(1,static_cast<int>(std::ceil(std::hypot(b.x-a.x,b.y-a.y)/g.edge_spacing)));
      for (int j=0;j<=steps;++j) {
        const Point p{a.x+(b.x-a.x)*j/steps,a.y+(b.y-a.y)*j/steps};
        const double dx=p.x-grid.origin.x, dy=p.y-grid.origin.y;
        const double mx=std::floor((dx*c+dy*s)/grid.resolution), my=std::floor((-dx*s+dy*c)/grid.resolution);
        if (!std::isfinite(mx) || !std::isfinite(my) || mx<0 || my<0 || mx>=grid.width || my>=grid.height) {
          return {true,"costmap coverage insufficient: point=("+std::to_string(p.x)+","+
            std::to_string(p.y)+") sweep_pose=("+std::to_string(poses[k].x)+","+
            std::to_string(poses[k].y)+","+std::to_string(poses[k].yaw)+")"};
        }
        const int cost=grid.data[static_cast<size_t>(my)*grid.width+static_cast<size_t>(mx)];
        if (cost<0 && g.unknown_collision) {return {true,"footprint collision: unknown costmap cell"};}
        if (cost>=g.threshold && !exempt(p,zone)) {
          result={true,"footprint collision: cost "+std::to_string(cost)+" >= "+std::to_string(g.threshold)};
          break;
        }
      }
    }
    if (k==0) {initial=result.blocked; cleared=!initial;}
    if (result.blocked && !(escape && !cleared)) {return result;}
    if (!result.blocked) {cleared=true;}
  }
  if (initial && !cleared) {return {true,"footprint reverse escape does not clear current overlap"};}
  return {false,initial ? "footprint reverse escape clear" : "footprint sweep clear"};
}
inline bool obstacleReason(const std::string & r)
{
  for (auto prefix : {"footprint collision: cost ","footprint collision: unknown",
    "footprint reverse escape","scan footprint sweep collision","scan reverse escape"}) {
    if (r.rfind(prefix,0)==0) {return true;}
  }
  return false;
}
struct ReleaseState
{
  bool active{}, ready{}, signature_set{};
  double floor{}, clear_since{-1}, first_scan{}, first_map{}, steering{};
  int travel{};
  void interrupt() {ready=false; clear_since=-1;}
  void prepare(int d,double steer,double threshold)
  {
    if (!signature_set || d!=travel || std::abs(steer-steering)>threshold) {
      signature_set=true; travel=d; steering=steer; floor=0; interrupt();
    }
  }
  void blocked(double speed) {active=true; floor=std::max(floor,speed); interrupt();}
  bool clear(double now,double scan,double map,double duration)
  {
    if (!active) {return true;}
    if (clear_since<0 || now<clear_since) {clear_since=now; first_scan=scan; first_map=map;}
    ready=now-clear_since>=duration && (scan<0 || scan!=first_scan) && (map<0 || map!=first_map);
    return ready;
  }
  void commit() {if (ready) {active=false; floor=0; interrupt();}}
};
}  // namespace forklift_safety
