#include <iostream>
#include <yaml-cpp/yaml.h>
#include "forklift_safety/collision_core.hpp"
using namespace forklift_safety;
int main()
{
  std::string line;
  while (std::getline(std::cin,line)) {
    try {
      const auto q=YAML::Load(line);
      Geometry g;
      if (q["footprint"]) {
        g.footprint.clear(); for (auto p:q["footprint"]) {g.footprint.push_back({p[0].as<double>(),p[1].as<double>()});}
      }
      if (q["threshold"]) {g.threshold=q["threshold"].as<int>();}
      if (q["unknown"]) {g.unknown_collision=q["unknown"].as<bool>();}
      if (q["padding"]) {g.padding=q["padding"].as<double>();}
      if (q["escape"]) {g.reverse_escape=q["escape"].as<bool>();}
      Command cmd; cmd.enable=true; cmd.forward=q["direction"].as<int>()>0; cmd.reverse=!cmd.forward;
      cmd.velocity_mps=q["speed"].as<double>(); cmd.steering_angle_rad=q["steering"].as<double>();
      const double horizon=q["horizon"].as<double>();
      auto poses=q["measured_yaw_rate"] ?
        pivotPoses(std::copysign(cmd.velocity_mps/g.pivot_radius,cmd.steering_angle_rad),q["measured_yaw_rate"].as<double>(),g) :
        predict(cmd,g,horizon,spatialStep(cmd.velocity_mps,g.scan_spacing,horizon));
      std::optional<Zone> zone;
      if (q["zone"]) {auto z=q["zone"]; zone=Zone{{z[0].as<double>(),z[1].as<double>(),z[2].as<double>()},z[3].as<double>(),z[4].as<double>()};}
      Result result;
      if (q["points"]) {
        std::vector<Point> points;
        for (auto p:q["points"]) {points.push_back({p[0].as<double>(),p[1].as<double>()});}
        result=scanSweep(points,cmd,g,poses,zone);
      } else {
        Grid grid; const auto m=q["grid"];
        grid.width=m["width"].as<int>(); grid.height=m["height"].as<int>(); grid.resolution=m["resolution"].as<double>();
        auto o=m["origin"]; grid.origin={o[0].as<double>(),o[1].as<double>(),o[2].as<double>()};
        grid.data=m["data"].as<std::vector<int16_t>>();
        auto p=q["pose"]; Pose pose{p[0].as<double>(),p[1].as<double>(),p[2].as<double>()};
        for (auto & pp:poses) {pp=compose(pose,pp);}
        result=gridSweep(grid,g,poses,zone,q["escape"].as<bool>());
      }
      std::cout<<(result.blocked ? "1" : "0")<<" "<<result.reason<<"\n";
    } catch (const std::exception & e) {std::cerr<<e.what()<<"\n"; return 1;}
  }
}
