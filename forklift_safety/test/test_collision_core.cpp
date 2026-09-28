#include <gtest/gtest.h>
#include "forklift_safety/collision_core.hpp"
using namespace forklift_safety;

TEST(Core, DynamicDistanceAndFutureStamp)
{
  EXPECT_NEAR(stoppingDistance(2.,.9,1.5,.5),3.633333333,1e-8);
  EXPECT_TRUE(std::isinf(sourceAge(0,10)));
  EXPECT_TRUE(std::isinf(sourceAge(10.2,10)));
  EXPECT_NEAR(sourceAge(9.9,10),.1,1e-8);
}
TEST(Core, PivotSweepsResidualAndCommandedDirections)
{
  Geometry g;
  const auto poses=pivotPoses(.1,-.2,g);
  double lo=0,hi=0;
  for (auto p:poses) {lo=std::min(lo,p.yaw); hi=std::max(hi,p.yaw);}
  EXPECT_LT(lo,-.3); EXPECT_GT(hi,.15);
}
TEST(Core, CoverageCannotBeReverseEscape)
{
  Grid grid; grid.width=grid.height=100; grid.resolution=.05;
  grid.origin={-2.5,-2.5,0}; grid.data.resize(10000);
  Geometry g;
  auto result=gridSweep(grid,g,{{100,100,0},{0,0,0}},{},true);
  EXPECT_TRUE(result.blocked); EXPECT_EQ(result.reason.find("costmap coverage"),0u);
}
TEST(Core, ReleaseNeedsDistinctFreshFramesAndCommit)
{
  ReleaseState r; r.prepare(1,0,.15); r.blocked(.8);
  EXPECT_FALSE(r.clear(10,1,1,.5)); EXPECT_FALSE(r.clear(11,1,2,.5));
  EXPECT_TRUE(r.clear(11.1,2,2,.5)); EXPECT_TRUE(r.active);
  r.interrupt(); r.commit(); EXPECT_TRUE(r.active);
  EXPECT_FALSE(r.clear(12,3,3,.5)); EXPECT_TRUE(r.clear(13,4,4,.5));
  r.commit(); EXPECT_FALSE(r.active); EXPECT_EQ(r.floor,0);
}
TEST(Core, DifferentManeuverRetainsLatchButResetsEnvelope)
{
  ReleaseState r; r.prepare(1,0,.15); r.blocked(.8); r.prepare(-1,0,.15);
  EXPECT_TRUE(r.active); EXPECT_EQ(r.floor,0);
  EXPECT_FALSE(obstacleReason("costmap coverage insufficient"));
  EXPECT_TRUE(obstacleReason("scan footprint sweep collision"));
}
TEST(Core, ReverseEscapeRejectsNewObstacle)
{
  Geometry g; g.footprint={{-.1,-.1},{.1,-.1},{.1,.1},{-.1,.1}}; g.padding=0;
  Command c; c.enable=c.reverse=true; c.velocity_mps=.1;
  auto poses=predict(c,g,3,.5);
  EXPECT_FALSE(scanSweep({{.08,0}},c,g,poses).blocked);
  EXPECT_TRUE(scanSweep({{.08,0},{-.25,0}},c,g,poses).blocked);
}
