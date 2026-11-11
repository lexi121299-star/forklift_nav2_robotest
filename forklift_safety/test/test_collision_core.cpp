#include <gtest/gtest.h>
#include "forklift_safety/collision_core.hpp"
#include "forklift_safety/trajectory_sweep.hpp"
using namespace forklift_safety;

static std::vector<Pose> straightReference(int travel=1)
{
  std::vector<Pose> path;
  for (int i=0;i<=80;++i) {path.push_back({travel*i*.025,0,0});}
  return path;
}

TEST(TrajectorySweep, StraightExitDoesNotHoldCurrentSteeringThroughClearance)
{
  Geometry g;
  Command c; c.enable=c.forward=true; c.velocity_mps=.066; c.steering_angle_rad=-1.1;
  const double horizon=stoppingDistance(c.velocity_mps,g.reaction,g.deceleration,g.clearance)/c.velocity_mps;
  const auto old=predict(c,g,horizon,spatialStep(c.velocity_mps,g.scan_spacing,horizon));
  const std::vector<Point> points{{1.3,-1.84}};
  ASSERT_TRUE(scanSweep(points,c,g,old).blocked);
  const auto next=trajectorySweepPoses(straightReference(),c,g,c.velocity_mps,-1.1,.12);
  ASSERT_TRUE(next);
  EXPECT_FALSE(scanSweep(points,c,g,*next).blocked);
}

TEST(TrajectorySweep, CoversNominalPathAndRateLimitedSteering)
{
  Geometry g;
  Command c; c.enable=c.forward=true; c.velocity_mps=.1;
  const auto poses=trajectorySweepPoses(straightReference(),c,g,.1,-1.1,.12);
  ASSERT_TRUE(poses);
  bool nominal=false,lagged=false;
  for (auto p:*poses) {
    nominal=nominal || (p.x>.4 && std::abs(p.y)<1e-6 && std::abs(p.yaw)<1e-6);
    lagged=lagged || std::abs(p.yaw)>.1;
  }
  EXPECT_TRUE(nominal); EXPECT_TRUE(lagged);
  EXPECT_TRUE(scanSweep({{.2,0}},c,g,*poses).blocked);
}

TEST(TrajectorySweep, SteeringLagCollisionCannotBeHiddenByClearCentreline)
{
  Geometry g;
  Command c; c.enable=c.forward=true; c.velocity_mps=.1;
  const auto reference=straightReference();
  const std::vector<Point> obstacle{{1.8,-.9}};
  EXPECT_FALSE(scanSweep(obstacle,c,g,reference).blocked);
  const auto poses=trajectorySweepPoses(reference,c,g,.1,-1.1,.12);
  ASSERT_TRUE(poses);
  EXPECT_TRUE(scanSweep(obstacle,c,g,*poses).blocked);
}

TEST(TrajectorySweep, ReverseUsesReverseBodyHeading)
{
  Geometry g;
  Command c; c.enable=c.reverse=true; c.velocity_mps=.1;
  const auto poses=trajectorySweepPoses(straightReference(-1),c,g,.1,0,.12);
  ASSERT_TRUE(poses);
  for (auto p:*poses) {EXPECT_LE(p.x,1e-9); EXPECT_NEAR(p.yaw,0,1e-9);}
  EXPECT_TRUE(scanSweep({{-1.9,0}},c,g,*poses).blocked);
}

TEST(TrajectorySweep, InvalidOrInsufficientReferenceFallsBack)
{
  Geometry g;
  Command c; c.enable=c.forward=true; c.velocity_mps=.1;
  auto path=straightReference();
  path.resize(3);
  EXPECT_FALSE(trajectorySweepPoses(path,c,g,.1,0,.12));
  EXPECT_FALSE(trajectorySweepPoses(straightReference(-1),c,g,.1,0,.12));
  path=straightReference(); path[1].yaw=pi;
  EXPECT_FALSE(trajectorySweepPoses(path,c,g,.1,0,.12));
  path=straightReference(); path[0].x=.5;
  EXPECT_FALSE(trajectorySweepPoses(path,c,g,.1,0,.12));
  path=straightReference(); path[3].x=std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(trajectorySweepPoses(path,c,g,.1,0,.12));
  EXPECT_FALSE(trajectorySweepPoses(straightReference(),c,g,.1,pi/2,.12));
  c.steering_angle_rad=pi/2;
  EXPECT_FALSE(trajectorySweepPoses(straightReference(),c,g,.1,0,.12));
}

TEST(TrajectorySweep, HighSpeedRetainsBrakingDistanceAndClearance)
{
  Geometry g;
  Command c; c.enable=c.forward=true; c.velocity_mps=2.;
  auto path=straightReference();
  EXPECT_FALSE(trajectorySweepPoses(path,c,g,2.,0,.12));
  for (int i=81;i<=240;++i) {path.push_back({i*.025,0,0});}
  const auto poses=trajectorySweepPoses(path,c,g,2.,0,.12);
  ASSERT_TRUE(poses);
  double farthest=0;
  for (auto p:*poses) {farthest=std::max(farthest,p.x);}
  EXPECT_NEAR(farthest,3.633333333,1e-6);
  const auto braking=brakingSweepPoses(c,g,0.);
  EXPECT_FALSE(scanSweep({{5.,0}},c,g,braking).blocked);
  EXPECT_TRUE(scanSweep({{5.,0}},c,g,*poses).blocked);
}

TEST(TrajectorySweep, RemoteTerminalHeadingDoesNotInvalidateCurrentClearWindow)
{
  Geometry g;
  Command c; c.enable=c.forward=true; c.velocity_mps=.1;
  auto path=straightReference();
  path.back().yaw=pi;
  EXPECT_TRUE(trajectorySweepPoses(path,c,g,.1,0,.12));
  path[5].yaw=pi;
  EXPECT_FALSE(trajectorySweepPoses(path,c,g,.1,0,.12));
}

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
TEST(Core, ReleaseEnvelopeOnlyShrinksAfterStoppedSettle)
{
  ReleaseState r; r.prepare(1,0,.15); r.blocked(1.4);
  EXPECT_FALSE(r.settle(10,0,.10,.03,.25));
  EXPECT_FALSE(r.settle(10.20,0,.10,.03,.25));
  EXPECT_TRUE(r.settle(10.25,0,.10,.03,.25));
  EXPECT_DOUBLE_EQ(r.floor,.10);
  r.blocked(1.4);
  EXPECT_FALSE(r.settle(11,.04,.10,.03,.25));
  EXPECT_DOUBLE_EQ(r.floor,1.4);
}
TEST(Core, ReverseEscapeRejectsNewObstacle)
{
  Geometry g; g.footprint={{-.1,-.1},{.1,-.1},{.1,.1},{-.1,.1}}; g.padding=0;
  Command c; c.enable=c.reverse=true; c.velocity_mps=.1;
  auto poses=predict(c,g,3,.5);
  EXPECT_FALSE(scanSweep({{.08,0}},c,g,poses).blocked);
  EXPECT_TRUE(scanSweep({{.08,0},{-.25,0}},c,g,poses).blocked);
}
TEST(Core, ForwardEscapeClearsRearOverlap)
{
  Geometry g; g.footprint={{-.1,-.1},{.1,-.1},{.1,.1},{-.1,.1}}; g.padding=0;
  g.forward_escape=true;
  Command c; c.enable=c.forward=true; c.velocity_mps=.1;
  auto poses=predict(c,g,3,.5);
  EXPECT_FALSE(scanSweep({{-.08,0}},c,g,poses).blocked);
  EXPECT_TRUE(scanSweep({{-.08,0},{.25,0}},c,g,poses).blocked);
}
TEST(Core, DirectionalEscapeEligibilityRejectsWrongSide)
{
  Geometry g; g.footprint={{-.1,-.1},{.1,-.1},{.1,.1},{-.1,.1}}; g.padding=0;
  g.forward_escape=true;
  Command c; c.enable=c.forward=true; c.velocity_mps=.3;
  EXPECT_TRUE(directionalEscapeEligible({{-.08,0}},c,g));
  EXPECT_FALSE(directionalEscapeEligible({{.08,0}},c,g));
}
TEST(Core, ScanSelfFilterExcludesOnlyConfiguredBodyReturns)
{
  Geometry g;
  g.footprint={{-.1,-.1},{.1,-.1},{.1,.1},{-.1,.1}};
  g.padding=0;
  g.scan_self_filter_rectangles={{-.05,.05,-.05,.05}};
  Command c; c.enable=c.forward=true; c.velocity_mps=.1; c.steering_angle_rad=g.pivot_angle;
  const auto poses=predict(c,g,.2,.1);
  EXPECT_FALSE(scanSweep({{0,0}},c,g,poses).blocked);
  EXPECT_TRUE(scanSweep({{0,0},{.08,0}},c,g,poses).blocked);
}
