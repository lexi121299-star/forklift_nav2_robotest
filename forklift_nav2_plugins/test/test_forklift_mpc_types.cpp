#include "forklift_nav2_plugins/forklift_mpc_types.hpp"

#include <cmath>
#include <deque>

#include "gtest/gtest.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace forklift_nav2_plugins
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

ForkliftVehicleModel testVehicleModel()
{
  return ForkliftVehicleModel({1.2, 0.5, 0.2, 1.0, 0.5, 1.0});
}

ForkliftVehicleModel pivotVehicleModel()
{
  return ForkliftVehicleModel({
    1.2,
    0.5 * kPi,
    1.6,
    1.0,
    0.5,
    1.0,
    true,
    0.5 * kPi,
    0.03,
    0.6});
}

ForkliftVehicleModel rearAxlePivotVehicleModel()
{
  return ForkliftVehicleModel({
    1.2,
    0.5 * kPi,
    1.6,
    1.0,
    0.5,
    1.0,
    true,
    0.5 * kPi,
    0.03,
    0.6,
    -0.34});
}

TEST(ForkliftMpcTypes, MakeStateNormalizesThetaAndClampsPhi)
{
  const auto vehicle_model = testVehicleModel();

  const auto state = makeMpcState(1.0, -2.0, 3.5 * kPi, 1.2, vehicle_model);

  EXPECT_DOUBLE_EQ(state.x, 1.0);
  EXPECT_DOUBLE_EQ(state.y, -2.0);
  EXPECT_NEAR(state.theta, -0.5 * kPi, 1e-9);
  EXPECT_DOUBLE_EQ(state.phi, 0.5);
}

TEST(ForkliftMpcTypes, MakeStateFromPoseUsesRosYawConvention)
{
  const auto vehicle_model = testVehicleModel();
  geometry_msgs::msg::Pose pose;
  pose.position.x = 2.0;
  pose.position.y = 3.0;

  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, 0.5 * kPi);
  pose.orientation = tf2::toMsg(quaternion);

  const auto state = makeMpcStateFromPose(pose, -0.25, vehicle_model);

  EXPECT_DOUBLE_EQ(state.x, 2.0);
  EXPECT_DOUBLE_EQ(state.y, 3.0);
  EXPECT_NEAR(state.theta, 0.5 * kPi, 1e-9);
  EXPECT_DOUBLE_EQ(state.phi, -0.25);
}

TEST(ForkliftMpcTypes, ControlToSteeringTargetRespectsVelocityAndSteeringRateLimits)
{
  const auto vehicle_model = testVehicleModel();

  const auto control = makeMpcControlToSteeringTarget(
    2.0, 0.0, 0.0, 0.5, 1.0, vehicle_model);

  EXPECT_DOUBLE_EQ(control.acceleration, 0.5);
  EXPECT_DOUBLE_EQ(control.steering_rate, 0.2);
}

TEST(ForkliftMpcTypes, PivotYawRequiresAngleAndRateToSettle)
{
  EXPECT_TRUE(pivotYawIsSettled(0.02, 0.02, 0.025, 0.03));
  EXPECT_FALSE(pivotYawIsSettled(0.03, 0.0, 0.025, 0.03));
  EXPECT_FALSE(pivotYawIsSettled(0.0, 0.04, 0.025, 0.03));
}

TEST(ForkliftMpcTypes, CommandFromControlAdvancesSteeringByRate)
{
  const auto vehicle_model = testVehicleModel();
  auto state = makeMpcState(0.0, 0.0, 0.0, 0.1, vehicle_model);
  state.velocity = 0.2;
  const MpcControl control{0.4, 0.2};

  const auto command = commandFromMpcControl(state, control, 0.5, vehicle_model);

  EXPECT_DOUBLE_EQ(command.velocity, 0.4);
  EXPECT_NEAR(command.steering_angle, 0.2, 1e-9);
}

TEST(ForkliftMpcTypes, PredictStateAdvancesPoseThetaAndPhi)
{
  const auto vehicle_model = testVehicleModel();
  auto state = makeMpcState(0.0, 0.0, 0.0, 0.0, vehicle_model);
  state.velocity = 0.1;
  const MpcControl control{0.5, 0.2};

  const auto next = predictMpcState(state, control, 1.0, vehicle_model);

  EXPECT_NEAR(next.x, 0.6, 1e-9);
  EXPECT_NEAR(next.y, 0.0, 1e-9);
  EXPECT_NEAR(next.phi, 0.2, 1e-9);
  EXPECT_NEAR(next.theta, 0.6 * std::tan(0.2) / 1.2, 1e-9);
}

TEST(ForkliftMpcTypes, PredictStateUsesPivotTurnKinematics)
{
  const auto vehicle_model = pivotVehicleModel();
  auto state = makeMpcState(0.0, 0.0, 0.0, 0.5 * kPi, vehicle_model);
  state.velocity = 0.5;
  const MpcControl control{0.5, 0.0};

  const auto next = predictMpcState(state, control, 1.0, vehicle_model);

  EXPECT_NEAR(next.x, 0.0, 1e-9);
  EXPECT_NEAR(next.y, 0.0, 1e-9);
  EXPECT_NEAR(next.phi, 0.5 * kPi, 1e-9);
  EXPECT_NEAR(next.theta, 1.0, 1e-9);
}

TEST(ForkliftMpcTypes, PredictStateUsesRearAxlePivotKinematics)
{
  const auto vehicle_model = rearAxlePivotVehicleModel();
  auto state = makeMpcState(1.0, 2.0, 0.0, 0.5 * kPi, vehicle_model);
  state.velocity = 0.5;
  const MpcControl control{0.5, 0.0};

  const auto next = predictMpcState(state, control, 1.0, vehicle_model);

  const double rear_axle_x_offset = vehicle_model.parameters().rear_axle_x_offset;
  const double rear_x_before = state.x + rear_axle_x_offset * std::cos(state.theta);
  const double rear_y_before = state.y + rear_axle_x_offset * std::sin(state.theta);
  const double rear_x_after = next.x + rear_axle_x_offset * std::cos(next.theta);
  const double rear_y_after = next.y + rear_axle_x_offset * std::sin(next.theta);
  EXPECT_NEAR(rear_x_after, rear_x_before, 1e-9);
  EXPECT_NEAR(rear_y_after, rear_y_before, 1e-9);
  EXPECT_NEAR(next.phi, 0.5 * kPi, 1e-9);
  EXPECT_NEAR(next.theta, 1.0, 1e-9);
}

TEST(ForkliftMpcTypes, NegativeDtDoesNotMoveState)
{
  const auto vehicle_model = testVehicleModel();
  const auto state = makeMpcState(1.0, 2.0, 0.3, 0.1, vehicle_model);
  const MpcControl control{0.6, 0.2};

  const auto next = predictMpcState(state, control, -1.0, vehicle_model);

  EXPECT_DOUBLE_EQ(next.x, state.x);
  EXPECT_DOUBLE_EQ(next.y, state.y);
  EXPECT_DOUBLE_EQ(next.theta, state.theta);
  EXPECT_DOUBLE_EQ(next.phi, state.phi);
}

TEST(ForkliftMpcTypes, PredictiveBrakeHoldsThroughOvershootUntilStationary)
{
  PivotBrakeState state;
  PivotBrakeOptions options;
  EXPECT_EQ(state.update(0.30, 0.20, 0.0, 1.0, options), PivotBrakeDecision::Drive);
  EXPECT_EQ(state.update(0.16, 0.20, 0.0, 1.1, options), PivotBrakeDecision::Hold);
  EXPECT_EQ(state.update(-0.07, 0.12, 0.0, 1.5, options), PivotBrakeDecision::Hold);
  EXPECT_EQ(state.corrections, 0);
  EXPECT_EQ(state.update(-0.07, 0.0, 0.0, 2.0, options), PivotBrakeDecision::Hold);
  EXPECT_EQ(state.update(-0.07, 0.0, 0.0, 2.31, options), PivotBrakeDecision::Drive);
  EXPECT_EQ(state.corrections, 1);
  EXPECT_EQ(state.update(-0.03, -0.15, 0.0, 2.5, options), PivotBrakeDecision::Hold);
  EXPECT_EQ(state.update(0.08, -0.12, 0.0, 3.0, options), PivotBrakeDecision::Hold);
  EXPECT_EQ(state.update(0.08, 0.0, 0.0, 3.5, options), PivotBrakeDecision::Hold);
  EXPECT_EQ(state.update(0.08, 0.0, 0.0, 3.81, options), PivotBrakeDecision::Recover);
  EXPECT_EQ(state.update(0.20, 0.0, 0.0, 4.0, options), PivotBrakeDecision::Failed);
}

TEST(ForkliftMpcTypes, PivotRequiresContinuousHeadingAndRateStability)
{
  PivotBrakeState state;
  PivotBrakeOptions o;
  EXPECT_EQ(state.update(0.01, 0.0, 0.0, 1.0, o), PivotBrakeDecision::Hold);
  EXPECT_EQ(state.update(0.01, 0.10, 0.0, 1.2, o), PivotBrakeDecision::Hold);
  EXPECT_EQ(state.update(0.01, 0.0, 0.0, 1.3, o), PivotBrakeDecision::Hold);
  EXPECT_EQ(state.update(0.04, 0.0, 0.0, 1.4, o), PivotBrakeDecision::Hold);
  EXPECT_EQ(state.update(0.01, 0.0, 0.0, 1.5, o), PivotBrakeDecision::Hold);
  EXPECT_EQ(state.update(0.01, 0.0, 0.0, 1.81, o), PivotBrakeDecision::Aligned);
}

TEST(ForkliftMpcTypes, PivotMustStopTranslationAndWorksInBothDirections)
{
  for (double sign : {-1.0, 1.0}) {
    PivotBrakeState state;
    PivotBrakeOptions o;
    EXPECT_EQ(state.update(sign * .10, sign * .20, 0, 1, o), PivotBrakeDecision::Hold);
    EXPECT_EQ(state.update(sign * .01, 0, .05, 2, o), PivotBrakeDecision::Hold);
    EXPECT_EQ(state.update(sign * .01, 0, 0, 3, o), PivotBrakeDecision::Hold);
    EXPECT_EQ(state.update(sign * .01, 0, 0, 3.31, o), PivotBrakeDecision::Aligned);
  }
}

TEST(ForkliftMpcTypes, AccelerationCapSurvivesLimitReleaseAndDirectionChange)
{
  EXPECT_NEAR(accelerationSpeedLimit(1.3, .08, .08, .5, .1), .13, 1e-9);
  EXPECT_NEAR(accelerationSpeedLimit(-1.3, -.08, -.08, .5, .1), -.13, 1e-9);
  EXPECT_DOUBLE_EQ(accelerationSpeedLimit(-.2, .2, .2, .5, .1), 0.0);
  EXPECT_DOUBLE_EQ(accelerationSpeedLimit(.2, 0, -.1, .5, .1), 0.0);
  EXPECT_DOUBLE_EQ(accelerationSpeedLimit(0, 1.3, 1.3, .5, .1), 0.0);
  EXPECT_DOUBLE_EQ(accelerationSpeedLimit(.1, 1.3, 1.3, .5, .1), .1);
  EXPECT_NEAR(accelerationSpeedLimit(1.3, 0, 0, .5, 5), .1, 1e-9);
}

TEST(ForkliftMpcTypes, CommandRampCrossesDeadbandWithBoundedFeedbackLead)
{
  for (double sign : {-1.0, 1.0}) {
    double command = 0.0;
    double actual = 0.0;
    for (int i = 0; i < 160; ++i) {
      const double next = accelerationSpeedLimit(
        sign * 0.6, command, actual, 0.5, 0.05, 0.5);
      EXPECT_LE(std::abs(next - command), 0.025 + 1e-9);
      command = next;
      // Low-speed deadband plus lag, as seen in the 920 recordings.
      const double response = std::abs(command) < 0.09 ? 0.0 : sign *
        std::max(0.0, std::abs(command) - 0.02);
      actual += 0.2 * (response - actual);
    }
    EXPECT_NEAR(command, sign * 0.6, 1e-6);
  }
  double stalled = 0.0;
  for (int i = 0; i < 160; ++i) {
    stalled = accelerationSpeedLimit(1.9, stalled, 0.0, 0.5, 0.05, 0.5);
  }
  EXPECT_LE(stalled, 0.275 + 1e-9);
  EXPECT_DOUBLE_EQ(accelerationSpeedLimit(0.0, .6, .5, .5, .05, .5), 0.0);
}

TEST(ForkliftMpcTypes, SteeringRampCrossesPositionDeadbandWithoutRunaway)
{
  double command = 0.07;
  double actual = 0.07;
  for (int i = 0; i < 200; ++i) {
    const double next = steeringCommandTarget(0.9, command, actual, .12, .05, .5);
    EXPECT_LE(std::abs(next - command), .006 + 1e-9);
    command = next;
    if (std::abs(command - actual) > .025) {
      actual += .5 * (command - actual);
    }
    EXPECT_LE(std::abs(command - actual), .06 + 1e-9);
  }
  EXPECT_NEAR(command, .9, 1e-9);
  EXPECT_NEAR(actual, .9, .025);
}

TEST(ForkliftMpcTypes, TargetRolloutStopsAccelerationAndSteeringAtSetpoints)
{
  const ForkliftVehicleModel model({1.4, 1.4, .12, 1.9, .5, .8});
  MpcState state;
  for (int i = 0; i < 30; ++i) {
    state = predictMpcStateToTarget(state, {.09, .10}, .2, model);
    EXPECT_LE(state.velocity, .09 + 1e-9);
    EXPECT_LE(state.phi, .10 + 1e-9);
  }
  EXPECT_NEAR(state.velocity, .09, 1e-9);
  EXPECT_NEAR(state.phi, .10, 1e-9);
  for (int i = 0; i < 30; ++i) {
    state = predictMpcStateToTarget(state, {-.09, -.10}, .2, model);
  }
  EXPECT_NEAR(state.velocity, -.09, 1e-9);
  EXPECT_NEAR(state.phi, -.10, 1e-9);
}

TEST(ForkliftMpcTypes, ApproachSpeedCanStopBeforePrimitiveBoundary)
{
  for (double distance : {0.0, 0.1, 0.3, 1.0, 3.0}) {
    const double v = stoppingSpeedLimit(distance, .5, .5);
    EXPECT_LE(v * .5 + v * v / (2 * .5), distance + 1e-9);
  }
  EXPECT_DOUBLE_EQ(stoppingSpeedLimit(-.1, .5, .5), 0.0);
}

TEST(ForkliftMpcTypes, DelayedMinimumSpeedPlantStopsWithBoundedCorrection)
{
  // Nonzero commands share one speed floor, with transport and motor lag.
  for (double target : {-M_PI, -M_PI_2, -.175, -.087, -.035, .035, .087, .175, M_PI_2, M_PI}) {
    PivotBrakeOptions options;
    PivotBrakeState state;
    std::deque<double> delayed(3, 0.0);
    double yaw = 0.0;
    double rate = 0.0;
    bool done = false;
    for (int tick = 0; tick < 400; ++tick) {
      const double error = ForkliftVehicleModel::normalizeAngle(target - yaw);
      const auto decision = state.update(error, rate, 0, tick * .1, options);
      if (decision == PivotBrakeDecision::Aligned || decision == PivotBrakeDecision::Recover) {
        EXPECT_LE(std::abs(rate), options.rate_tolerance);
        EXPECT_LE(std::abs(error), options.recovery_tolerance);
        EXPECT_LE(state.corrections, 1);
        done = true;
        break;
      }
      ASSERT_NE(decision, PivotBrakeDecision::Failed) << target;
      delayed.push_back(decision == PivotBrakeDecision::Drive ? std::copysign(.20, error) : 0.0);
      const double demand = delayed.front();
      delayed.pop_front();
      rate += (demand - rate) * (1.0 - std::exp(-.1 / .35));
      yaw = ForkliftVehicleModel::normalizeAngle(yaw + rate * .1);
    }
    EXPECT_TRUE(done) << target;
  }
}

TEST(PivotHeadingContinuity, HalfTurnNoiseDoesNotReverseDirection)
{
  for (const double sign : {-1.0, 1.0}) {
    double previous = sign * (M_PI - .001);
    PivotBrakeState brake;
    for (int i = 0; i < 100; ++i) {
      const double wrapped = ForkliftVehicleModel::normalizeAngle(
        sign * (M_PI + (i % 2 ? .001 : -.001)));
      previous = continuousPivotError(wrapped, previous);
      EXPECT_GT(sign * previous, 3.0);
      EXPECT_EQ(brake.update(previous, 0., 0., i * .05, {}), PivotBrakeDecision::Drive);
    }
    for (int i = 1; i <= 100; ++i) {
      previous = continuousPivotError(sign * (M_PI * (1.0 - i / 100.0)), previous);
    }
    EXPECT_NEAR(previous, 0., 1e-9);
  }
}

TEST(PivotHeadingContinuity, OvershootStillBrakesAndSettles)
{
  PivotBrakeState brake;
  PivotBrakeOptions o;
  double error = continuousPivotError(.1, .2);
  EXPECT_EQ(brake.update(error, 0., 0., 0., o), PivotBrakeDecision::Drive);
  error = continuousPivotError(-.01, error);
  EXPECT_LT(error, 0.);
  EXPECT_EQ(brake.update(error, .1, 0., .1, o), PivotBrakeDecision::Hold);
  EXPECT_EQ(brake.update(error, 0., 0., .2, o), PivotBrakeDecision::Hold);
  EXPECT_EQ(brake.update(error, 0., 0., .6, o), PivotBrakeDecision::Aligned);
}

}  // namespace
}  // namespace forklift_nav2_plugins
