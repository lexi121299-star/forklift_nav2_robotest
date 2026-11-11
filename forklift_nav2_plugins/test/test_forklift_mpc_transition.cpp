#include "forklift_nav2_plugins/forklift_mpc_controller.hpp"

#include "gtest/gtest.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace forklift_nav2_plugins
{

class ForkliftMpcControllerTestAccess
{
public:
  static void setup(ForkliftMpcController & c)
  {
    c.clock_ = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    c.tf_ = std::make_shared<tf2_ros::Buffer>(c.clock_);
    c.transform_tolerance_ = 0;
    c.use_collision_check_ = false;
    c.new_goal_steering_settle_enabled_ = false;
    c.allow_pivot_turn_ = true;
    c.max_steering_angle_ = M_PI_2;
    c.max_velocity_ = 1.3;
    c.min_velocity_ = .06;
    c.preview_window_points_ = 15;
    c.vehicle_model_.setParameters({1.4, M_PI_2, .12, 1.3, .5, .8, true, M_PI_2, .03, .6});
    MpcTrajectory route;
    MpcTrajectoryPoint p;
    p.speed_limit = 1.3;
    p.pivot_motion = true;
    p.state.theta = -1;
    route.push_back(p);
    p.state.theta = 0;
    route.push_back(p);
    p.pivot_motion = false;
    for (int i = 1; i <= 50; ++i) {
      p.state.x = i * .1;
      route.push_back(p);
    }
    p.state.theta = M_PI_2;
    p.pivot_motion = true;
    route.push_back(p);
    p.pivot_motion = false;
    for (int i = 1; i <= 30; ++i) {
      p.state.y = i * .1;
      route.push_back(p);
    }
    for (std::size_t i = 1; i < route.size(); ++i) {
      route[i].distance = route[i - 1].distance + std::hypot(
        route[i].state.x - route[i - 1].state.x,
        route[i].state.y - route[i - 1].state.y);
    }
    std_msgs::msg::Header header;
    header.frame_id = "map";
    c.global_plan_ = trajectoryToPath(route, header);
    c.global_trajectory_ = route;
    c.completed_pivot_index_ = 1;
    c.pivot_completion_latched_ = true;
    c.completed_pivot_left_preview_ = true;
    c.post_pivot_recovery_active_ = true;
    c.post_pivot_recovery_started_ns_ = c.clock_->now().nanoseconds();
    c.last_navigation_output_velocity_ = .08;
    c.last_navigation_output_ns_ = c.clock_->now().nanoseconds() - 100000000;
  }

  static geometry_msgs::msg::TwistStamped run(
    ForkliftMpcController & c, double x, double y, double heading = 0,
    double speed = .08, double rate = 0, const std::string & frame = "map")
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = frame;
    pose.header.stamp = c.clock_->now();
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    tf2::Quaternion q;
    q.setRPY(0, 0, heading);
    pose.pose.orientation = tf2::toMsg(q);
    geometry_msgs::msg::Twist velocity;
    velocity.linear.x = speed;
    velocity.angular.z = rate;
    return c.computeVelocityCommands(pose, velocity);
  }

  static bool recovering(const ForkliftMpcController & c) {return c.post_pivot_recovery_active_;}
  static bool pivoting(const ForkliftMpcController & c) {return c.pivot_maneuver_active_;}
  static void adjacentPivots(ForkliftMpcController & c) {c.completed_pivot_left_preview_ = false;}
  static void pendingPivotBrakeParameters(
    ForkliftMpcController & c, double activation_distance, double clearance)
  {
    c.pivot_activation_distance_ = activation_distance;
    c.primitive_brake_clearance_m_ = clearance;
    c.primitive_brake_reaction_time_sec_ = .5;
    c.primitive_brake_deceleration_mps2_ = .5;
  }
  static void controllerPeriodElapsed(ForkliftMpcController & c)
  {
    c.last_navigation_output_ns_ = c.clock_->now().nanoseconds() - 50000000;
  }
  static void disableOffsetEntryRecovery(ForkliftMpcController & c)
  {
    c.pivot_entry_offset_recovery_enabled_ = false;
  }
  static void rectangularFootprint(ForkliftMpcController & c, double length, double width)
  {
    c.footprint_.clear();
    for (const auto & xy : std::vector<std::pair<double, double>>{
      {length, width}, {length, -width}, {-length, -width}, {-length, width}})
    {
      geometry_msgs::msg::Point p;
      p.x = xy.first;
      p.y = xy.second;
      c.footprint_.push_back(p);
    }
  }
  static bool offsetEntrySweep(
    const ForkliftMpcController & c, double delta)
  {
    MpcState state;
    state.x = 4.954;
    state.y = .129;
    return c.offsetPivotEntryCollisionFree(state, c.global_trajectory_, 52u, delta);
  }
  static std::pair<double, double> roadProtection(
    ForkliftMpcController & c, const MpcTrajectory & road, const MpcState & state,
    std::size_t end = std::numeric_limits<std::size_t>::max())
  {
    c.safety_gate_enabled_ = true;
    c.safety_slowdown_distance_ = 1.;
    c.safety_sample_spacing_ = .025;
    c.safety_collision_cost_threshold_ = 254;
    end = std::min(end, road.size()-1u);
    const auto projection = projectMpcSegment(road, state, 0u, end);
    return {c.nearestSafetyObstacleDistance(
        state, projection.reverse_motion ? -1. : 1., projection.steering_reference),
      c.nearestTrajectorySafetyObstacleDistance(road, projection, end)};
  }
  static MpcTrajectory curveThenStraight(bool reverse)
  {
    MpcTrajectory road;
    for (int i = 0; i <= 80; ++i) {
      const double s = i*.025;
      const double angle = std::min(s, .2)/.6;
      const double x = .6*std::sin(angle)+std::max(0.,s-.2)*std::cos(angle);
      const double y = .6*(1.-std::cos(angle))+std::max(0.,s-.2)*std::sin(angle);
      MpcTrajectoryPoint p;
      p.state.x = 2.+(reverse ? -x : x);
      p.state.y = 2.+y;
      p.state.theta = reverse ? -angle : angle;
      p.distance = s;
      p.reverse_motion = reverse;
      p.steering_angle = s <= .2 ? std::atan(1.4/.6) : 0.;
      road.push_back(p);
    }
    return road;
  }
  static MpcTrajectory reversingStraight(bool reverse_first, bool repeated_cusp = false)
  {
    MpcTrajectory road;
    const double sign = reverse_first ? -1. : 1.;
    for (int i = 0; i <= 60; ++i) {
      MpcTrajectoryPoint p;
      p.state.x = 2. + sign * .025 * (i <= 20 ? i : 40-i);
      p.state.y = 2.;
      p.distance = i * .025;
      p.reverse_motion = i <= 20 ? reverse_first : !reverse_first;
      p.stop_before_point = i == 21;
      if (repeated_cusp && i == 21) {
        p.state.x = road.back().state.x;
        p.distance = road.back().distance;
      }
      road.push_back(p);
    }
    return road;
  }
  static nav_msgs::msg::Path safetyReference(
    const ForkliftMpcController & c,const MpcTrajectory & road,const MpcState & state)
  {
    const auto projection=projectMpcSegment(road,state,0u,road.size()-1u);
    return c.makeSafetyReference(road,projection,state,road.size()-1u);
  }
  static void immediatePivotHandoff(ForkliftMpcController & c, bool reverse_departure)
  {
    c.pivot_step_enabled_ = false;
    c.pivot_yaw_settle_duration_sec_ = 0.;
    c.post_pivot_steering_settle_duration_sec_ = 0.;
    c.post_pivot_hold_duration_sec_ = 0.;
    c.post_pivot_capture_enabled_ = false;
    c.post_pivot_recovery_enabled_ = false;
    c.allow_reverse_ = true;
    c.max_reverse_velocity_ = .3;
    if (reverse_departure) {
      for (std::size_t i = 52u; i < c.global_trajectory_.size(); ++i) {
        auto & p = c.global_trajectory_[i];
        p.state.theta = -M_PI_2;
        p.body_heading_ref = -M_PI_2;
        p.tangent_yaw = M_PI_2;
        p.reverse_motion = i > 52u;
        p.velocity_reference = -.3;
      }
      c.global_plan_ = trajectoryToPath(c.global_trajectory_, c.global_plan_.header);
    }
  }
  static void expiredWallClock(ForkliftMpcController & c)
  {
    c.post_pivot_recovery_timeout_sec_ = .1;
    c.post_pivot_recovery_started_ns_ = c.clock_->now().nanoseconds() - 30000000000LL;
  }
  static void expiredMotionBudget(ForkliftMpcController & c)
  {
    c.post_pivot_recovery_timeout_sec_ = .1;
    c.post_pivot_recovery_motion_sec_ = .2;
    c.post_pivot_recovery_started_ns_ = c.clock_->now().nanoseconds();
  }
  static void captureTolerance(
    ForkliftMpcController & c, double heading, double lateral)
  {
    c.post_pivot_capture_heading_tolerance_ = heading;
    c.post_pivot_capture_lateral_tolerance_m_ = lateral;
  }
  static bool capturePoseAcceptable(
    const ForkliftMpcController & c, double yaw_error,
    double lateral_error, bool projection_valid = true)
  {
    return c.postPivotCapturePoseAcceptable(
      yaw_error, lateral_error, projection_valid);
  }
  static void collisionMap(ForkliftMpcController & c, nav2_costmap_2d::Costmap2D & map)
  {
    c.costmap_ = &map;
    c.use_collision_check_ = true;
    c.footprint_collision_checker_ = std::make_unique<
      nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>>(&map);
    for (const auto & xy : std::vector<std::pair<double, double>>{
      {.2, .2}, {.2, -.2}, {-.2, -.2}, {-.2, .2}})
    {
      geometry_msgs::msg::Point p;
      p.x = xy.first;
      p.y = xy.second;
      c.footprint_.push_back(p);
    }
  }
  static double target(const ForkliftMpcController & c) {return c.completed_pivot_target_yaw_;}
  static void movingTransition(ForkliftMpcController & c)
  {
    c.post_pivot_transition_active_ = true;
    c.last_steering_angle_ = M_PI_2;
    c.post_pivot_recovery_active_ = false;
  }
  static void initialCurvePresteer(ForkliftMpcController & c, double steering)
  {
    for (auto & point : c.global_trajectory_) {
      point.steering_angle = steering;
    }
    c.new_goal_steering_settle_enabled_ = true;
    c.new_goal_steering_settle_active_ = true;
    c.new_goal_steering_ready_ns_ = 0;
    c.last_steering_angle_ = 0.0;
  }
  static double lastSteering(const ForkliftMpcController & c)
  {
    return c.last_steering_angle_;
  }
  static bool steeringHandoff(const ForkliftMpcController & c)
  {
    return c.new_goal_steering_settle_active_;
  }
  static void replacementStraight(ForkliftMpcController & c, bool interrupt_pivot)
  {
    c.new_goal_steering_settle_enabled_ = true;
    MpcTrajectory route(2);
    route.back().state.x = 5.0;
    std_msgs::msg::Header header;
    header.frame_id = "map";
    const auto path = trajectoryToPath(route, header);
    c.setPlan(path);
    c.new_goal_steering_settle_active_ = false;
    c.pivot_maneuver_active_ = interrupt_pivot;
    c.setPlan(path);
  }
  static void redeliverPlan(ForkliftMpcController & c)
  {
    const auto path = c.global_plan_;
    c.setPlan(path);
  }
  static void halfTurn(ForkliftMpcController & c, double target)
  {
    setup(c);
    c.activate();
    c.pivot_completion_latched_ = false;
    c.completed_pivot_index_ = std::numeric_limits<std::size_t>::max();
    c.post_pivot_recovery_active_ = false;
    c.global_trajectory_.clear();
    MpcTrajectoryPoint p;
    p.pivot_motion = true;
    p.speed_limit = .3;
    c.global_trajectory_.push_back(p);
    p.state.theta = target;
    c.global_trajectory_.push_back(p);
    p.pivot_motion = false;
    for (int i = 1; i < 100; ++i) {
      p.state.x = -.05 * i;
      p.distance = .05 * i;
      c.global_trajectory_.push_back(p);
    }
    std_msgs::msg::Header header;
    header.frame_id = "map";
    c.global_plan_ = trajectoryToPath(c.global_trajectory_, header);
  }
  static double motionBlockCommand(bool reverse, bool solver)
  {
    ForkliftMpcController c;
    setup(c);
    c.activate();
    c.use_mpc_solver_ = solver;
    c.allow_reverse_ = true;
    c.max_reverse_velocity_ = .3;
    c.completed_pivot_index_ = std::numeric_limits<std::size_t>::max();
    c.pivot_completion_latched_ = false;
    c.post_pivot_recovery_active_ = false;
    c.global_trajectory_.clear();
    for (int i = 0; i <= 100; ++i) {
      MpcTrajectoryPoint p;
      p.state.x = (reverse ? -1. : 1.) * i * .05;
      p.distance = i * .05;
      p.reverse_motion = reverse;
      p.tangent_yaw = reverse ? M_PI : 0.;
      p.body_heading_ref = 0.;
      p.speed_limit = .3;
      p.velocity_reference = reverse ? -.3 : .3;
      // A future reverse point must not switch the active forward block.
      if (!reverse && i == 70) {p.reverse_motion = true;}
      c.global_trajectory_.push_back(p);
    }
    std_msgs::msg::Header header;
    header.frame_id = "map";
    c.global_plan_ = trajectoryToPath(c.global_trajectory_, header);
    c.last_navigation_output_velocity_ = 0.;
    return run(c, 0., 0., 0., 0.).twist.linear.x;
  }
  static std::pair<double, double> laggedTracking(double curvature)
  {
    ForkliftMpcController c;
    setup(c);
    c.activate();
    c.max_velocity_ = .6;
    c.max_steering_angle_velocity_ = .12;
    c.min_velocity_ = .09;
    c.path_distance_weight_ = 36.0;
    c.heading_weight_ = 12.0;
    c.steering_reference_weight_ = 4.0;
    c.immediate_steering_reference_weight_ = 10.0;
    c.velocity_reference_weight_ = 10.0;
    c.steering_change_weight_ = 100.0;
    c.last_steering_angle_ = 0.07;
    c.use_steering_feedback_ = true;
    c.has_steering_feedback_ = true;
    c.global_trajectory_.clear();
    for (int i = 0; i <= 240; ++i) {
      const double s = i * .05;
      MpcTrajectoryPoint p;
      p.state.x = curvature == 0 ? s : std::sin(s * curvature) / curvature;
      p.state.y = curvature == 0 ? 0 : (1 - std::cos(s * curvature)) / curvature;
      p.state.theta = s * curvature;
      p.body_heading_ref = p.tangent_yaw = p.state.theta;
      p.distance = s;
      p.curvature = curvature;
      p.steering_angle = std::atan(1.4 * curvature);
      p.speed_limit = p.velocity_reference = curvature == 0 ? .6 : .09;
      c.global_trajectory_.push_back(p);
    }
    std_msgs::msg::Header header;
    header.frame_id = "map";
    c.global_plan_ = trajectoryToPath(c.global_trajectory_, header);
    MpcState actual;
    actual.phi = .07;
    double largest_speed = 0.0;
    double largest_steering = 0.0;
    for (int tick = 0; tick < 180; ++tick) {
      c.measured_steering_angle_ = actual.phi;
      c.steering_feedback_received_ns_ = c.clock_->now().nanoseconds();
      c.last_navigation_output_ns_ = c.clock_->now().nanoseconds() - 50000000;
      const auto output = run(c, actual.x, actual.y, actual.theta, actual.velocity);
      largest_speed = std::max(largest_speed, output.twist.linear.x);
      const double target_speed = output.twist.linear.x < .09 ? 0.0 :
        std::max(0.0, output.twist.linear.x - .02);
      actual.velocity += .2 * (target_speed - actual.velocity);
      if (std::abs(c.last_steering_angle_ - actual.phi) > .025) {
        actual.phi += .5 * (c.last_steering_angle_ - actual.phi);
      }
      largest_steering = std::max(largest_steering, actual.phi);
      const auto next = c.vehicle_model_.predict(
        {actual.x, actual.y, actual.theta, actual.phi},
        {actual.velocity, actual.phi}, .05);
      actual.x = next.x;
      actual.y = next.y;
      actual.theta = next.theta;
    }
    return {largest_speed, largest_steering};
  }
  static void trackingSlowdown(
    ForkliftMpcController & c, double threshold, double full_error,
    double recovery_speed)
  {
    c.cross_track_slowdown_threshold_m_ = threshold;
    c.cross_track_slowdown_full_error_m_ = full_error;
    c.cross_track_recovery_max_speed_mps_ = recovery_speed;
  }
  static double trackingSpeedLimit(
    const ForkliftMpcController & c, double error, double speed)
  {
    return c.trackingErrorSpeedLimit(error, speed);
  }
  static void previewPolicy(
    ForkliftMpcController & c, double minimum, double maximum,
    double margin, double acceleration)
  {
    c.preview_min_distance_ = minimum;
    c.tracking_preview_min_distance_m_ = minimum;
    c.lookahead_distance_ = maximum;
    c.preview_max_distance_ = maximum;
    c.preview_distance_margin_ = margin;
    c.max_acceleration_ = acceleration;
    c.preview_time_sec_ = 2.0;
    c.planned_deceleration_mps2_ = 0.5;
  }
  static double previewDistance(
    const ForkliftMpcController & c, double speed, double maximum_speed)
  {
    return c.dynamicPreviewDistance(speed, maximum_speed);
  }
  static double profilePreviewDistance(const ForkliftMpcController & c)
  {
    return c.profilePreviewDistance();
  }
  static void profilePreviewPolicy(
    ForkliftMpcController & c, double design_speed)
  {
    c.preview_max_distance_ = 6.0;
    c.preview_distance_margin_ = 0.30;
    c.preview_time_sec_ = 2.0;
    c.planned_deceleration_mps2_ = 0.50;
    c.profile_preview_design_speed_mps_ = design_speed;
  }
  static double profileSpeedLimit(
    const ForkliftMpcController & c,
    const MpcPreviewWindow & window,
    double fallback)
  {
    return c.previewSpeedLimit(window, fallback);
  }
  static void steeringTrackingPolicy(
    ForkliftMpcController & c, double tolerance, double full_error,
    double recovery_speed)
  {
    c.steering_reference_tracking_tolerance_ = tolerance;
    c.steering_reference_tracking_full_error_ = full_error;
    c.steering_reference_tracking_max_speed_ = recovery_speed;
  }
  static double steeringTrackingSpeedLimit(
    const ForkliftMpcController & c, double measured, double reference,
    double speed)
  {
    return c.steeringReferenceTrackingSpeedLimit(
      measured, reference, speed);
  }
  static void curveExitPolicy(
    ForkliftMpcController & c, double reference_max, double enter_error,
    double reference_drop)
  {
    c.curve_exit_steering_settle_enabled_ = true;
    c.curve_exit_reference_max_angle_rad_ = reference_max;
    c.curve_exit_steering_enter_error_rad_ = enter_error;
    c.curve_exit_steering_reference_drop_rad_ = reference_drop;
  }
  static bool curveExitReturnRequired(
    const ForkliftMpcController & c, double measured, double reference)
  {
    return c.curveExitSteeringReturnRequired(measured, reference);
  }
  static bool straightCandidateValidWithWall(
    ForkliftMpcController & c, double wall_x, double hard_horizon)
  {
    nav2_costmap_2d::Costmap2D map(100, 100, .05, -1, -1, nav2_costmap_2d::FREE_SPACE);
    unsigned int wall_mx = 0;
    unsigned int ignored_my = 0;
    EXPECT_TRUE(map.worldToMap(wall_x, 0.0, wall_mx, ignored_my));
    for (unsigned int my = 0; my < map.getSizeInCellsY(); ++my) {
      map.setCost(wall_mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
    }
    c.costmap_ = &map;
    c.use_collision_check_ = true;
    c.hard_collision_prediction_horizon_sec_ = hard_horizon;
    c.footprint_collision_checker_ = std::make_unique<
      nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>>(&map);
    c.footprint_.clear();
    for (const auto & xy : std::vector<std::pair<double, double>>{
      {.2, .2}, {.2, -.2}, {-.2, -.2}, {-.2, .2}})
    {
      geometry_msgs::msg::Point point;
      point.x = xy.first;
      point.y = xy.second;
      c.footprint_.push_back(point);
    }

    MpcTrajectory trajectory;
    for (int i = 0; i <= 20; ++i) {
      MpcTrajectoryPoint point;
      point.state.x = .1 * i;
      point.distance = .1 * i;
      point.speed_limit = .5;
      trajectory.push_back(point);
    }
    std_msgs::msg::Header header;
    header.frame_id = "map";
    const auto path = trajectoryToPath(trajectory, header);
    geometry_msgs::msg::Twist velocity;
    velocity.linear.x = .5;
    MpcState start_state;
    start_state.velocity = velocity.linear.x;
    return c.scoreCandidate(
      .5, 0.0, start_state, velocity, trajectory, path, 0u,
      trajectory.size() - 1u).valid;
  }
  static void transform(ForkliftMpcController & c, double heading)
  {
    geometry_msgs::msg::TransformStamped tr;
    tr.header.frame_id = "odom";
    tr.child_frame_id = "map";
    tr.header.stamp = c.clock_->now();
    tf2::Quaternion q;
    q.setRPY(0, 0, heading);
    tr.transform.rotation = tf2::toMsg(q);
    c.tf_->setTransform(tr, "test", true);
  }
};

TEST(ForkliftMpcTransition, NextPivotInPreviewDoesNotClearRecoveryOrReleaseSpeed)
{
  ForkliftMpcController c;
  ForkliftMpcControllerTestAccess::setup(c);
  const auto cmd = ForkliftMpcControllerTestAccess::run(c, 4.0, .10);
  EXPECT_TRUE(ForkliftMpcControllerTestAccess::recovering(c));
  EXPECT_FALSE(ForkliftMpcControllerTestAccess::pivoting(c));
  EXPECT_GT(cmd.twist.linear.x, 0);
  EXPECT_LT(cmd.twist.linear.x, .15);
}

TEST(ForkliftMpcTransition, OutsidePivotEntryCorridorStopsBeforeBoundary)
{
  ForkliftMpcController c;
  ForkliftMpcControllerTestAccess::setup(c);
  try {
    ForkliftMpcControllerTestAccess::run(c, 4.95, .30);
    FAIL() << "Expected an explicit pivot corridor failure";
  } catch (const std::runtime_error & error) {
    EXPECT_NE(std::string(error.what()).find("entry corridor missed"), std::string::npos);
  }
  EXPECT_FALSE(ForkliftMpcControllerTestAccess::pivoting(c));
}

TEST(ForkliftMpcTransition, PendingPivotBrakeReachesActivationBoundary)
{
  ForkliftMpcController c;
  ForkliftMpcControllerTestAccess::setup(c);
  ForkliftMpcControllerTestAccess::pendingPivotBrakeParameters(c, .35, .50);

  // The old clearance-based limit was zero at 0.50 m remaining and parked the
  // vehicle outside the 0.35 m pivot activation radius. The approach must
  // retain a positive speed until the pivot state machine can take over.
  const auto cmd = ForkliftMpcControllerTestAccess::run(c, 4.50, .03);
  EXPECT_FALSE(ForkliftMpcControllerTestAccess::pivoting(c));
  EXPECT_GT(cmd.twist.linear.x, 0.0);
}

TEST(ForkliftMpcTransition, TightPivotEntryDoesNotRotateThirtyFiveCentimetresEarly)
{
  ForkliftMpcController c;
  ForkliftMpcControllerTestAccess::setup(c);
  ForkliftMpcControllerTestAccess::pendingPivotBrakeParameters(c, .10, .50);
  const auto approach = ForkliftMpcControllerTestAccess::run(c, 4.65, .0);
  EXPECT_FALSE(ForkliftMpcControllerTestAccess::pivoting(c));
  EXPECT_GT(approach.twist.linear.x, 0.0);
  ForkliftMpcControllerTestAccess::run(c, 4.92, .0, 0.0, 0.0);
  EXPECT_TRUE(ForkliftMpcControllerTestAccess::pivoting(c));
}

TEST(ForkliftMpcTransition, PivotApproachKeepsMovingWithRecordedLateralOffset)
{
  ForkliftMpcController c;
  ForkliftMpcControllerTestAccess::setup(c);
  ForkliftMpcControllerTestAccess::pendingPivotBrakeParameters(c, .10, .50);
  // 141320: longitudinal remainder .075 m, lateral offset .099 m.
  // This is outside the 2D activation circle despite being inside its radius
  // along the reference line. It must approach, not return zero indefinitely.
  const auto cmd = ForkliftMpcControllerTestAccess::run(
    c, 4.925, .099, 0.0, 0.0);
  EXPECT_FALSE(ForkliftMpcControllerTestAccess::pivoting(c));
  EXPECT_GT(cmd.twist.linear.x, 0.0);
}

TEST(ForkliftMpcTransition, PivotApproachKeepsMovingWithFiveCentimetreOffset)
{
  ForkliftMpcController c;
  ForkliftMpcControllerTestAccess::setup(c);
  ForkliftMpcControllerTestAccess::pendingPivotBrakeParameters(c, .10, .50);
  // The preceding run stalled at .097 m longitudinal and .052 m lateral.
  const auto cmd = ForkliftMpcControllerTestAccess::run(
    c, 4.903, .052, 0.0, 0.0);
  EXPECT_FALSE(ForkliftMpcControllerTestAccess::pivoting(c));
  EXPECT_GT(cmd.twist.linear.x, 0.0);
}

TEST(ForkliftMpcTransition, OffsetPivotApproachTransfersControlOnlyInsideCircle)
{
  ForkliftMpcController c;
  ForkliftMpcControllerTestAccess::setup(c);
  ForkliftMpcControllerTestAccess::pendingPivotBrakeParameters(c, .10, .50);
  for (double x : {4.925, 4.950, 4.975}) {
    ForkliftMpcControllerTestAccess::controllerPeriodElapsed(c);
    const auto cmd = ForkliftMpcControllerTestAccess::run(c, x, .099, 0.0, 0.0);
    EXPECT_FALSE(ForkliftMpcControllerTestAccess::pivoting(c));
    EXPECT_GT(cmd.twist.linear.x, 0.0);
  }
  ForkliftMpcControllerTestAccess::run(c, 4.995, .099, 0.0, 0.0);
  EXPECT_TRUE(ForkliftMpcControllerTestAccess::pivoting(c));
}

TEST(ForkliftMpcTransition, MissedPivotCircleReportsFailureInsteadOfPermanentStop)
{
  ForkliftMpcController c;
  ForkliftMpcControllerTestAccess::setup(c);
  ForkliftMpcControllerTestAccess::pendingPivotBrakeParameters(c, .10, .50);
  try {
    ForkliftMpcControllerTestAccess::run(c, 5.0, .12, 0.0, 0.0);
    FAIL() << "Expected an explicit missed-circle failure";
  } catch (const std::runtime_error & error) {
    EXPECT_NE(std::string(error.what()).find("pivot entry centre missed"), std::string::npos);
  }
  EXPECT_FALSE(ForkliftMpcControllerTestAccess::pivoting(c));
}

TEST(ForkliftMpcTransition, RecordedThirteenCentimetreOffsetCanEnterValidatedPivot)
{
  for (double lateral : {-.129, .129}) {
    ForkliftMpcController c;
    ForkliftMpcControllerTestAccess::setup(c);
    ForkliftMpcControllerTestAccess::pendingPivotBrakeParameters(c, .10, .50);
    nav2_costmap_2d::Costmap2D map(600, 600, .02, -1., -2., 0);
    ForkliftMpcControllerTestAccess::collisionMap(c, map);
    const auto cmd = ForkliftMpcControllerTestAccess::run(c, 4.954, lateral, -.029, 0.);
    EXPECT_TRUE(ForkliftMpcControllerTestAccess::pivoting(c));
    EXPECT_DOUBLE_EQ(cmd.twist.linear.x, 0.);
  }
}

TEST(ForkliftMpcTransition, OffsetPivotEntryBrakesBeforeValidatingActualCentre)
{
  ForkliftMpcController c;
  ForkliftMpcControllerTestAccess::setup(c);
  ForkliftMpcControllerTestAccess::pendingPivotBrakeParameters(c, .10, .50);
  nav2_costmap_2d::Costmap2D map(600, 600, .02, -1., -2., 0);
  ForkliftMpcControllerTestAccess::collisionMap(c, map);
  const auto cmd = ForkliftMpcControllerTestAccess::run(c, 4.954, .129, -.029, .07);
  EXPECT_FALSE(ForkliftMpcControllerTestAccess::pivoting(c));
  EXPECT_DOUBLE_EQ(cmd.twist.linear.x, 0.);
}

TEST(ForkliftMpcTransition, OffsetPivotRecoveryDoesNotRotateEarly)
{
  ForkliftMpcController c;
  ForkliftMpcControllerTestAccess::setup(c);
  ForkliftMpcControllerTestAccess::pendingPivotBrakeParameters(c, .10, .50);
  nav2_costmap_2d::Costmap2D map(600, 600, .02, -1., -2., 0);
  ForkliftMpcControllerTestAccess::collisionMap(c, map);
  const auto cmd = ForkliftMpcControllerTestAccess::run(c, 4.80, .129, 0., 0.);
  EXPECT_FALSE(ForkliftMpcControllerTestAccess::pivoting(c));
  EXPECT_GT(cmd.twist.linear.x, 0.);
}

TEST(ForkliftMpcTransition, OffsetPivotRecoveryCannotExceedLateralCorridor)
{
  ForkliftMpcController c;
  ForkliftMpcControllerTestAccess::setup(c);
  ForkliftMpcControllerTestAccess::pendingPivotBrakeParameters(c, .10, .50);
  nav2_costmap_2d::Costmap2D map(600, 600, .02, -1., -2., 0);
  ForkliftMpcControllerTestAccess::collisionMap(c, map);
  EXPECT_THROW(ForkliftMpcControllerTestAccess::run(c, 4.99, .16, 0., 0.), std::runtime_error);
  EXPECT_FALSE(ForkliftMpcControllerTestAccess::pivoting(c));
}

TEST(ForkliftMpcTransition, OffsetPivotRecoveryCanBeDisabled)
{
  ForkliftMpcController c;
  ForkliftMpcControllerTestAccess::setup(c);
  ForkliftMpcControllerTestAccess::pendingPivotBrakeParameters(c, .10, .50);
  ForkliftMpcControllerTestAccess::disableOffsetEntryRecovery(c);
  nav2_costmap_2d::Costmap2D map(600, 600, .02, -1., -2., 0);
  ForkliftMpcControllerTestAccess::collisionMap(c, map);
  EXPECT_THROW(ForkliftMpcControllerTestAccess::run(c, 5., .129, 0., 0.), std::runtime_error);
  EXPECT_FALSE(ForkliftMpcControllerTestAccess::pivoting(c));
}

TEST(ForkliftMpcTransition, OffsetPivotRejectsObstacleOnlyInsideIntermediateSweep)
{
  ForkliftMpcController c;
  ForkliftMpcControllerTestAccess::setup(c);
  ForkliftMpcControllerTestAccess::pendingPivotBrakeParameters(c, .10, .50);
  nav2_costmap_2d::Costmap2D map(600, 600, .02, -1., -2., 0);
  ForkliftMpcControllerTestAccess::collisionMap(c, map);
  ForkliftMpcControllerTestAccess::rectangularFootprint(c, .8, .2);
  unsigned int x, y;
  ASSERT_TRUE(map.worldToMap(5.52, .69, x, y));
  map.setCost(x, y, nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_FALSE(ForkliftMpcControllerTestAccess::offsetEntrySweep(c, M_PI_2));
  EXPECT_THROW(ForkliftMpcControllerTestAccess::run(c, 4.954, .129, 0., 0.), std::runtime_error);
  EXPECT_FALSE(ForkliftMpcControllerTestAccess::pivoting(c));
}

TEST(ForkliftMpcTransition, OffsetPivotRejectsBlockedDeparture)
{
  ForkliftMpcController c;
  ForkliftMpcControllerTestAccess::setup(c);
  ForkliftMpcControllerTestAccess::pendingPivotBrakeParameters(c, .10, .50);
  nav2_costmap_2d::Costmap2D map(600, 600, .02, -1., -2., 0);
  ForkliftMpcControllerTestAccess::collisionMap(c, map);
  unsigned int x, y;
  ASSERT_TRUE(map.worldToMap(4.954, .68, x, y));
  map.setCost(x, y, nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_FALSE(ForkliftMpcControllerTestAccess::offsetEntrySweep(c, M_PI_2));
  EXPECT_THROW(ForkliftMpcControllerTestAccess::run(c, 4.954, .129, 0., 0.), std::runtime_error);
  EXPECT_FALSE(ForkliftMpcControllerTestAccess::pivoting(c));
}

TEST(ForkliftMpcTransition, OffsetPivotFullSweepRespectsLongRotationDirection)
{
  ForkliftMpcController c;
  ForkliftMpcControllerTestAccess::setup(c);
  nav2_costmap_2d::Costmap2D map(600, 600, .02, -1., -2., 0);
  ForkliftMpcControllerTestAccess::collisionMap(c, map);
  ForkliftMpcControllerTestAccess::rectangularFootprint(c, .8, .2);
  unsigned int x, y;
  ASSERT_TRUE(map.worldToMap(5.52, .69, x, y));
  map.setCost(x, y, nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_TRUE(ForkliftMpcControllerTestAccess::offsetEntrySweep(c, -.25 * M_PI));
  EXPECT_FALSE(ForkliftMpcControllerTestAccess::offsetEntrySweep(c, 1.75 * M_PI));
}

TEST(ForkliftMpcTransition, OffsetPivotRecoveryRequiresCostmapCoverage)
{
  ForkliftMpcController c;
  ForkliftMpcControllerTestAccess::setup(c);
  ForkliftMpcControllerTestAccess::pendingPivotBrakeParameters(c, .10, .50);
  nav2_costmap_2d::Costmap2D map(300, 300, .02, -1., -2., 0);
  ForkliftMpcControllerTestAccess::collisionMap(c, map);
  EXPECT_FALSE(ForkliftMpcControllerTestAccess::offsetEntrySweep(c, M_PI_2));
  EXPECT_THROW(ForkliftMpcControllerTestAccess::run(c, 4.954, .129, 0., 0.), std::runtime_error);
  EXPECT_FALSE(ForkliftMpcControllerTestAccess::pivoting(c));
}

TEST(ForkliftMpcTransition, ValidatedOffsetPivotContinuesForwardAndReverseDeparture)
{
  for (bool reverse : {false, true}) {
    ForkliftMpcController c;
    ForkliftMpcControllerTestAccess::setup(c);
    ForkliftMpcControllerTestAccess::pendingPivotBrakeParameters(c, .10, .50);
    ForkliftMpcControllerTestAccess::immediatePivotHandoff(c, reverse);
    nav2_costmap_2d::Costmap2D map(600, 600, .02, -1., -2., 0);
    ForkliftMpcControllerTestAccess::collisionMap(c, map);
    ForkliftMpcControllerTestAccess::run(c, 4.954, .129, 0., 0.);
    ASSERT_TRUE(ForkliftMpcControllerTestAccess::pivoting(c));
    const double heading = reverse ? -M_PI_2 : M_PI_2;
    ForkliftMpcControllerTestAccess::run(c, 4.954, .129, heading, 0.);
    ASSERT_FALSE(ForkliftMpcControllerTestAccess::pivoting(c));
    ForkliftMpcControllerTestAccess::controllerPeriodElapsed(c);
    const auto cmd = ForkliftMpcControllerTestAccess::run(c, 4.954, .129, heading, 0.);
    EXPECT_GT(cmd.twist.linear.x * (reverse ? -1. : 1.), 0.);
  }
}

TEST(ForkliftMpcTransition, RoadProtectionDoesNotExtendTightCurveAfterRoadStraightens)
{
  for (bool reverse : {false, true}) {
    ForkliftMpcController c;
    ForkliftMpcControllerTestAccess::setup(c);
    nav2_costmap_2d::Costmap2D map(250, 250, .02, 0., 0., 0);
    ForkliftMpcControllerTestAccess::collisionMap(c, map);
    ForkliftMpcControllerTestAccess::rectangularFootprint(c, .08, .08);
    unsigned int x, y;
    const double arc_x = .6 * std::sin(.7 / .6);
    const double arc_y = .6 * (1. - std::cos(.7 / .6));
    ASSERT_TRUE(map.worldToMap(2.+(reverse ? -arc_x : arc_x), 2.+arc_y, x, y));
    map.setCost(x, y, nav2_costmap_2d::LETHAL_OBSTACLE);
    const auto road = ForkliftMpcControllerTestAccess::curveThenStraight(reverse);
    MpcState state;
    state.x = state.y = 2.;
    const auto result = ForkliftMpcControllerTestAccess::roadProtection(c, road, state);
    EXPECT_LT(result.first, .80);
    EXPECT_TRUE(std::isinf(result.second));
  }
}

TEST(ForkliftMpcTransition, RoadProtectionPreservesActualLateralOffset)
{
  ForkliftMpcController c;
  ForkliftMpcControllerTestAccess::setup(c);
  nav2_costmap_2d::Costmap2D map(250, 250, .02, 0., 0., 0);
  ForkliftMpcControllerTestAccess::collisionMap(c, map);
  unsigned int x, y;
  ASSERT_TRUE(map.worldToMap(2.5, 2.32, x, y));
  map.setCost(x, y, nav2_costmap_2d::LETHAL_OBSTACLE);
  MpcTrajectory road;
  for (int i = 0; i <= 80; ++i) {
    MpcTrajectoryPoint p;
    p.state.x = 2.+i*.025;
    p.state.y = 2.;
    p.distance = i*.025;
    road.push_back(p);
  }
  MpcState state;
  state.x = state.y = 2.;
  EXPECT_TRUE(std::isinf(ForkliftMpcControllerTestAccess::roadProtection(c, road, state).second));
  state.y += .15;
  EXPECT_LT(ForkliftMpcControllerTestAccess::roadProtection(c, road, state).second, .6);
}

TEST(ForkliftMpcTransition, RoadProtectionStopsAtCurrentMotionBlockBoundary)
{
  ForkliftMpcController c;
  ForkliftMpcControllerTestAccess::setup(c);
  nav2_costmap_2d::Costmap2D map(250, 250, .02, 0., 0., 0);
  ForkliftMpcControllerTestAccess::collisionMap(c, map);
  unsigned int x, y;
  ASSERT_TRUE(map.worldToMap(2.8, 2., x, y));
  map.setCost(x, y, nav2_costmap_2d::LETHAL_OBSTACLE);
  MpcTrajectory road;
  for (int i = 0; i <= 40; ++i) {
    MpcTrajectoryPoint p;
    p.state.x = 2.+i*.025;
    p.state.y = 2.;
    p.distance = i*.025;
    road.push_back(p);
  }
  MpcState state;
  state.x = state.y = 2.;
  EXPECT_TRUE(std::isinf(ForkliftMpcControllerTestAccess::roadProtection(c, road, state, 16u).second));
  EXPECT_LT(ForkliftMpcControllerTestAccess::roadProtection(c, road, state).second, 1.);
}

TEST(ForkliftMpcTransition, RoadProtectionDoesNotTreatDirectionChangeAsObstacle)
{
  for (bool reverse_first : {false, true}) {
    for (bool repeated_cusp : {false, true}) {
      ForkliftMpcController c;
      ForkliftMpcControllerTestAccess::setup(c);
      nav2_costmap_2d::Costmap2D map(250, 250, .02, 0., 0., 0);
      ForkliftMpcControllerTestAccess::collisionMap(c, map);
      ForkliftMpcControllerTestAccess::rectangularFootprint(c, .04, .04);
      const auto road = ForkliftMpcControllerTestAccess::reversingStraight(
        reverse_first, repeated_cusp);
      MpcState state;
      state.x = state.y = 2.;
      EXPECT_TRUE(std::isinf(
          ForkliftMpcControllerTestAccess::roadProtection(c, road, state).second))
        << "reverse_first=" << reverse_first << " repeated_cusp=" << repeated_cusp;
    }
  }
}

TEST(ForkliftMpcTransition, PublishedSafetyReferenceRetainsPoseErrorAndStopsAtCusp)
{
  ForkliftMpcController c;
  ForkliftMpcControllerTestAccess::setup(c);
  const auto road=ForkliftMpcControllerTestAccess::reversingStraight(false);
  MpcState state;state.x=2.;state.y=2.07;state.theta=.03;
  const auto reference=ForkliftMpcControllerTestAccess::safetyReference(c,road,state);
  ASSERT_GT(reference.poses.size(),2u);
  EXPECT_DOUBLE_EQ(reference.poses.front().pose.position.x,state.x);
  EXPECT_DOUBLE_EQ(reference.poses.front().pose.position.y,state.y);
  for (const auto & p:reference.poses) {
    EXPECT_NEAR(p.pose.position.y,state.y,1e-6);
    EXPECT_LE(p.pose.position.x,2.5+1e-6);
    EXPECT_NEAR(tf2::getYaw(p.pose.orientation),state.theta,1e-6);
  }
}

TEST(ForkliftMpcTransition, RoadProtectionStillChecksObstaclesBeforeDirectionChange)
{
  for (bool reverse_first : {false, true}) {
    ForkliftMpcController c;
    ForkliftMpcControllerTestAccess::setup(c);
    nav2_costmap_2d::Costmap2D map(500, 500, .01, 0., 0., 0);
    ForkliftMpcControllerTestAccess::collisionMap(c, map);
    ForkliftMpcControllerTestAccess::rectangularFootprint(c, .04, .04);
    const auto road = ForkliftMpcControllerTestAccess::reversingStraight(reverse_first);
    unsigned int x, y;
    ASSERT_TRUE(map.worldToMap(2. + (reverse_first ? -.3 : .3), 2., x, y));
    map.setCost(x, y, nav2_costmap_2d::LETHAL_OBSTACLE);
    MpcState state;
    state.x = state.y = 2.;
    EXPECT_LT(ForkliftMpcControllerTestAccess::roadProtection(c, road, state).second, .4);
  }
}

TEST(ForkliftMpcTransition, RoadProtectionChecksIncomingBoundaryFootprint)
{
  for (bool reverse_first : {false, true}) {
    ForkliftMpcController c;
    ForkliftMpcControllerTestAccess::setup(c);
    nav2_costmap_2d::Costmap2D map(500, 500, .01, 0., 0., 0);
    ForkliftMpcControllerTestAccess::collisionMap(c, map);
    ForkliftMpcControllerTestAccess::rectangularFootprint(c, .04, .04);
    const auto road = ForkliftMpcControllerTestAccess::reversingStraight(reverse_first);
    unsigned int x, y;
    ASSERT_TRUE(map.worldToMap(2. + (reverse_first ? -.5 : .5), 2., x, y));
    map.setCost(x, y, nav2_costmap_2d::LETHAL_OBSTACLE);
    MpcState state;
    state.x = state.y = 2.;
    EXPECT_LE(ForkliftMpcControllerTestAccess::roadProtection(c, road, state).second, .5);
  }
}

TEST(ForkliftMpcTransition, RoadProtectionChecksOutgoingBlockAfterDirectionChanges)
{
  for (bool reverse_first : {false, true}) {
    ForkliftMpcController c;
    ForkliftMpcControllerTestAccess::setup(c);
    nav2_costmap_2d::Costmap2D map(500, 500, .01, 0., 0., 0);
    ForkliftMpcControllerTestAccess::collisionMap(c, map);
    ForkliftMpcControllerTestAccess::rectangularFootprint(c, .04, .04);
    const auto road = ForkliftMpcControllerTestAccess::reversingStraight(reverse_first);
    unsigned int x, y;
    ASSERT_TRUE(map.worldToMap(2. + (reverse_first ? .2 : -.2), 2., x, y));
    map.setCost(x, y, nav2_costmap_2d::LETHAL_OBSTACLE);
    MpcState state;
    state.x = state.y = 2.;
    EXPECT_TRUE(std::isinf(
        ForkliftMpcControllerTestAccess::roadProtection(c, road, state).second));
    state.x += reverse_first ? .05 : -.05;
    EXPECT_LT(ForkliftMpcControllerTestAccess::roadProtection(c, road, state).second, .25);
  }
}

TEST(ForkliftMpcTransition, RoadProtectionAtRepeatedCuspChecksActualFootprint)
{
  for (bool reverse_first : {false, true}) {
    ForkliftMpcController c;
    ForkliftMpcControllerTestAccess::setup(c);
    nav2_costmap_2d::Costmap2D map(500, 500, .01, 0., 0., 0);
    ForkliftMpcControllerTestAccess::collisionMap(c, map);
    ForkliftMpcControllerTestAccess::rectangularFootprint(c, .04, .04);
    const auto road = ForkliftMpcControllerTestAccess::reversingStraight(reverse_first, true);
    MpcState state;
    state.x = 2. + (reverse_first ? -.5 : .5);
    state.y = 2.;
    EXPECT_TRUE(std::isinf(
        ForkliftMpcControllerTestAccess::roadProtection(c, road, state).second));
    unsigned int x, y;
    ASSERT_TRUE(map.worldToMap(state.x + .04, state.y, x, y));
    map.setCost(x, y, nav2_costmap_2d::LETHAL_OBSTACLE);
    EXPECT_DOUBLE_EQ(ForkliftMpcControllerTestAccess::roadProtection(c, road, state).second, 0.);
  }
}

TEST(ForkliftMpcTransition, SteeringReturnCannotReleaseWhileBodyRotates)
{
  ForkliftMpcController c;
  ForkliftMpcControllerTestAccess::setup(c);
  ForkliftMpcControllerTestAccess::movingTransition(c);
  const auto cmd = ForkliftMpcControllerTestAccess::run(c, 0, 0, 0, 0, .12);
  EXPECT_DOUBLE_EQ(cmd.twist.linear.x, 0);
  EXPECT_FALSE(ForkliftMpcControllerTestAccess::recovering(c));
}

TEST(ForkliftMpcTransition, NewGoalPresteersToInitialCurveBeforeDriving)
{
  ForkliftMpcController c;
  ForkliftMpcControllerTestAccess::setup(c);
  ForkliftMpcControllerTestAccess::initialCurvePresteer(c, .7);
  const auto cmd = ForkliftMpcControllerTestAccess::run(c, 2.0, 0.0, 0.0, 0.0);
  EXPECT_DOUBLE_EQ(cmd.twist.linear.x, 0.0);
  EXPECT_NEAR(ForkliftMpcControllerTestAccess::lastSteering(c), .7, 1e-9);
}

TEST(ForkliftMpcTransition, StraightTrackingAcceleratesThroughActuatorLag)
{
  const auto result = ForkliftMpcControllerTestAccess::laggedTracking(0.0);
  EXPECT_GT(result.first, .5);
}

TEST(ForkliftMpcTransition, ReverseCommandsRemainReverseForSolverAndFallback)
{
  EXPECT_LT(ForkliftMpcControllerTestAccess::motionBlockCommand(true, true), 0.);
  EXPECT_LT(ForkliftMpcControllerTestAccess::motionBlockCommand(true, false), 0.);
}

TEST(ForkliftMpcTransition, FutureReversePointDoesNotChangeCurrentDirection)
{
  EXPECT_GT(ForkliftMpcControllerTestAccess::motionBlockCommand(false, true), 0.);
  EXPECT_GT(ForkliftMpcControllerTestAccess::motionBlockCommand(false, false), 0.);
}

TEST(ForkliftMpcTransition, InterruptedPivotReplanKeepsZeroSpeedSteeringHandoff)
{
  ForkliftMpcController c;
  ForkliftMpcControllerTestAccess::setup(c);
  ForkliftMpcControllerTestAccess::replacementStraight(c, true);
  ASSERT_TRUE(ForkliftMpcControllerTestAccess::steeringHandoff(c));
  ForkliftMpcControllerTestAccess::redeliverPlan(c);
  EXPECT_TRUE(ForkliftMpcControllerTestAccess::steeringHandoff(c));
}

TEST(ForkliftMpcTransition, SameGoalNormalTrackingDoesNotRearmSteeringHandoff)
{
  ForkliftMpcController c;
  ForkliftMpcControllerTestAccess::setup(c);
  ForkliftMpcControllerTestAccess::replacementStraight(c, false);
  EXPECT_FALSE(ForkliftMpcControllerTestAccess::steeringHandoff(c));
}

TEST(ForkliftMpcTransition, CurveTrackingDoesNotStallAtFeedbackPlusOneStep)
{
  const auto result = ForkliftMpcControllerTestAccess::laggedTracking(.8);
  EXPECT_GT(result.second, .4);
}

TEST(ForkliftMpcTransition, CrossTrackErrorProgressivelyLimitsRecoverySpeed)
{
  ForkliftMpcController c;
  ForkliftMpcControllerTestAccess::setup(c);
  ForkliftMpcControllerTestAccess::trackingSlowdown(c, .10, .30, .15);

  EXPECT_DOUBLE_EQ(
    ForkliftMpcControllerTestAccess::trackingSpeedLimit(c, .05, .90), .90);
  EXPECT_NEAR(
    ForkliftMpcControllerTestAccess::trackingSpeedLimit(c, .20, .90), .525, 1e-9);
  EXPECT_DOUBLE_EQ(
    ForkliftMpcControllerTestAccess::trackingSpeedLimit(c, .35, .90), .15);
  EXPECT_DOUBLE_EQ(
    ForkliftMpcControllerTestAccess::trackingSpeedLimit(c, -.35, .90), .15);
}

TEST(ForkliftMpcTransition, DynamicPreviewExpandsWithReachableSpeed)
{
  ForkliftMpcController c;
  ForkliftMpcControllerTestAccess::setup(c);
  ForkliftMpcControllerTestAccess::previewPolicy(c, .75, 2.0, .30, .50);

  EXPECT_NEAR(
    ForkliftMpcControllerTestAccess::previewDistance(c, 0.0, .90),
    2.0, 1e-9);
  EXPECT_NEAR(
    ForkliftMpcControllerTestAccess::previewDistance(c, .90, .90),
    2.0, 1e-9);

  ForkliftMpcControllerTestAccess::previewPolicy(c, .75, 6.0, .30, .50);
  EXPECT_NEAR(
    ForkliftMpcControllerTestAccess::previewDistance(c, 2.0, 2.0),
    5.3, 1e-9);
}

TEST(ForkliftMpcTransition, ProfilePreviewUsesIndependentDesignSpeed)
{
  ForkliftMpcController c;
  ForkliftMpcControllerTestAccess::setup(c);
  ForkliftMpcControllerTestAccess::profilePreviewPolicy(c, 1.90);

  EXPECT_NEAR(
    ForkliftMpcControllerTestAccess::profilePreviewDistance(c), 4.91, 1e-9);
}

TEST(ForkliftMpcTransition, TrackingPreviewKeepsConfiguredLowSpeedFloor)
{
  ForkliftMpcController c;
  ForkliftMpcControllerTestAccess::setup(c);
  ForkliftMpcControllerTestAccess::previewPolicy(c, 3.80, 6.0, .30, .50);

  EXPECT_NEAR(
    ForkliftMpcControllerTestAccess::previewDistance(c, 0.20, 1.10),
    3.80, 1e-9);
}

TEST(ForkliftMpcTransition, DistantCurveUsesBackwardProfileInsteadOfImmediateLimit)
{
  ForkliftMpcController c;
  ForkliftMpcControllerTestAccess::setup(c);
  MpcPreviewWindow window;
  window.valid = true;
  for (const auto & sample : std::vector<std::pair<double, double>>{
      {0.0, 1.2}, {0.1, 1.2}, {5.0, 0.6}})
  {
    MpcTrajectoryPoint point;
    point.distance = sample.first;
    point.speed_limit = sample.second;
    window.points.push_back(point);
  }

  EXPECT_NEAR(
    ForkliftMpcControllerTestAccess::profileSpeedLimit(c, window, 1.2),
    1.2, 1e-9);
  window.points[1].speed_limit = 0.8;
  EXPECT_NEAR(
    ForkliftMpcControllerTestAccess::profileSpeedLimit(c, window, 1.2),
    std::sqrt(0.8 * 0.8 + 2.0 * 0.5 * 0.1), 1e-9);
}

TEST(ForkliftMpcTransition, SteeringReferenceLagLimitsSpeedContinuously)
{
  ForkliftMpcController c;
  ForkliftMpcControllerTestAccess::setup(c);
  ForkliftMpcControllerTestAccess::steeringTrackingPolicy(c, .05, .20, .10);

  EXPECT_DOUBLE_EQ(
    ForkliftMpcControllerTestAccess::steeringTrackingSpeedLimit(
      c, .20, .18, .90), .90);
  EXPECT_NEAR(
    ForkliftMpcControllerTestAccess::steeringTrackingSpeedLimit(
      c, .20, .075, .90), .50, 1e-9);
  EXPECT_DOUBLE_EQ(
    ForkliftMpcControllerTestAccess::steeringTrackingSpeedLimit(
      c, .30, 0.0, .90), .10);
}

TEST(ForkliftMpcTransition, CurveExitStopsOnlyForLargeReturnLagNearStraight)
{
  ForkliftMpcController c;
  ForkliftMpcControllerTestAccess::setup(c);
  ForkliftMpcControllerTestAccess::curveExitPolicy(c, .12, .20, .10);

  EXPECT_TRUE(
    ForkliftMpcControllerTestAccess::curveExitReturnRequired(c, -.60, -.08));
  EXPECT_TRUE(
    ForkliftMpcControllerTestAccess::curveExitReturnRequired(c, -.60, .05));
  EXPECT_FALSE(
    ForkliftMpcControllerTestAccess::curveExitReturnRequired(c, -.22, -.08));
  EXPECT_FALSE(
    ForkliftMpcControllerTestAccess::curveExitReturnRequired(c, -.60, -.30));
}

TEST(ForkliftMpcTransition, DistantConstantSteeringCollisionIsSoftCost)
{
  ForkliftMpcController c;
  ForkliftMpcControllerTestAccess::setup(c);

  EXPECT_TRUE(
    ForkliftMpcControllerTestAccess::straightCandidateValidWithWall(c, .55, .60));
}

TEST(ForkliftMpcTransition, NearTermCollisionRemainsHardReject)
{
  ForkliftMpcController c;
  ForkliftMpcControllerTestAccess::setup(c);

  EXPECT_FALSE(
    ForkliftMpcControllerTestAccess::straightCandidateValidWithWall(c, .35, .60));
}

TEST(ForkliftMpcTransition, ReachingNextPivotTransfersRecoveryEvenWithoutPreviewGap)
{
  ForkliftMpcController c;
  ForkliftMpcControllerTestAccess::setup(c);
  ForkliftMpcControllerTestAccess::adjacentPivots(c);
  const auto cmd = ForkliftMpcControllerTestAccess::run(c, 4.95, .03);
  EXPECT_TRUE(ForkliftMpcControllerTestAccess::pivoting(c));
  EXPECT_FALSE(ForkliftMpcControllerTestAccess::recovering(c));
  EXPECT_DOUBLE_EQ(cmd.twist.linear.x, 0.0);
}

TEST(ForkliftMpcTransition, CachedHeadingFollowsMapOdomTransform)
{
  ForkliftMpcController c;
  ForkliftMpcControllerTestAccess::setup(c);
  ForkliftMpcControllerTestAccess::transform(c, -.2);
  ForkliftMpcControllerTestAccess::run(c, 2 * std::cos(.2), -2 * std::sin(.2), -.2, .08, 0, "odom");
  EXPECT_NEAR(ForkliftMpcControllerTestAccess::target(c), -.2, 1e-9);
  ForkliftMpcControllerTestAccess::transform(c, -.4);
  ForkliftMpcControllerTestAccess::run(c, 2 * std::cos(.4), -2 * std::sin(.4), -.4, .08, 0, "odom");
  EXPECT_NEAR(ForkliftMpcControllerTestAccess::target(c), -.4, 1e-9);
}

TEST(ForkliftMpcTransition, ExternalStopDoesNotConsumeRecoveryMotionBudget)
{
  ForkliftMpcController c;
  ForkliftMpcControllerTestAccess::setup(c);
  ForkliftMpcControllerTestAccess::expiredWallClock(c);
  EXPECT_NO_THROW(ForkliftMpcControllerTestAccess::run(c, 4, .1, 0, 0));
  EXPECT_TRUE(ForkliftMpcControllerTestAccess::recovering(c));
}

TEST(ForkliftMpcTransition, CaptureUsesIndependentLateralTolerance)
{
  ForkliftMpcController c;
  ForkliftMpcControllerTestAccess::setup(c);
  ForkliftMpcControllerTestAccess::captureTolerance(c, .025, .20);

  EXPECT_TRUE(ForkliftMpcControllerTestAccess::capturePoseAcceptable(c, .010, .147));
  EXPECT_FALSE(ForkliftMpcControllerTestAccess::capturePoseAcceptable(c, .026, .147));
  EXPECT_FALSE(ForkliftMpcControllerTestAccess::capturePoseAcceptable(c, .010, .201));
  EXPECT_FALSE(ForkliftMpcControllerTestAccess::capturePoseAcceptable(c, .010, .147, false));
}

TEST(ForkliftMpcTransition, RecoveryMotionTimeoutFallsBackToNormalTracking)
{
  ForkliftMpcController c;
  ForkliftMpcControllerTestAccess::setup(c);
  ForkliftMpcControllerTestAccess::expiredMotionBudget(c);

  EXPECT_NO_THROW(ForkliftMpcControllerTestAccess::run(c, 4.0, .10));
  EXPECT_FALSE(ForkliftMpcControllerTestAccess::recovering(c));
}

TEST(ForkliftMpcTransition, RecoveryStillRejectsCollidingCommands)
{
  ForkliftMpcController c;
  ForkliftMpcControllerTestAccess::setup(c);
  nav2_costmap_2d::Costmap2D map(100, 100, .1, -2, -2, nav2_costmap_2d::LETHAL_OBSTACLE);
  ForkliftMpcControllerTestAccess::collisionMap(c, map);
  EXPECT_THROW(ForkliftMpcControllerTestAccess::run(c, 4, .1), std::runtime_error);
}

TEST(ForkliftMpcTransition, SegmentProjectionIgnoresSamplingAndTransformsConsistently)
{
  MpcTrajectory trajectory(3);
  trajectory[1].state.x = 1.0;
  trajectory[2].state.x = 2.0;
  MpcState pose{.5, .03, 0, 0};
  auto p = projectMpcSegment(trajectory, pose, 0, 2);
  ASSERT_TRUE(p.valid);
  EXPECT_NEAR(p.distance, .03, 1e-9);
  EXPECT_NEAR(p.cross_track, .03, 1e-9);
  EXPECT_NEAR(p.arc_length, .5, 1e-9);
  EXPECT_NEAR(p.heading_error, 0.0, 1e-9);
  EXPECT_NEAR(p.remaining, 1.5, 1e-9);
  const auto transformed = transformMpcTrajectory(trajectory, 3, 4, M_PI_2);
  auto q = projectMpcSegment(transformed, {2.97, 4.5, M_PI_2, 0}, 0, 2);
  EXPECT_NEAR(q.distance, p.distance, 1e-9);
  EXPECT_NEAR(q.cross_track, p.cross_track, 1e-9);
  EXPECT_NEAR(q.heading, M_PI_2, 1e-9);
  EXPECT_NEAR(q.heading_error, 0.0, 1e-9);
  EXPECT_NEAR(q.remaining, p.remaining, 1e-9);
  for (auto & point : trajectory) {
    point.reverse_motion = true;
  }
  const auto reverse = projectMpcSegment(trajectory, pose, 0, 2);
  EXPECT_NEAR(std::abs(reverse.heading), M_PI, 1e-9);
  EXPECT_NEAR(std::abs(reverse.heading_error), M_PI, 1e-9);
}

TEST(ForkliftMpcTransition, SegmentProjectionPreservesGlobalArcLength)
{
  MpcTrajectory trajectory(4);
  for (std::size_t i = 0; i < trajectory.size(); ++i) {
    trajectory[i].state.x = static_cast<double>(i);
    trajectory[i].distance = static_cast<double>(i);
  }

  const auto projection = projectMpcSegment(
    trajectory, {2.5, .1, 0, 0}, 2, 3);

  ASSERT_TRUE(projection.valid);
  EXPECT_NEAR(projection.arc_length, 2.5, 1e-9);
  EXPECT_NEAR(projection.cross_track, .1, 1e-9);
}

TEST(ForkliftMpcTransition, FrenetProjectionWrapsHeadingAcrossPi)
{
  MpcTrajectory trajectory(2);
  trajectory[0].state = {0.0, 0.0, 0.0, 0.0};
  trajectory[1].state = {-1.0, -0.01, 0.0, 0.0};
  trajectory[1].distance = std::hypot(1.0, 0.01);
  const auto projection = projectMpcSegment(
    trajectory, {-.5, .02, -M_PI + .02, 0.0}, 0, 1);
  ASSERT_TRUE(projection.valid);
  EXPECT_LT(std::abs(projection.heading_error), .05);
}

TEST(ForkliftMpcTransition, HalfTurnKeepsSteeringSignDespiteYawNoise)
{
  for (const double sign : {-1., 1.}) {
    ForkliftMpcController c;
    ForkliftMpcControllerTestAccess::halfTurn(c, sign * (M_PI - .001));
    for (int i = 0; i < 20; ++i) {
      ForkliftMpcControllerTestAccess::run(c, 0., 0., i % 2 ? -.002 * sign : 0., 0.);
      EXPECT_TRUE(ForkliftMpcControllerTestAccess::pivoting(c));
      EXPECT_GT(sign * ForkliftMpcControllerTestAccess::lastSteering(c), 1.5);
    }
    // A replacement path chooses its own turn rather than inheriting the old one.
    ForkliftMpcControllerTestAccess::halfTurn(c, -sign * (M_PI - .001));
    ForkliftMpcControllerTestAccess::run(c, 0., 0., 0., 0.);
    EXPECT_LT(sign * ForkliftMpcControllerTestAccess::lastSteering(c), -1.5);
  }
}

}  // namespace forklift_nav2_plugins
