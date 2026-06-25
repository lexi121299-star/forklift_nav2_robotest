/*********************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2010, Rice University
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of Rice University nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES ARE DISCLAIMED.
 *
 * Adapted from OMPL's ReedsSheppStateSpace implementation so the
 * planner core remains dependency-free on ROS 2 Foxy.
 *********************************************************************/

#include "forklift_oru_planner/reeds_shepp.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace forklift_oru_planner
{
namespace reeds_shepp
{
namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;
constexpr double kZero = 10.0 * std::numeric_limits<double>::epsilon();

using Types = std::array<SegmentType, 5>;

constexpr std::array<Types, 18> kPathTypes{{
    {{SegmentType::LEFT, SegmentType::RIGHT, SegmentType::LEFT, SegmentType::NOP,
      SegmentType::NOP}},
    {{SegmentType::RIGHT, SegmentType::LEFT, SegmentType::RIGHT, SegmentType::NOP,
      SegmentType::NOP}},
    {{SegmentType::LEFT, SegmentType::RIGHT, SegmentType::LEFT, SegmentType::RIGHT,
      SegmentType::NOP}},
    {{SegmentType::RIGHT, SegmentType::LEFT, SegmentType::RIGHT, SegmentType::LEFT,
      SegmentType::NOP}},
    {{SegmentType::LEFT, SegmentType::RIGHT, SegmentType::STRAIGHT, SegmentType::LEFT,
      SegmentType::NOP}},
    {{SegmentType::RIGHT, SegmentType::LEFT, SegmentType::STRAIGHT, SegmentType::RIGHT,
      SegmentType::NOP}},
    {{SegmentType::LEFT, SegmentType::STRAIGHT, SegmentType::RIGHT, SegmentType::LEFT,
      SegmentType::NOP}},
    {{SegmentType::RIGHT, SegmentType::STRAIGHT, SegmentType::LEFT, SegmentType::RIGHT,
      SegmentType::NOP}},
    {{SegmentType::LEFT, SegmentType::RIGHT, SegmentType::STRAIGHT, SegmentType::RIGHT,
      SegmentType::NOP}},
    {{SegmentType::RIGHT, SegmentType::LEFT, SegmentType::STRAIGHT, SegmentType::LEFT,
      SegmentType::NOP}},
    {{SegmentType::RIGHT, SegmentType::STRAIGHT, SegmentType::RIGHT, SegmentType::LEFT,
      SegmentType::NOP}},
    {{SegmentType::LEFT, SegmentType::STRAIGHT, SegmentType::LEFT, SegmentType::RIGHT,
      SegmentType::NOP}},
    {{SegmentType::LEFT, SegmentType::STRAIGHT, SegmentType::RIGHT, SegmentType::NOP,
      SegmentType::NOP}},
    {{SegmentType::RIGHT, SegmentType::STRAIGHT, SegmentType::LEFT, SegmentType::NOP,
      SegmentType::NOP}},
    {{SegmentType::LEFT, SegmentType::STRAIGHT, SegmentType::LEFT, SegmentType::NOP,
      SegmentType::NOP}},
    {{SegmentType::RIGHT, SegmentType::STRAIGHT, SegmentType::RIGHT, SegmentType::NOP,
      SegmentType::NOP}},
    {{SegmentType::LEFT, SegmentType::RIGHT, SegmentType::STRAIGHT, SegmentType::LEFT,
      SegmentType::RIGHT}},
    {{SegmentType::RIGHT, SegmentType::LEFT, SegmentType::STRAIGHT, SegmentType::RIGHT,
      SegmentType::LEFT}}
  }};

double mod2pi(double value)
{
  value = std::fmod(value, kTwoPi);
  if (value < -kPi) {
    value += kTwoPi;
  } else if (value > kPi) {
    value -= kTwoPi;
  }
  return value;
}

void polar(double x, double y, double & radius, double & angle)
{
  radius = std::hypot(x, y);
  angle = std::atan2(y, x);
}

Path makePath(
  std::size_t type,
  double t,
  double u,
  double v,
  double w = 0.0,
  double x = 0.0)
{
  Path path;
  path.types = kPathTypes[type];
  path.lengths = {{t, u, v, w, x}};
  for (const double length : path.lengths) {
    path.total_length += std::abs(length);
  }
  path.valid = std::isfinite(path.total_length);
  return path;
}

struct SearchResult
{
  Path best;
  bool allow_reverse{true};
};

void consider(SearchResult & result, const Path & candidate)
{
  if (!candidate.valid) {
    return;
  }
  if (!result.allow_reverse &&
    std::any_of(
      candidate.lengths.begin(), candidate.lengths.end(),
      [](double length) {return length < -kZero;}))
  {
    return;
  }
  if (!result.best.valid || candidate.total_length < result.best.total_length) {
    result.best = candidate;
  }
}

void tauOmega(
  double u, double v, double xi, double eta, double phi,
  double & tau, double & omega)
{
  const double delta = mod2pi(u - v);
  const double a = std::sin(u) - std::sin(delta);
  const double b = std::cos(u) - std::cos(delta) - 1.0;
  const double t1 = std::atan2(eta * a - xi * b, xi * a + eta * b);
  const double t2 = 2.0 * (std::cos(delta) - std::cos(v) - std::cos(u)) + 3.0;
  tau = t2 < 0.0 ? mod2pi(t1 + kPi) : mod2pi(t1);
  omega = mod2pi(tau - u + v - phi);
}

bool lpSpLp(double x, double y, double phi, double & t, double & u, double & v)
{
  polar(x - std::sin(phi), y - 1.0 + std::cos(phi), u, t);
  if (t < -kZero) {
    return false;
  }
  v = mod2pi(phi - t);
  return v >= -kZero;
}

bool lpSpRp(double x, double y, double phi, double & t, double & u, double & v)
{
  double t1 = 0.0;
  double u1 = 0.0;
  polar(x + std::sin(phi), y - 1.0 - std::cos(phi), u1, t1);
  const double squared = u1 * u1;
  if (squared < 4.0) {
    return false;
  }
  u = std::sqrt(squared - 4.0);
  t = mod2pi(t1 + std::atan2(2.0, u));
  v = mod2pi(t - phi);
  return t >= -kZero && v >= -kZero;
}

void csc(double x, double y, double phi, SearchResult & path)
{
  double t = 0.0;
  double u = 0.0;
  double v = 0.0;
  if (lpSpLp(x, y, phi, t, u, v)) {
    consider(path, makePath(14, t, u, v));
  }
  if (lpSpLp(-x, y, -phi, t, u, v)) {
    consider(path, makePath(14, -t, -u, -v));
  }
  if (lpSpLp(x, -y, -phi, t, u, v)) {
    consider(path, makePath(15, t, u, v));
  }
  if (lpSpLp(-x, -y, phi, t, u, v)) {
    consider(path, makePath(15, -t, -u, -v));
  }
  if (lpSpRp(x, y, phi, t, u, v)) {
    consider(path, makePath(12, t, u, v));
  }
  if (lpSpRp(-x, y, -phi, t, u, v)) {
    consider(path, makePath(12, -t, -u, -v));
  }
  if (lpSpRp(x, -y, -phi, t, u, v)) {
    consider(path, makePath(13, t, u, v));
  }
  if (lpSpRp(-x, -y, phi, t, u, v)) {
    consider(path, makePath(13, -t, -u, -v));
  }
}

bool lpRmL(double x, double y, double phi, double & t, double & u, double & v)
{
  double radius = 0.0;
  double theta = 0.0;
  polar(x - std::sin(phi), y - 1.0 + std::cos(phi), radius, theta);
  if (radius > 4.0) {
    return false;
  }
  u = -2.0 * std::asin(0.25 * radius);
  t = mod2pi(theta + 0.5 * u + kPi);
  v = mod2pi(phi - t + u);
  return t >= -kZero && u <= kZero;
}

void ccc(double x, double y, double phi, SearchResult & path)
{
  double t = 0.0;
  double u = 0.0;
  double v = 0.0;
  if (lpRmL(x, y, phi, t, u, v)) {
    consider(path, makePath(0, t, u, v));
  }
  if (lpRmL(-x, y, -phi, t, u, v)) {
    consider(path, makePath(0, -t, -u, -v));
  }
  if (lpRmL(x, -y, -phi, t, u, v)) {
    consider(path, makePath(1, t, u, v));
  }
  if (lpRmL(-x, -y, phi, t, u, v)) {
    consider(path, makePath(1, -t, -u, -v));
  }

  const double xb = x * std::cos(phi) + y * std::sin(phi);
  const double yb = x * std::sin(phi) - y * std::cos(phi);
  if (lpRmL(xb, yb, phi, t, u, v)) {
    consider(path, makePath(0, v, u, t));
  }
  if (lpRmL(-xb, yb, -phi, t, u, v)) {
    consider(path, makePath(0, -v, -u, -t));
  }
  if (lpRmL(xb, -yb, -phi, t, u, v)) {
    consider(path, makePath(1, v, u, t));
  }
  if (lpRmL(-xb, -yb, phi, t, u, v)) {
    consider(path, makePath(1, -v, -u, -t));
  }
}

bool lpRupLumRm(double x, double y, double phi, double & t, double & u, double & v)
{
  const double xi = x + std::sin(phi);
  const double eta = y - 1.0 - std::cos(phi);
  const double rho = 0.25 * (2.0 + std::hypot(xi, eta));
  if (rho > 1.0) {
    return false;
  }
  u = std::acos(rho);
  tauOmega(u, -u, xi, eta, phi, t, v);
  return t >= -kZero && v <= kZero;
}

bool lpRumLumRp(double x, double y, double phi, double & t, double & u, double & v)
{
  const double xi = x + std::sin(phi);
  const double eta = y - 1.0 - std::cos(phi);
  const double rho = (20.0 - xi * xi - eta * eta) / 16.0;
  if (rho < 0.0 || rho > 1.0) {
    return false;
  }
  u = -std::acos(rho);
  if (u < -0.5 * kPi) {
    return false;
  }
  tauOmega(u, u, xi, eta, phi, t, v);
  return t >= -kZero && v >= -kZero;
}

void cccc(double x, double y, double phi, SearchResult & path)
{
  double t = 0.0;
  double u = 0.0;
  double v = 0.0;
  if (lpRupLumRm(x, y, phi, t, u, v)) {
    consider(path, makePath(2, t, u, -u, v));
  }
  if (lpRupLumRm(-x, y, -phi, t, u, v)) {
    consider(path, makePath(2, -t, -u, u, -v));
  }
  if (lpRupLumRm(x, -y, -phi, t, u, v)) {
    consider(path, makePath(3, t, u, -u, v));
  }
  if (lpRupLumRm(-x, -y, phi, t, u, v)) {
    consider(path, makePath(3, -t, -u, u, -v));
  }
  if (lpRumLumRp(x, y, phi, t, u, v)) {
    consider(path, makePath(2, t, u, u, v));
  }
  if (lpRumLumRp(-x, y, -phi, t, u, v)) {
    consider(path, makePath(2, -t, -u, -u, -v));
  }
  if (lpRumLumRp(x, -y, -phi, t, u, v)) {
    consider(path, makePath(3, t, u, u, v));
  }
  if (lpRumLumRp(-x, -y, phi, t, u, v)) {
    consider(path, makePath(3, -t, -u, -u, -v));
  }
}

bool lpRmSmLm(double x, double y, double phi, double & t, double & u, double & v)
{
  double rho = 0.0;
  double theta = 0.0;
  polar(x - std::sin(phi), y - 1.0 + std::cos(phi), rho, theta);
  if (rho < 2.0) {
    return false;
  }
  const double r = std::sqrt(rho * rho - 4.0);
  u = 2.0 - r;
  t = mod2pi(theta + std::atan2(r, -2.0));
  v = mod2pi(phi - 0.5 * kPi - t);
  return t >= -kZero && u <= kZero && v <= kZero;
}

bool lpRmSmRm(double x, double y, double phi, double & t, double & u, double & v)
{
  double rho = 0.0;
  double theta = 0.0;
  polar(-y + 1.0 + std::cos(phi), x + std::sin(phi), rho, theta);
  if (rho < 2.0) {
    return false;
  }
  t = theta;
  u = 2.0 - rho;
  v = mod2pi(t + 0.5 * kPi - phi);
  return t >= -kZero && u <= kZero && v <= kZero;
}

void ccscForward(double x, double y, double phi, SearchResult & path)
{
  double t = 0.0;
  double u = 0.0;
  double v = 0.0;
  if (lpRmSmLm(x, y, phi, t, u, v)) {
    consider(path, makePath(4, t, -0.5 * kPi, u, v));
  }
  if (lpRmSmLm(-x, y, -phi, t, u, v)) {
    consider(path, makePath(4, -t, 0.5 * kPi, -u, -v));
  }
  if (lpRmSmLm(x, -y, -phi, t, u, v)) {
    consider(path, makePath(5, t, -0.5 * kPi, u, v));
  }
  if (lpRmSmLm(-x, -y, phi, t, u, v)) {
    consider(path, makePath(5, -t, 0.5 * kPi, -u, -v));
  }
  if (lpRmSmRm(x, y, phi, t, u, v)) {
    consider(path, makePath(8, t, -0.5 * kPi, u, v));
  }
  if (lpRmSmRm(-x, y, -phi, t, u, v)) {
    consider(path, makePath(8, -t, 0.5 * kPi, -u, -v));
  }
  if (lpRmSmRm(x, -y, -phi, t, u, v)) {
    consider(path, makePath(9, t, -0.5 * kPi, u, v));
  }
  if (lpRmSmRm(-x, -y, phi, t, u, v)) {
    consider(path, makePath(9, -t, 0.5 * kPi, -u, -v));
  }
}

void ccscBackward(double x, double y, double phi, SearchResult & path)
{
  const double xb = x * std::cos(phi) + y * std::sin(phi);
  const double yb = x * std::sin(phi) - y * std::cos(phi);
  double t = 0.0;
  double u = 0.0;
  double v = 0.0;
  if (lpRmSmLm(xb, yb, phi, t, u, v)) {
    consider(path, makePath(6, v, u, -0.5 * kPi, t));
  }
  if (lpRmSmLm(-xb, yb, -phi, t, u, v)) {
    consider(path, makePath(6, -v, -u, 0.5 * kPi, -t));
  }
  if (lpRmSmLm(xb, -yb, -phi, t, u, v)) {
    consider(path, makePath(7, v, u, -0.5 * kPi, t));
  }
  if (lpRmSmLm(-xb, -yb, phi, t, u, v)) {
    consider(path, makePath(7, -v, -u, 0.5 * kPi, -t));
  }
  if (lpRmSmRm(xb, yb, phi, t, u, v)) {
    consider(path, makePath(10, v, u, -0.5 * kPi, t));
  }
  if (lpRmSmRm(-xb, yb, -phi, t, u, v)) {
    consider(path, makePath(10, -v, -u, 0.5 * kPi, -t));
  }
  if (lpRmSmRm(xb, -yb, -phi, t, u, v)) {
    consider(path, makePath(11, v, u, -0.5 * kPi, t));
  }
  if (lpRmSmRm(-xb, -yb, phi, t, u, v)) {
    consider(path, makePath(11, -v, -u, 0.5 * kPi, -t));
  }
}

bool lpRmSLmRp(double x, double y, double phi, double & t, double & u, double & v)
{
  const double xi = x + std::sin(phi);
  const double eta = y - 1.0 - std::cos(phi);
  const double rho = std::hypot(xi, eta);
  if (rho < 2.0) {
    return false;
  }
  u = 4.0 - std::sqrt(rho * rho - 4.0);
  if (u > kZero) {
    return false;
  }
  t = mod2pi(std::atan2((4.0 - u) * xi - 2.0 * eta, -2.0 * xi + (u - 4.0) * eta));
  v = mod2pi(t - phi);
  return t >= -kZero && v >= -kZero;
}

void ccscc(double x, double y, double phi, SearchResult & path)
{
  double t = 0.0;
  double u = 0.0;
  double v = 0.0;
  if (lpRmSLmRp(x, y, phi, t, u, v)) {
    consider(path, makePath(16, t, -0.5 * kPi, u, -0.5 * kPi, v));
  }
  if (lpRmSLmRp(-x, y, -phi, t, u, v)) {
    consider(path, makePath(16, -t, 0.5 * kPi, -u, 0.5 * kPi, -v));
  }
  if (lpRmSLmRp(x, -y, -phi, t, u, v)) {
    consider(path, makePath(17, t, -0.5 * kPi, u, -0.5 * kPi, v));
  }
  if (lpRmSLmRp(-x, -y, phi, t, u, v)) {
    consider(path, makePath(17, -t, 0.5 * kPi, -u, 0.5 * kPi, -v));
  }
}

}  // namespace

Path shortestPath(
  double start_x,
  double start_y,
  double start_yaw,
  double goal_x,
  double goal_y,
  double goal_yaw,
  double turning_radius,
  bool allow_reverse)
{
  if (!(turning_radius > 0.0) || !std::isfinite(turning_radius)) {
    return {};
  }

  const double dx = goal_x - start_x;
  const double dy = goal_y - start_y;
  const double cosine = std::cos(start_yaw);
  const double sine = std::sin(start_yaw);
  const double x = (cosine * dx + sine * dy) / turning_radius;
  const double y = (-sine * dx + cosine * dy) / turning_radius;
  const double phi = goal_yaw - start_yaw;

  SearchResult result;
  result.allow_reverse = allow_reverse;
  csc(x, y, phi, result);
  ccc(x, y, phi, result);
  cccc(x, y, phi, result);
  ccscForward(x, y, phi, result);
  ccscBackward(x, y, phi, result);
  ccscc(x, y, phi, result);
  return result.best;
}

}  // namespace reeds_shepp
}  // namespace forklift_oru_planner
