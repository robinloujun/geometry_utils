/*
geometry_utils: Utility library to provide common geometry types and transformations
Copyright (C) 2013  Nathan Michael
              2015  Erik Nelson

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 2
of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
*/

#ifndef GEOMETRY_UTILS_ROS_H
#define GEOMETRY_UTILS_ROS_H

#include "GeometryUtils.h"

#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Transform.h>
#include <nav_msgs/Odometry.h>

namespace geometry_utils {
namespace ros {

inline Vec3f FromROS(const geometry_msgs::Point& p) {
  return Vec3f(p.x, p.y, p.z);
}

inline Vec3f FromROS(const geometry_msgs::Point32& p) {
  return Vec3f(p.x, p.y, p.z);
}

inline Vec3f FromROS(const geometry_msgs::Vector3& p) {
  return Vec3f(p.x, p.y, p.z);
}

inline Quaternionf FromROS(const geometry_msgs::Quaternion& msg) {
  return Quaternionf(msg.w, msg.x, msg.y, msg.z);
}

template <typename T>
inline Transform3Base<T> FromROSPose(const geometry_msgs::Pose& msg) {
  return Transform3Base<T>(FromROS(msg.position), QuatToR(FromROS(msg.orientation)));
}

template <typename T>
inline Transform3Base<T> FromROSTransform(const geometry_msgs::Transform& msg) {
  return Transform3Base<T>(FromROS(msg.translation), QuatToR(FromROS(msg.rotation)));
}

template <typename T>
inline geometry_msgs::Point ToRosPoint(const Vector2Base<T>& v) {
  geometry_msgs::Point msg;
  msg.x = v(0);
  msg.y = v(1);
  msg.z = 0.0;

  return msg;
}

template <typename T>
inline geometry_msgs::Point ToRosPoint(const Vector3Base<T>& v) {
  geometry_msgs::Point msg;
  msg.x = v(0);
  msg.y = v(1);
  msg.z = v(2);

  return msg;
}

template <typename T>
inline geometry_msgs::Point32 ToRosPoint32(const Vector2Base<T>& v) {
  geometry_msgs::Point32 msg;
  msg.x = v(0);
  msg.y = v(1);
  msg.z = 0.0f;

  return msg;
}

template <typename T>
inline geometry_msgs::Point32 ToRosPoint32(const Vector3Base<T>& v) {
  geometry_msgs::Point32 msg;
  msg.x = v(0);
  msg.y = v(1);
  msg.z = v(2);

  return msg;
}

template <typename T>
inline geometry_msgs::Vector3 ToRosVec(const Vector2Base<T>& v) {
  geometry_msgs::Vector3 msg;
  msg.x = v(0);
  msg.y = v(1);
  msg.z = 0.0;

  return msg;
}

template <typename T>
inline geometry_msgs::Vector3 ToRosVec(const Vector3Base<T>& v) {
  geometry_msgs::Vector3 msg;
  msg.x = v(0);
  msg.y = v(1);
  msg.z = v(2);

  return msg;
}

template <typename T>
inline geometry_msgs::Quaternion ToRosQuat(const QuaternionBase<T>& quat) {
  geometry_msgs::Quaternion msg;
  msg.w = quat.W();
  msg.x = quat.X();
  msg.y = quat.Y();
  msg.z = quat.Z();

  return msg;
}

template <typename T>
inline geometry_msgs::Pose ToRosPose(const Transform2Base<T>& trans) {
  geometry_msgs::Pose msg;
  msg.position = ToRosPoint(trans.translation);
  msg.orientation = ToRosQuat(RToQuat(Rotation3Base<T>(trans.rotation)));

  return msg;
}

template <typename T>
inline geometry_msgs::Pose ToRosPose(const Transform3Base<T>& trans) {
  geometry_msgs::Pose msg;
  msg.position = ToRosPoint(trans.translation);
  msg.orientation = ToRosQuat(RToQuat(trans.rotation));

  return msg;
}

template <typename T>
inline geometry_msgs::Transform ToRosTransform(const Transform2Base<T>& trans) {
  geometry_msgs::Transform msg;
  msg.translation = ToRosVec(trans.translation);
  msg.rotation = ToRosQuat(RToQuat(Rotation3Base<T>(trans.rotation)));

  return msg;
}

template <typename T>
inline geometry_msgs::Transform ToRosTransform(const Transform3Base<T>& trans) {
  geometry_msgs::Transform msg;
  msg.translation = ToRosVec(trans.translation);
  msg.rotation = ToRosQuat(RToQuat(trans.rotation));

  return msg;
}

template <typename T>
inline geometry_msgs::Quaternion ToRosQuat(const Vector3Base<T>& angles) {
  return ToRosQuat(RToQuat(ZYXToR(angles)));
}

template <typename T>
inline Vector3Base<T> RosQuatToZYX(const geometry_msgs::Quaternion& msg) {
  return RToZYX(QuatToR(FromROS(msg)));
}

template <typename T>
inline double GetRoll(const geometry_msgs::Quaternion& q) {
  return Rotation3Base<T>(FromROS(q)).Roll();
}

template <typename T>
inline double GetPitch(const geometry_msgs::Quaternion& q) {
  return Rotation3Base<T>(FromROS(q)).Pitch();
}

template <typename T>
inline double GetYaw(const geometry_msgs::Quaternion& q) {
  return Rotation3Base<T>(FromROS(q)).Yaw();
}

}
}
#endif
