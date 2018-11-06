/*
  geometry_utils: Utility library to provide common geometry types and transformations
  Copyright (C) 2014  Nathan Michael
                2016  Erik Nelson

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

#ifndef GEOMETRY_UTILS_H
#define GEOMETRY_UTILS_H

#include "Vector2.h"
#include "Vector3.h"
#include "Vector4.h"
#include "Quaternion.h"
#include "Matrix2x2.h"
#include "Matrix3x3.h"
#include "Matrix4x4.h"
#include "Rotation2.h"
#include "Rotation3.h"
#include "Transform2.h"
#include "Transform3.h"

namespace geometry_utils {

inline double Unroll(double x) {
  x = fmod(x, 2.0 * M_PI);
  if (x < 0)
    x += 2.0 * M_PI;
  return x;
}

inline double Normalize(double x) {
  x = fmod(x + M_PI, 2.0 * M_PI);
  if (x < 0)
    x += 2.0 * M_PI;
  return x - M_PI;
}

inline double S1Distance(double from, double to) {
  double result = Unroll(Unroll(to) - Unroll(from));
  if (result > M_PI)
    result = -(2.0 * M_PI - result);
  return Normalize(result);
}

inline double Rad2Deg(double angle) {
  return angle * 180.0 * M_1_PI;
}

inline float Rad2Deg(float angle) {
  return angle * 180.0 * M_1_PI;
}

inline double Deg2Rad(double angle) {
  return angle * M_PI / 180.0;
}

inline float Deg2Rad(float angle) {
  return angle * M_PI / 180.0;
}

template <typename T>
inline Vector3Base<T> Rad2Deg(const Vector3Base<T>& angles) {
  return Vector3Base<T>(Rad2Deg(angles(0)), Rad2Deg(angles(1)), Rad2Deg(angles(2)));
}

template <typename T>
inline Vector3Base<T> Deg2Rad(const Vector3Base<T>& angles) {
  return Vector3Base<T>(Deg2Rad(angles(0)), Deg2Rad(angles(1)), Deg2Rad(angles(2)));
}

template <typename T>
inline Vector3Base<T> RToZYX(const Rotation3Base<T>& rot) {
  return rot.GetEulerZYX();
}

template <typename T>
inline Rotation3Base<T> ZYXToR(const Vector3Base<T>& angles) {
  return Rotation3Base<T>(angles);
}

inline Rot3d ZYXToR(const Vec3d& angles) {
  return Rot3d(angles);
}

inline Rot3f ZYXToR(const Vec3f& angles) {
  return Rot3f(angles);
}

template <typename T>
inline Rotation3Base<T> QuatToR(const QuaternionBase<T>& quat) {
  return Rotation3Base<T>(quat);
}

template <typename T>
inline QuaternionBase<T> RToQuat(const Rotation3Base<T>& rot) {
  return QuaternionBase<T>(Eigen::Quaternion<T>(rot.Eigen()));
}

template <typename T>
inline double GetRoll(const Rotation3Base<T>& r) {
  return r.Roll();
}

template <typename T>
inline double GetRoll(const QuaternionBase<T>& q) {
  return Rotation3Base<T>(q).Roll();
}

template <typename T>
inline double GetPitch(const Rotation3Base<T>& r) {
  return r.Pitch();
}

template <typename T>
inline double GetPitch(const QuaternionBase<T>& q) {
  return Rotation3Base<T>(q).Pitch();
}

template <typename T>
inline double GetYaw(const Rotation3Base<T>& r) {
  return r.Yaw();
}

template <typename T>
inline double GetYaw(const QuaternionBase<T>& q) {
  return Rotation3Base<T>(q).Yaw();
}

template <typename T>
inline double SO3Error(const QuaternionBase<T>& q1, const QuaternionBase<T>& q2) {
  return Rotation3Base<T>(q1).Error(Rotation3Base<T>(q2));
}

template <typename T>
inline double SO3Error(const Rotation3Base<T>& r1, const Rotation3Base<T>& r2) {
  return r1.Error(r2);
}

template <typename T>
inline Vector3Base<T> CartesianToSpherical(const Vector3Base<T>& v) {
  double rho = v.Norm();
  return Vector3Base<T>(rho, acos(v.Z() / rho), atan2(v.Y(), v.X()));
}

template <typename T>
inline Vector3Base<T> SphericalToCartesian(const Vector3Base<T>& v) {
  return Vector3Base<T>(v(0) * sin(v(1)) * cos(v(2)), v(0) * sin(v(1)) * sin(v(2)),
              v(0) * cos(v(1)));
}

template <typename T>
inline Vector3Base<T> NEDCartesian(const Vector3Base<T>& v) {
  return Vector3Base<T>(v(0), -v(1), -v(2));
}

}

#endif
