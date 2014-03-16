#ifndef __VEC3_H
#define __VEC3_H

#include <math.h>

template <class T>
struct vec2 {
  T x, y;

  vec2() {}
  vec2(T x_, T y_): x(x_), y(y_) {}
};

template <class T>
static inline double operator*(const vec2<T> &a, const vec2<T> &b) {
  return a.x*b.x + a.y*b.y;
}

template<class T>
struct vec3 {
  T x, y, z;

  vec3() {}
  vec3(T x_, T y_, T z_): x(x_), y(y_), z(z_) {}

  vec2<T> xy() const { return vec2<T>(x, y); }
  vec2<T> xz() const { return vec2<T>(x, z); }
  vec2<T> yz() const { return vec2<T>(y, z); }

  vec3& operator+=(const vec3<T>& b) {
    x += b.x; y += b.y; z += b.z;
    return *this;
  }

  vec3& operator-=(const vec3<T>& b) {
    x -= b.x; y -= b.y; z -= b.z;
    return *this;
  }

  vec3& operator*=(const T s) {
    x *= s; y *= s; z *= s;
    return *this;
  }

  vec3& normalize() {
    T norm = static_cast<T>(1.0 / sqrt(static_cast<double> (x*x + y*y + z*z)));
    x *= norm; y *= norm; z *= norm;
    return *this;
  }
};

template <class T>
static inline vec3<T> operator+(const vec3<T>& a, const vec3<T>& b) {
  return vec3<T>(a.x+b.x, a.y+b.y, a.z+b.z);
}

template <class T>
static inline vec3<T> operator-(const vec3<T>& a, const vec3<T>& b) {
  return vec3<T>(a.x-b.x, a.y-b.y, a.z-b.z);
}

template <class T>
static inline vec3<T> operator-(const vec3<T>& a) {
  return vec3<T>(-a.x, -a.y, -a.z);
}

template <class T, class U>
static inline T operator*(const vec3<T>& a, const vec3<U>& b) {
  return a.x*b.x + a.y*b.y + a.z*b.z;
}

template <class T>
static inline vec3<T> operator*(T s, const vec3<T>& b) {
  return vec3<T>(s*b.x, s*b.y, s*b.z);
}

template <class T>
static inline vec3<T> operator*(const vec3<T>& b, T s) {
  return vec3<T>(s*b.x, s*b.y, s*b.z);
}

template <class T>
static inline vec3<T> cross(const vec3<T>& a, const vec3<T>& b) {
  return vec3<T>(
      a.y*b.z - a.z*b.y,
      a.z*b.x - a.x*b.z,
      a.x*b.y - a.y*b.x);
}

#endif  // __VEC3_H
