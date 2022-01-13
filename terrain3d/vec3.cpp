#include "vec3.h"
#include <cmath>

vec3::vec3() {
  vector[0] = 0;
  vector[1] = 0;
  vector[2] = 0;
}

vec3::vec3(double x, double y, double z) {
  vector[0] = x;
  vector[1] = y;
  vector[2] = z;
}

vec3 vec3::create() {
  return vec3(0, 0, 0);
}

vec3 vec3::fromValues(double x, double y, double z) {
  return vec3(x, y, z);
}

vec3 vec3::cross(vec3 v1, vec3 v2) {
  return vec3(
    v1[1] * v2[2] - v1[2] * v2[1],
    v1[2] * v2[0] - v1[0] * v2[2],
    v1[0] * v2[1] - v1[1] * v2[0]
  );
}

vec3 vec3::normalize(vec3 v) {
  auto m = v[0] * v[0] + v[1] * v[1]+ v[2] * v[2];

  auto magnitude = std::sqrt(m);

  return vec3(
    v[0] / magnitude,
    v[1] / magnitude,
    v[2] / magnitude
  );
}

