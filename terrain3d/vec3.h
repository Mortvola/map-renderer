#pragma once

class vec3
{
public:
  vec3();

  vec3(double x, double y, double z);

  static vec3 create();

  static vec3 fromValues(double x, double y, double z);

  static vec3 cross(vec3 v1, vec3 v2);

  static vec3 normalize(vec3 v);

  double operator[](int index) const {
    return vector[index];
  }

private:
  double vector[3];
};

