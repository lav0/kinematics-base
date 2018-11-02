#pragma once

#include "..\VeryBasicMath\Matrix.h"

using Matrix = verybasicmath::Matrix;


namespace kinematics
{
  struct DHConvention
  {
    DHConvention(double a, double alfa, double d, double tetta)
      : link_length(a), link_twist(alfa), link_offset(d), joint_angle(tetta) {}

    double link_length;
    double get_a() const { return link_length; }

    double link_twist;
    double get_alfa() const { return link_twist; }

    double link_offset;
    double get_d() const { return link_offset; }

    double joint_angle;
    double get_tetta() const { return joint_angle; }

    Matrix matrix() const;
  };
}
