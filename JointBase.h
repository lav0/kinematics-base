#pragma once
#include <cassert>
#include <cfloat>

#include "IKinematicsJoint.h"

namespace kinematics
{
  using DHConvention = verybasicmath::DHConvention;

  class JointBase : public IJoint
  {
  public:

    JointBase(const DHConvention& convention);
    Matrix transformation() const override;
    Point relativePosition() const override;

    bool increase(double step) override;
    void setMinMax(double min, double max) override;

  protected:

    virtual double& value() = 0;

    DHConvention convention_;
    Matrix jointTrs_;

    double min_ = -DBL_MAX;
    double max_ = DBL_MAX;
  };
}
