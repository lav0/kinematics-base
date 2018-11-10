#pragma once
#include <cassert>
#include <cfloat>

#include "IKinematicsJoint.h"
#include "DHConvention.h"

namespace kinematics
{
  class JointBase : public IJoint
  {
  public:

    JointBase(const DHConvention& convention);
    ~JointBase() override {}

    Matrix transformation() const override;
    Point relativePosition() const override;

    bool increase(double step) override;
    void setMinMax(double min, double max) override;
    void getMinMax(double& min, double& max) override;

    double getJointValue() override;

  protected:

    virtual double& value() = 0;

    DHConvention convention_;
    Matrix jointTrs_;

    double min_ = -DBL_MAX;
    double max_ = DBL_MAX;
  };
}
