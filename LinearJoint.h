#pragma once

#include "JointBase.h"

namespace kinematics
{
  class LinearJoint : public JointBase
  {
  public:

    LinearJoint(const DHConvention& convention);

    bool isRotary() override { return false; }

    std::unique_ptr<IJoint> deepCopy() const override;

  private:

    double& value() override;

  };
}