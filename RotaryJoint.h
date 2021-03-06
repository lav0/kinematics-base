#pragma once

#include "JointBase.h"

namespace kinematics
{
  class RotaryJoint : public JointBase
  {
  public:

    RotaryJoint(const DHConvention& convention);

    bool isRotary() override { return true; }

    std::unique_ptr<IJoint> deepCopy() const override;

  private:

    double& value() override;

  };
}