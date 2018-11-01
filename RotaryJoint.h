#pragma once

#include "JointBase.h"

namespace kinematics
{
  class RotaryJoint : public JointBase
  {
  public:

    RotaryJoint(const DHConvention& convention);

    bool isRotary() override { return true; }

  private:

    double& value() override;

  };
}