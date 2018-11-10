#pragma once
#include "IKinematicsJoint.h"

namespace kinematics {
  class IChain
  {
  public:

    virtual bool addJoint(const JointPtr&) = 0;
    virtual size_t size() const noexcept = 0;
    virtual Point goForward() = 0;
    virtual Point goForwardTo(size_t) = 0;
    virtual bool goInverse(const Point&) = 0;

    virtual ~IChain() {}
  };
}