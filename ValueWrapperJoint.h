#pragma once

#include "IKinematicsJoint.h"

namespace kinematics
{
  class ValueWrapperJoint
  {
  public:
    explicit ValueWrapperJoint(const JointPtr& a) : j_(a) {}
    explicit ValueWrapperJoint(int) : j_(nullptr) {}
    ValueWrapperJoint(const ValueWrapperJoint& other) : j_(other.j_) {}
    ValueWrapperJoint& operator=(const ValueWrapperJoint& other)
    {
      j_ = other.j_;
      return *this;
    }

    const double get() const 
    {
      return j_->getJointValue();
    }
    void set(const double& a)
    { 
      auto old = get();
      j_->increase(a - old);
    }

    JointPtr& joint() { return j_; }

  private:

    JointPtr j_ = nullptr;
  };

}