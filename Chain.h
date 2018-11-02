#pragma once

#include "IChain.h"
#include <cassert>
#include <vector>

namespace kinematics {

  class Chain : public IChain
  {
  public:

    bool addJoint(const JointPtr&) override;
    size_t size() const noexcept override;
    Point goForward() override;
    Point goForwardTo(size_t) override;

  private:

    std::vector<JointPtr> chain_;
  };
}
