#pragma once

#include "IChain.h"
#include <vector>

namespace kinematics {

  class Chain : public IChain
  {
  public:

    bool addJoint(IJoint*) override;
    size_t size() const override;
    Point goForward() override;

  private:

    std::vector<IJoint*> chain_;
  };
}
