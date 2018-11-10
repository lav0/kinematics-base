#pragma once

#include "IChain.h"
#include "../NelderMeadSolver/Solver.h"
#include "../NelderMeadSolver/SimplexFactory.h"
#include <cassert>
#include <vector>

namespace kinematics {

  class Chain : public IChain
  {
  public:

    ~Chain() override {}

    bool addJoint(const JointPtr&) override;
    size_t size() const noexcept override;
    Point goForward() override;
    Point goForwardTo(size_t) override;
    bool goInverse(const Point&) override;

  private:

    std::vector<JointPtr> chain_;
  };
}
