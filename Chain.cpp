#include "Chain.h"

bool kinematics::Chain::addJoint(IJoint * joint)
{
  chain_.push_back(joint);
  return true;
}

size_t kinematics::Chain::size() const
{
  return chain_.size();
}

Point kinematics::Chain::goForward()
{
  auto j1 = chain_[0];
  auto j2 = chain_[1];

  auto trs = j1->transformation() * j2->relativePosition();

  return trs;
}
