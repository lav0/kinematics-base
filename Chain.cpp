#include "Chain.h"

bool kinematics::Chain::addJoint(const JointPtr& joint)
{
  chain_.push_back(joint);
  return true;
}

size_t kinematics::Chain::size() const noexcept
{
  return chain_.size();
}

Point kinematics::Chain::goForward()
{
  return goForwardTo(size());
}

Point kinematics::Chain::goForwardTo(size_t joint_position)
{
  assert(joint_position >= 0);
  assert(joint_position <= size());

  if (joint_position <= 0)
  {
    return Point();
  }

  Matrix trs;
  for (size_t i = 0; i < joint_position - 1; i++)
  {
    trs = trs * chain_[i]->transformation();
  }

  return trs * chain_[joint_position-1]->relativePosition();
}
