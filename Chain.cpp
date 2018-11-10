#include "Chain.h"
#include "ValueWrapperJoint.h"

using namespace kinematics;

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

bool kinematics::Chain::goInverse(const Point & point)
{
  using namespace nmsolver;

  VariableSetGeneric<ValueWrapperDouble> pattern_varset(chain_.size());

  std::vector<ValueWrapperJoint> wjoints;
  for (size_t i = 0; i < chain_.size(); ++i)
  {
    auto& joint = chain_[i];
    double min, max;
    joint->getMinMax(min, max);
    pattern_varset.set_limits(i, min, max);
    pattern_varset.set_var(i, joint->getJointValue());

    auto wjoint = ValueWrapperJoint(joint);
    wjoints.push_back(wjoint);
  }

  auto obj_fnc = [this, &wjoints, &point](const IVariableSet* vars)
  {
    using Type = const VariableSetGeneric<ValueWrapperDouble>*;
    auto values = dynamic_cast<Type>(vars);

    for (size_t i=0; i<values->size(); ++i)
      wjoints[i].set(values->get_var(i));

    auto now = goForward();
    auto x = (now[0] - point[0]);
    auto y = (now[1] - point[1]);
    auto z = (now[2] - point[2]);

    return x*x + y*y + z*z;
  };
    
  auto simplex = SimplexFactory::simplexGenericDouble(
    obj_fnc, 
    wjoints.size() + 1, 
    pattern_varset
  );

  Solver solver;
  solver.addSimplex(simplex.get());

  auto prec = 1e-5;
  auto solution = IVariableSetUPtr();
  auto value = 0.0;
  auto solved = solver.solve(false, prec, solution, value);
  
  for (auto& wj : wjoints) {
    if (wj.joint()->isRotary())
    {
      auto angle = wj.joint()->getJointValue();
      while (angle > pi)
        angle -= 2 * pi;

      while (angle < -pi)
        angle += 2 * pi;

      wj.set(angle);
    }
  }

  return solved;
}
