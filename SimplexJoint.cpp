#include "../../NelderMeadSolver/NelderMeadSolver/NelderMeadSolver/SimplexGeneric.h"
#include "ValueWrapperJoint.h"

using namespace kinematics;
using namespace nmsolver;

using SimplexGenericJoint = SimplexGeneric<ValueWrapperJoint>;
using VariableSetJointPtr = std::unique_ptr<
  nmsolver::VariableSetGeneric<
    ValueWrapperJoint
  >
>;

SimplexGenericJoint::SimplexGeneric(
  std::function<double(const IVariableSet*)> obj_function,
  size_t simplex_size
)
  : //std::function<double(const IVariableSetUPtr&)> obj_function, 
  expected_size_(simplex_size)
{
  variables_.reserve(simplex_size);
}

double SimplexGenericJoint::get_deviation() const
{
  return 0;
}

IVariableSetUPtr SimplexGenericJoint::reflection()
{
  return VariableSetJointPtr();
}

IVariableSetUPtr SimplexGenericJoint::expansion()
{
  return VariableSetJointPtr();
}

IVariableSetUPtr SimplexGenericJoint::contraction()
{
  return VariableSetJointPtr();
}

void SimplexGenericJoint::shrink()
{
}

double SimplexGenericJoint::value_in_point(const IVariableSetUPtr & a) const
{
  return 0.0;
}

const double SimplexGenericJoint::best_value() const
{
  return 0.0;
}

const double SimplexGenericJoint::second_worst_value() const
{
  return 0.0;
}

const double SimplexGenericJoint::worst_value() const
{
  return 0.0;
}

void SimplexGenericJoint::replace_maximum(const IVariableSetUPtr & a_new_point)
{
}

template<>
IVariableSetUPtr SimplexGenericJoint::get_gravity_centre() const
{
  using VariableSetJoint = VariableSetGeneric<T>;
  using Wrapper = VariableSetJoint::_type;

  auto size = variable_size();
  auto res = std::make_unique<VariableSetJoint>(size);

  auto base_varset = dynamic_cast<VariableSetJoint*>(variables_[0].get());

  for (size_t i = 0; i < size; ++i)
  {
    std::unique_ptr<IJoint> fd = base_varset->get_gut(i).joint()->deepCopy();
    auto sum = Wrapper(std::move(fd));
    sum.set(0);

    for (auto& varset : variables_)
    {
      auto a = sum.get();
      auto b = varset->get_var(i);
      auto d = (a + b);
      sum.set(d);
    }
    sum.set(sum.get() / simplex_size());

    res->set_gut(i, sum);
    //res->set_var(i, sum.get());
  }

  return res;
}
template<>
IVariableSetUPtr SimplexGenericJoint::get_centroid()
{
  return IVariableSetUPtr();
}

bool SimplexGenericJoint::addVariableSet(IVariableSetSPtr& varset)
{
  if (variables_.size() < expected_size_)
  {
    variables_.push_back(varset);
    return true;
  }

  return false;
}

template<>
size_t SimplexGenericJoint::simplex_size() const
{
  return variables_.size();
}

template<>
size_t SimplexGenericJoint::variable_size() const
{
  if (!variables_.empty())
    return variables_.front()->size();

  return size_t(0);
}

IVariableSetSPtr SimplexGenericJoint::get_variable(size_t index) const
{
  return variables_[index];
}
