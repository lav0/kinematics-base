#include "JointBase.h"

kinematics::JointBase::JointBase(const DHConvention & convention)
  : convention_(convention)
{
  jointTrs_ = convention_.matrix();
}

Matrix kinematics::JointBase::transformation() const
{
  return jointTrs_;
}

Point kinematics::JointBase::relativePosition() const
{
  return jointTrs_ * Point();
}

bool kinematics::JointBase::increase(double step)
{
  value() += step;

  bool ok = true;

  if (step > 0) {
    if (value() > max_)
    {
      value() = max_;
      ok = false;
    }
  }
  else
  {
    if (value() < min_)
    {
      value() = min_;
      ok = false;
    }
  }

  jointTrs_ = convention_.matrix();

  return ok;
}

void kinematics::JointBase::setMinMax(double min, double max)
{
  assert(min <= max);
  min_ = min;
  max_ = max;

  if (value() > max_)
    value() = max_;

  if (value() < min_)
    value() = min_;
}
