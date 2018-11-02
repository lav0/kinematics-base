#pragma once

#include <memory>
#include "..\VeryBasicMath\Matrix.h"

using Matrix = verybasicmath::Matrix;
using Point = verybasicmath::Point;

class IJoint
{
public:

  virtual bool isRotary() = 0;

  virtual Matrix transformation() const = 0;
  virtual Point relativePosition() const = 0;

  virtual bool increase(double step) = 0;
  virtual void setMinMax(double min, double max) = 0;

  virtual ~IJoint() {}
};

using JointPtr = std::shared_ptr<IJoint>;

