#pragma once
#include "IKinematicsJoint.h"

class IChain
{
public:

  virtual bool addJoint(IJoint*) = 0;
  virtual size_t size() const = 0;
  virtual Point goForward() = 0;

  virtual ~IChain() {}
};