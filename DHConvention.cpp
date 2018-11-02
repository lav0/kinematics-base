#include "DHConvention.h"

Matrix kinematics::DHConvention::matrix() const
{
  double mMat[4][4];

  for (unsigned i = 0; i<4; ++i)
  {
    for (unsigned j = 0; j<4; ++j)
    {
      if (i == j)
        mMat[i][j] = 1;
      else
        mMat[i][j] = 0;
    }
  }

  auto ct = std::cos(get_tetta());
  auto st = std::sin(get_tetta());
  auto ca = std::cos(get_alfa());
  auto sa = std::sin(get_alfa());

  mMat[0][0] = ct;  mMat[0][1] = -st * ca; mMat[0][2] = st * sa;
  mMat[1][0] = st;  mMat[1][1] = ct * ca; mMat[1][2] = -ct * sa;
  mMat[2][1] = sa;       mMat[2][2] = ca;

  mMat[0][3] = ct * get_a();
  mMat[1][3] = st * get_a();
  mMat[2][3] = get_d();
  
  return Matrix(mMat);
}
