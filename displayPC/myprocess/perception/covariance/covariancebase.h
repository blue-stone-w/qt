#ifndef COVARIANCEBASE_H
#define COVARIANCEBASE_H

#include "Covariance.h"

class CovarianceBase: public PerceptionInterface
{
 public:
  CovarianceBase();

  void getObjectCloud() override;

 private:
  // devide ground and nonground
  Covariance covariance;
};

#endif // COVARIANCEBASE_H
