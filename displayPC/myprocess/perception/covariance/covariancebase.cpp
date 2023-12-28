#include "covariancebase.h"

CovarianceBase::CovarianceBase()
{

}


void CovarianceBase::getObjectCloud()
{

  covariance.filteredWithNearestNeighborVariance(parseOriCloud());
  editObjectCloud() = covariance.cloud_out;

}
