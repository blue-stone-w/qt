#ifndef TRUCKPARAM_H
#define TRUCKPARAM_H

#include "string"
#include "vector"

struct TruckParam
{
  std::string name;
  std::vector<int> ip;
  std::vector<double> lf,lb;
};

#endif // TRUCKPARAM_H
