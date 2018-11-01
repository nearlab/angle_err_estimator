#ifndef SIMULATOR_H
#define SIMULATOR_H

#include <Eigen/Dense>
#include "quatMath.h"
#include <vector>
#include "params.h"

Eigen::VectorXd measSimulator(const Eigen::VectorXd& state, const Params& params);

#endif