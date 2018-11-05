#ifndef SIMULATOR_H
#define SIMULATOR_H

#include <Eigen/Dense>
#include <vector>
#include "params.h"

Eigen::VectorXd measSimulator(const Eigen::VectorXd& state, const Params& params, const bool& noise = false);

#endif