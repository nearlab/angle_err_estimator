#ifndef SIMULATOR_H
#define SIMULATOR_H

#include <Eigen/Dense>
#include <vector>
#include "params.h"
#include <random>

Eigen::VectorXd measSimulator(const Eigen::VectorXd& state, Params& params, const bool& noise = false);

#endif