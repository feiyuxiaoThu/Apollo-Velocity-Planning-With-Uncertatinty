/*
 * @Author: fujiawei0724
 * @Date: 2022-09-24 09:03:57
 * @LastEditors: fujiawei0724
 * @LastEditTime: 2022-09-24 10:14:57
 * @Description: interface for a velocity planning method in Baidu Apollo
 */
#pragma once
#include <iostream>
#include <string.h>
#include <vector>
#include <array>
#include "StGraph.hpp"
#include "modules/planning/math/piecewise_jerk/piecewise_jerk_speed_problem.h"

namespace Apollo {

class PjsoInterface {
 public: 
 
    static bool runOnce(const std::array<double, 3>& start_state, const double& s_max, const double& max_speed, const double& min_speed, const double& max_acc, const double& min_acc, const double& max_jerk, const double& min_jerk, const double& cruise_speed, const std::vector<VelocityPlanning::Cube2D<double>>& cubes_path, const double& expected_acc, std::vector<double>* s, std::vector<double>* ds, std::vector<double>* dds, std::vector<double>* t);

    static std::vector<VelocityPlanning::Cube2D<double>> selectCubesPath(const std::vector<std::vector<VelocityPlanning::Cube2D<double>>>& cubes_paths);

};

} // End of namespace Apollo