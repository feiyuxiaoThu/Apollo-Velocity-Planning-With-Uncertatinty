/*
 * @Author: fujiawei0724
 * @Date: 2022-09-24 09:05:13
 * @LastEditors: fujiawei0724
 * @LastEditTime: 2022-09-24 11:13:02
 * @Description: 
 */

#include "Common.hpp"

namespace Apollo {

bool PjsoInterface::runOnce(const std::array<double, 3>& start_state, const double& s_max, const double& max_speed, const double& min_speed, const double& max_acc, const double& min_acc, const double& max_jerk, const double& min_jerk, const double& cruise_speed, const std::vector<VelocityPlanning::Cube2D<double>>& cubes_path, const double& expected_acc, std::vector<double>* s, std::vector<double>* ds, std::vector<double>* dds, std::vector<double>* t) {
    // Set constant
    const double delta_t = 0.1;
    double total_length = s_max;
    const double total_time = 5.0;
    int num_of_knots = static_cast<int>(total_time / delta_t) + 1;

    // Define problem
    apollo::planning::PiecewiseJerkSpeedProblem piecewise_jerk_problem = apollo::planning::PiecewiseJerkSpeedProblem(num_of_knots, delta_t, start_state);

    // Set weights for acceleration and jerk
    piecewise_jerk_problem.set_weight_ddx(1.0);
    piecewise_jerk_problem.set_weight_dddx(1.0);

    // Set s bounds
    piecewise_jerk_problem.set_x_bounds(0.0, total_length);

    // Set v bounds
    piecewise_jerk_problem.set_dx_bounds(min_speed, max_speed);

    // Set acceleration bounds
    piecewise_jerk_problem.set_ddx_bounds(min_acc, max_acc);

    // Set jerk bounds
    piecewise_jerk_problem.set_dddx_bound(min_jerk, max_jerk);

    // Set reference velocity and its weight
    double cruise_v_weight = 1.0;
    piecewise_jerk_problem.set_dx_ref(cruise_v_weight, cruise_speed);

    // Set s bounds from cubes
    std::vector<std::pair<double, double>> s_bounds;
    // for (int i = 0; i < num_of_knots; i++) {
    //     double cur_lower = 0.0;
    //     double cur_upper = 0.0;
    //     if (i % 5 == 0 && i != 0 && i != num_of_knots - 1) {
    //         int next_cube_index = i / 5;
    //         int prev_cube_index = next_cube_index - 1;
    //         cur_lower = std::max(cubes_path[next_cube_index].s_start_, cubes_path[prev_cube_index].s_start_);
    //         cur_upper = std::min(cubes_path[next_cube_index].s_end_, cubes_path[prev_cube_index].s_end_);
            
    //     } else if (i == num_of_knots - 1) {
    //         int cube_index = i / 5 - 1;
    //         cur_lower = cubes_path[cube_index].s_start_;
    //         cur_upper = cubes_path[cube_index].s_end_;
    //     } else {
    //         int cube_index = i / 5;
    //         cur_lower = cubes_path[cube_index].s_start_;
    //         cur_upper = cubes_path[cube_index].s_end_;
    //     }
    //     s_bounds.emplace_back(std::make_pair(cur_lower, cur_upper));

    //     // std::cout << "Index: " << i << ", upper: " << cur_upper << ", lower: " << cur_lower << std::endl;

    // }

    for (int i = 0; i < num_of_knots; i++) {
        double cur_lower = 0.0;
        double cur_upper = 0.0;
        if (i == num_of_knots - 1) {
            int cube_index = i / 5 - 1;
            cur_lower = std::max(cubes_path[cube_index].s_end_ - 5.0, 0.0);
            cur_upper = cubes_path[cube_index].s_end_;
        } else {
            cur_lower = 0.0;
            cur_upper = total_length;
        }
        s_bounds.emplace_back(std::make_pair(cur_lower, cur_upper));

        // std::cout << "Index: " << i << ", upper: " << cur_upper << ", lower: " << cur_lower << std::endl;

    }
    piecewise_jerk_problem.set_x_bounds(std::move(s_bounds));

    // Set reference speed (calculate from expected speed)
    std::vector<double> v_ref;
    std::vector<double> penalty_dx;
    std::vector<std::pair<double, double>> v_bounds;
    for (int i = 0; i < num_of_knots; i++) {
        double corre_time = i * delta_t;
        double cur_velocity = start_state[1] + corre_time * expected_acc;
        v_ref.emplace_back(cur_velocity);
        penalty_dx.emplace_back(0.0);
        v_bounds.emplace_back(0.0, max_speed);
    }
    double ref_v_weight = 1.0;
    piecewise_jerk_problem.set_x_ref(ref_v_weight, std::move(v_ref));
    piecewise_jerk_problem.set_penalty_dx(penalty_dx);
    piecewise_jerk_problem.set_dx_bounds(std::move(v_bounds));

    bool res = piecewise_jerk_problem.Optimize();
    if (!res) {
        return false;
    }

    *s = piecewise_jerk_problem.opt_x();
    *ds = piecewise_jerk_problem.opt_dx();
    *dds = piecewise_jerk_problem.opt_ddx();
    for (int i = 0; i < num_of_knots; i++) {
        (*t).emplace_back(i * delta_t);
    }

    return true;

}

std::vector<VelocityPlanning::Cube2D<double>> PjsoInterface::selectCubesPath(const std::vector<std::vector<VelocityPlanning::Cube2D<double>>>& cubes_paths) {
    double max_s = INT_MIN;
    double target_index = -1;
    for (int i = 0; i < cubes_paths.size(); i++) {
        double cur_s = cubes_paths[i].back().s_end_;
        if (cur_s > max_s) {
            max_s = cur_s;
            target_index = i;
        }
    }
    assert(target_index != -1);
    return cubes_paths[target_index];
}

} // End of namespace Apollo

