/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, Texas A&M University
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Texas A&M University nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Authors: Redwan Newaz et al. */

#include "Planner/STLFIRMCP.h"
#include <yaml-cpp/yaml.h>
#include <cmath>
#include <sstream>
#include <stdexcept>


STLFIRMCP::STLFIRMCP(const firm::SpaceInformation::SpaceInformationPtr &si, bool debugMode)
    : FIRMCP(si, debugMode)
    , covMax_(1.0)
    , epsGoal_(1.0)
    , firmCostMax_(1000.0)
    , lambdaStl_(0.3)
    , stlCostScale_(1000.0)
    , smoothTau_(1.0)
    , smoothType_(STLRom::SmoothType::SOFTMAX)
    , goalX_(0.0), goalY_(0.0)
    , goalCached_(false)
    , stlFormulaStr_("phi_safe := alw_[0,1] (safe[t] > 0)\nphi_firm := alw_[0,1] (firm_cost_max - firm_cost[t] > 0)\nphi_info := alw_[0,1] (cov_max - tr_cov[t] > 0)\nphi_reach := ev_[0,1] (eps_goal - dist_goal[t] > 0)\nphi := phi_safe and phi_firm and phi_info and phi_reach")
    , cachedRolloutCost_(0.0)
    , rolloutCostCached_(false)
{
}

STLFIRMCP::~STLFIRMCP(void)
{
}

// ---------------------------------------------------------------------------
// Parameter loading
// ---------------------------------------------------------------------------

void STLFIRMCP::loadParametersFromFile(const std::string &pathToFile)
{
    // First load all base FIRMCP parameters
    FIRMCP::loadParametersFromFile(pathToFile);

    YAML::Node config = YAML::LoadFile(pathToFile);
    const auto& sc = config["stl_firmcp"];

    if (sc["stl_formula"])
        stlFormulaStr_ = sc["stl_formula"].as<std::string>();

    if (sc["cov_max"])
        covMax_ = sc["cov_max"].as<double>();

    if (sc["eps_goal"])
        epsGoal_ = sc["eps_goal"].as<double>();

    if (sc["firm_cost_max"])
        firmCostMax_ = sc["firm_cost_max"].as<double>();

    if (sc["lambda_stl"])
        lambdaStl_ = sc["lambda_stl"].as<double>();

    if (sc["stl_cost_scale"])
        stlCostScale_ = sc["stl_cost_scale"].as<double>();

    if (sc["smooth_tau"])
        smoothTau_ = sc["smooth_tau"].as<double>();

    if (sc["smooth_type"])
    {
        std::string st = sc["smooth_type"].as<std::string>();
        if (st == "SOFTMAX")
            smoothType_ = STLRom::SmoothType::SOFTMAX;
        else if (st == "EXACT")
            smoothType_ = STLRom::SmoothType::EXACT;
        else
            smoothType_ = STLRom::SmoothType::LSE;
    }

    OMPL_INFORM("STLFIRMCP: lambda_stl=%.2f  stl_cost_scale=%.1f  cov_max=%.3f  eps_goal=%.3f",
                lambdaStl_, stlCostScale_, covMax_, epsGoal_);

    buildSTLMonitor();
}

void STLFIRMCP::beginRolloutTrajectory()
{
    rolloutTrajectory_.clear();
    rolloutCostCached_ = false;
    cachedRolloutCost_ = 0.0;
}

void STLFIRMCP::recordRolloutTrajectoryVertex(const Vertex vertex)
{
    if (rolloutTrajectory_.empty() || rolloutTrajectory_.back() != vertex)
        rolloutTrajectory_.push_back(vertex);
}

double STLFIRMCP::finalizeRolloutTrajectoryCost(const double totalCostToGo)
{
    // FIRMCP calls this at every return point in the recursive pomcpSimulate /
    // pomcpRollout chain.  We must evaluate STL exactly once per particle and
    // return the same value for all subsequent recursive calls so that the costs
    // do not compound across recursion levels.
    if (rolloutCostCached_)
        return cachedRolloutCost_;

    // Not enough trajectory data yet – fall back to the FIRM cost.
    if (rolloutTrajectory_.size() < 2)
        return totalCostToGo;

    const double rho = evaluateSTLRobustness(rolloutTrajectory_);
    cachedRolloutCost_ = -rho * stlCostScale_;
    rolloutCostCached_ = true;

    OMPL_INFORM("STLFIRMCP: rho=%.4f  stl_cost=%.1f  traj_len=%zu",
                rho, cachedRolloutCost_, rolloutTrajectory_.size());

    return cachedRolloutCost_;
}

// ---------------------------------------------------------------------------
// STL monitor construction
// ---------------------------------------------------------------------------

void STLFIRMCP::buildSTLMonitor()
{
    STLRom::STLDriver driver;

    /*
     * Build the complete STL spec string:
     *  1. Signal declarations (col 0 = time, col 1 = tr_cov, col 2 = dist_goal)
     *  2. Parameter declarations with current values
     *  3. The formula lines loaded from YAML (stlFormulaStr_)
     *
     * The formula lines must use STLRom syntax:
     *   - alw_[a,b] / ev_[a,b]  (not G / F)
     *   - signal[t]              (e.g. tr_cov[t])
     *   - < / >                  (not >= / <=)
     *   - and / or               (not & / |)
     */
    std::ostringstream spec;
        spec << "signal tr_cov, dist_goal, safe, firm_cost\n";
        spec << "param cov_max = " << covMax_
            << ", eps_goal = "  << epsGoal_
            << ", firm_cost_max = " << firmCostMax_ << "\n";
    spec << stlFormulaStr_ << "\n";

    const std::string fullSpec = spec.str();

    if (!driver.parse_string(fullSpec))
    {
        OMPL_ERROR("STLFIRMCP: Failed to parse STL spec:\n%s", fullSpec.c_str());
        throw std::runtime_error("STLFIRMCP: STL formula parse error. Check stl_formula in YAML.");
    }

    stlMonitor_ = driver.get_monitor("phi");

    if (!stlMonitor_.formula)
    {
        OMPL_ERROR("STLFIRMCP: STL monitor formula is null.  "
                   "Ensure formula is assigned to 'phi' (e.g. 'phi := ...').");
        throw std::runtime_error("STLFIRMCP: STL monitor has no formula.");
    }

    // Propagate current param values into the monitor
    // (the monitor keeps its own map after parsing)
    stlMonitor_.param_map["cov_max"]  = covMax_;
    stlMonitor_.param_map["eps_goal"] = epsGoal_;
    stlMonitor_.param_map["firm_cost_max"] = firmCostMax_;

    OMPL_INFORM("STLFIRMCP: STL monitor built.  Formula: %s", stlFormulaStr_.c_str());
}

// ---------------------------------------------------------------------------
// STL robustness evaluation on the full rollout trajectory
// ---------------------------------------------------------------------------

double STLFIRMCP::evaluateSTLRobustness(const std::vector<Vertex> &trajectory)
{
    // Need at least two vertices so the STL monitor has a valid time interval.
    if (trajectory.size() < 2)
        return 0.0;

    // -----------------------------------------------------------------------
    // Lazily cache goal position (available after the graph is solved)
    // -----------------------------------------------------------------------
    if (!goalCached_ && !goalM_.empty())
    {
        ompl::base::State* gState = stateProperty_[goalM_[0]];
        goalX_ = gState->as<FIRM::StateType>()->getX();
        goalY_ = gState->as<FIRM::StateType>()->getY();
        goalCached_ = true;
    }

    // -----------------------------------------------------------------------
    // Build full trace and evaluate
    // -----------------------------------------------------------------------
    stlMonitor_.reset_signal_data();

    stlMonitor_.set_eval_time(0.0, 1.0);

    for (size_t i = 0; i < trajectory.size(); ++i)
    {
        ompl::base::State *state = stateProperty_[trajectory[i]];

        const double trCov = state->as<FIRM::StateType>()->getTraceCovariance();
        const double x = state->as<FIRM::StateType>()->getX();
        const double y = state->as<FIRM::StateType>()->getY();
        const double distGoal = std::sqrt((x - goalX_) * (x - goalX_) +
                                          (y - goalY_) * (y - goalY_));
        const double safe = si_->isValid(state) ? 1.0 : 0.0;
        const double firmCost = getCostToGoWithApproxStabCost(trajectory[i]);

        // Sample format: {time, tr_cov, dist_goal, safe, firm_cost}
        const double t = (trajectory.size() == 1) ? 0.0
                         : static_cast<double>(i) / static_cast<double>(trajectory.size() - 1);
        stlMonitor_.add_sample({t, trCov, distGoal, safe, firmCost});
    }

    // Update parameter values (may have been tuned; param_map is used at eval)
    stlMonitor_.param_map["cov_max"]  = covMax_;
    stlMonitor_.param_map["eps_goal"] = epsGoal_;
    stlMonitor_.param_map["firm_cost_max"] = firmCostMax_;

    double rho = 0.0;
    if (smoothType_ == STLRom::SmoothType::EXACT)
        rho = stlMonitor_.eval_rob(0.0, 1.0);
    else
        rho = stlMonitor_.eval_smooth_rob(0.0, 1.0, smoothTau_, smoothType_);

    return rho;   // positive = satisfied, negative = violated
}

// ---------------------------------------------------------------------------
// Virtual hook override: blend FIRM heuristic with STL cost
// ---------------------------------------------------------------------------

double STLFIRMCP::computeCostToGoForNeighbor(const Vertex from, const Vertex to, double edgeCost)
{
    // Extend the current rollout trajectory with 'to' so that each neighbor
    // gets an individual STL estimate that reflects its own state.  Without
    // this, every neighbor shares the same trajectory robustness and the
    // importance sampling in pomcpRollout is effectively uniform.
    std::vector<Vertex> extTraj = rolloutTrajectory_;
    if (extTraj.empty() || extTraj.back() != to)
        extTraj.push_back(to);

    // Fall back to the FIRM heuristic when the extended trajectory is still
    // too short for meaningful STL evaluation (e.g. very first expansion).
    if (extTraj.size() < 2)
        return edgeCost + getCostToGoWithApproxStabCost(to);

    const double rho = evaluateSTLRobustness(extTraj);
    return -rho * stlCostScale_;
}
