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

/* Authors: Sung Kyun Kim et al. */

#include "Planner/STLFIRMCP.h"
#include <yaml-cpp/yaml.h>
#include <cmath>
#include <sstream>
#include <stdexcept>


STLFIRMCP::STLFIRMCP(const firm::SpaceInformation::SpaceInformationPtr &si, bool debugMode)
    : FIRMCP(si, debugMode)
    , covMax_(1.0)
    , epsGoal_(1.0)
    , lambdaStl_(0.3)
    , stlCostScale_(1000.0)
    , smoothTau_(1.0)
    , smoothType_(STLRom::SmoothType::LSE)
    , goalX_(0.0), goalY_(0.0)
    , goalCached_(false)
    // Default STL specification (overridden from YAML):
    //   phi_info  : covariance stays below cov_max (belief quality)
    //   phi_reach : distance to goal drops below eps_goal (reachability)
    , stlFormulaStr_("phi := G[0,1](cov_max - tr_cov >= 0) & F[0,1](eps_goal - dist_goal >= 0)")
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
    spec << "signal tr_cov, dist_goal\n";
    spec << "param cov_max = " << covMax_
         << ", eps_goal = "  << epsGoal_ << "\n";
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

    OMPL_INFORM("STLFIRMCP: STL monitor built.  Formula: %s", stlFormulaStr_.c_str());
}

// ---------------------------------------------------------------------------
// STL robustness evaluation on a 2-point trace (from → to)
// ---------------------------------------------------------------------------

double STLFIRMCP::evaluateSTLRobustness(const Vertex from, const Vertex to)
{
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
    // Extract signals at both endpoints
    // -----------------------------------------------------------------------
    ompl::base::State* stateFrom = stateProperty_[from];
    ompl::base::State* stateTo   = stateProperty_[to];

    double trCovFrom    = stateFrom->as<FIRM::StateType>()->getTraceCovariance();
    double trCovTo      = stateTo->as<FIRM::StateType>()->getTraceCovariance();

    double xFrom = stateFrom->as<FIRM::StateType>()->getX();
    double yFrom = stateFrom->as<FIRM::StateType>()->getY();
    double xTo   = stateTo->as<FIRM::StateType>()->getX();
    double yTo   = stateTo->as<FIRM::StateType>()->getY();

    double distGoalFrom = std::sqrt((xFrom - goalX_) * (xFrom - goalX_) +
                                    (yFrom - goalY_) * (yFrom - goalY_));
    double distGoalTo   = std::sqrt((xTo   - goalX_) * (xTo   - goalX_) +
                                    (yTo   - goalY_) * (yTo   - goalY_));

    // -----------------------------------------------------------------------
    // Build 2-point trace and evaluate
    // -----------------------------------------------------------------------
    stlMonitor_.reset_signal_data();
    stlMonitor_.set_eval_time(0.0, 1.0);

    // Sample format: {time, tr_cov (col 1), dist_goal (col 2)}
    stlMonitor_.add_sample({0.0, trCovFrom, distGoalFrom});
    stlMonitor_.add_sample({1.0, trCovTo,   distGoalTo});

    // Update parameter values (may have been tuned; param_map is used at eval)
    stlMonitor_.param_map["cov_max"]  = covMax_;
    stlMonitor_.param_map["eps_goal"] = epsGoal_;

    double rho = stlMonitor_.eval_smooth_rob(smoothTau_, smoothType_);

    return rho;   // positive = satisfied, negative = violated
}

// ---------------------------------------------------------------------------
// Virtual hook override: blend FIRM heuristic with STL cost
// ---------------------------------------------------------------------------

double STLFIRMCP::computeCostToGoForNeighbor(const Vertex from, const Vertex to, double edgeCost)
{
    // FIRM global heuristic (equation [29]-[30] in the BVL paper)
    double firmCost = edgeCost + getCostToGoWithApproxStabCost(to);

    // Guard: if the goal position is not yet cached (graph still being built),
    // fall back to the pure FIRM heuristic to avoid garbage dist_goal values.
    if (!goalCached_ && goalM_.empty())
        return firmCost;

    // STL smooth robustness: ρ > 0 when spec satisfied
    //   → cost contribution = −ρ * scale  (lower is better when ρ is larger)
    double rho     = evaluateSTLRobustness(from, to);
    double stlCost = -rho * stlCostScale_;

    // Blended heuristic
    double blended = (1.0 - lambdaStl_) * firmCost + lambdaStl_ * stlCost;

    return blended;
}
