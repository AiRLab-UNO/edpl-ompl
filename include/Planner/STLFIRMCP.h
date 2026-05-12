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

#ifndef STLFIRMCP_PLANNER_H
#define STLFIRMCP_PLANNER_H

#include "Planner/FIRMCP.h"
#include "STLRom/stl_driver.h"
#include "STLRom/stl_monitor.h"
#include "STLRom/smooth_approx.h"

#include <vector>

/**
 * \brief STLFIRMCP – FIRMCP with an STL-robustness-based heuristic.
 *
 * Extends FIRMCP by using the STL robustness penalty as the only heuristic
 * signal for both neighbor selection and rollout backup.
 *
 * The cost for a neighbor action (from → to) becomes:
 *
 *   h = −ρ · scale
 *
 * where ρ is the robustness of the full rollout trajectory.
 *
 * The STL specification is a conjunction of:
 *   φ_safe  : G[0,1](safe > 0)                    – trajectory stays collision-free
 *   φ_firm  : G[0,1](firm_cost_max − firm_cost > 0) – FIRM cost stays below a threshold
 *   φ_info  : G[0,1](cov_max − tr_cov > 0)       – belief covariance stays below cov_max
 *   φ_reach : F[0,1](eps_goal − dist_goal > 0)    – robot reaches within eps_goal of goal
 *
 * Both signals are evaluated over the full rollout trajectory collected during
 * POMCP simulation, not just on the last edge endpoint pair.
 *
 * Parameters are loaded from the YAML setup file under the `stl_firmcp` key.
 */
class STLFIRMCP : public FIRMCP
{

public:

    STLFIRMCP(const firm::SpaceInformation::SpaceInformationPtr &si,
              bool debugMode = false);

    virtual ~STLFIRMCP(void);

    /** \brief Load planner parameters.  Reads both `firmcp` and `stl_firmcp`
     *         sections from the YAML file and builds the STL monitor. */
    virtual void loadParametersFromFile(const std::string &pathToFile) override;

    /** \\brief Reset the rollout trajectory before a new POMCP particle is simulated. */
    virtual void beginRolloutTrajectory() override;

    /** \\brief Record a visited vertex in the current rollout trajectory. */
    virtual void recordRolloutTrajectoryVertex(const Vertex vertex) override;

    /** \\brief Add the STL penalty for the full rollout trajectory to the returned total cost. */
    virtual double finalizeRolloutTrajectoryCost(const double totalCostToGo) override;


protected:

    /**
     * \brief Override FIRMCP's hook.
     *
        * Returns STL-only cost  h = −ρ·scale,
        * where ρ is the (smooth) robustness of phi along the current rollout trace.
     */
    virtual double computeCostToGoForNeighbor(const Vertex from,
                                              const Vertex to,
                                              double edgeCost) override;

private:

    // -----------------------------------------------------------------------
    // STL monitor
    // -----------------------------------------------------------------------
    STLRom::STLMonitor stlMonitor_;   ///< re-used each call (data cleared each time)

    /** Build the STLMonitor from current parameters.  Called at the end of
     *  loadParametersFromFile(). */
    void buildSTLMonitor();

    /**
     * \brief Evaluate smooth STL robustness on the collected rollout trajectory.
     * \return ρ  (positive = satisfied, negative = violated)
     */
    double evaluateSTLRobustness(const std::vector<Vertex> &trajectory);

    // -----------------------------------------------------------------------
    // Parameters (loaded from YAML stl_firmcp section)
    // -----------------------------------------------------------------------

    /** STL formula base string (signal/param names already pre-registered). */
    std::string stlFormulaStr_;

    /** Maximum acceptable trace(P) – belief quality threshold. */
    double covMax_;

    /** Goal-reaching distance threshold [same units as state space]. */
    double epsGoal_;

    /** Maximum acceptable heuristic FIRM cost. */
    double firmCostMax_;

    /** Compatibility parameter retained in YAML. */
    double lambdaStl_;

    /**
     * Scale factor to bring STL robustness into the same numerical range as
     * the FIRM cost-to-go (e.g., obstacle_cost_to_go ≈ 20 000).
     */
    double stlCostScale_;

    /** Temperature for smooth approximation (higher = closer to exact min/max). */
    double smoothTau_;

    /** Smooth approximation type: LSE (log-sum-exp), SOFTMAX, or EXACT. */
    STLRom::SmoothType smoothType_;

    /** Cached goal position extracted from goalM_[0] at first evaluation. */
    double goalX_, goalY_;
    bool goalCached_;

    /** Vertices visited by the current rollout trajectory. */
    std::vector<Vertex> rolloutTrajectory_;

    /** Cache for the STL cost computed the first time finalizeRolloutTrajectoryCost
     *  is called for the current particle.  Reset each call to beginRolloutTrajectory(). */
    double cachedRolloutCost_;
    bool   rolloutCostCached_;
};

#endif // STLFIRMCP_PLANNER_H
