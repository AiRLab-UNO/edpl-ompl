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

/**
 * \brief STLFIRMCP – FIRMCP with an STL-robustness-based heuristic.
 *
 * Extends FIRMCP by replacing the FIRM global cost-to-go heuristic with a
 * blend of:
 *   - the original FIRM heuristic  J̃_g(b→B_j)
 *   - an STL smooth-robustness penalty  -ρ * stlCostScale_
 *
 * The cost for a neighbor action (from → to) becomes:
 *
 *   h = (1 − λ) · firmCost  +  λ · (−ρ · scale)
 *
 * where ρ = phi.eval_smooth_rob(smoothTau_, smoothType_).
 * Canonically, ρ > 0 when the STL spec is satisfied (reward) so −ρ converts it
 * to a cost (lower is better), matching the FIRMCP min-cost-to-go formulation.
 *
 * The STL specification is a conjunction of:
 *   φ_info  : G[0,1](cov_max  − tr_cov   ≥ 0)  – belief covariance stays below cov_max
 *   φ_reach : F[0,1](eps_goal − dist_goal ≥ 0)  – robot reaches within eps_goal of goal
 *
 * Both signals are evaluated on a minimal two-point trace:
 *   t=0 : state at vertex `from`
 *   t=1 : state at vertex `to`
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


protected:

    /**
     * \brief Override FIRMCP's hook.
     *
     * Returns  (1−λ)·firmCost + λ·(−ρ·scale)
     * where firmCost = edgeCost + getCostToGoWithApproxStabCost(to)
     * and   ρ        = smooth robustness of phi along the 2-point trace (from→to).
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
     * \brief Evaluate smooth STL robustness on the 2-point trace from→to.
     * \return ρ  (positive = satisfied, negative = violated)
     */
    double evaluateSTLRobustness(const Vertex from, const Vertex to);

    // -----------------------------------------------------------------------
    // Parameters (loaded from YAML stl_firmcp section)
    // -----------------------------------------------------------------------

    /** STL formula base string (signal/param names already pre-registered). */
    std::string stlFormulaStr_;

    /** Maximum acceptable trace(P) – belief quality threshold. */
    double covMax_;

    /** Goal-reaching distance threshold [same units as state space]. */
    double epsGoal_;

    /**
     * Blend weight λ ∈ [0,1].
     * 0 → pure FIRM heuristic, 1 → pure STL cost.
     */
    double lambdaStl_;

    /**
     * Scale factor to bring STL robustness into the same numerical range as
     * the FIRM cost-to-go (e.g., obstacle_cost_to_go ≈ 20 000).
     */
    double stlCostScale_;

    /** Temperature for smooth approximation (higher = closer to exact min/max). */
    double smoothTau_;

    /** Smooth approximation type: LSE (log-sum-exp) or MM (max-min). */
    STLRom::SmoothType smoothType_;

    /** Cached goal position extracted from goalM_[0] at first evaluation. */
    double goalX_, goalY_;
    bool goalCached_;
};

#endif // STLFIRMCP_PLANNER_H
