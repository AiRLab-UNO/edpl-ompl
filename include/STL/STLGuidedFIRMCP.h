#ifndef FIRMCP_STL_GUIDED_FIRMCP_H
#define FIRMCP_STL_GUIDED_FIRMCP_H

#include <memory>
#include "Planner/FIRMCP.h"
#include "STL/STLRobustness.h"

namespace firmcp_stl
{

// Variant 1: FIRMCP whose rollout edge-selection is biased by STL robustness.
//
//   cost'(edge) = FIRMCP_cost(edge) - alpha * STL_rob(traj_to_target)
//
// Higher robustness shrinks the effective edge cost, steering rollouts toward
// edges whose target satisfies more of the STL spec.
class STLGuidedFIRMCP : public FIRMCP
{
public:
    STLGuidedFIRMCP(const firm::SpaceInformation::SpaceInformationPtr& si,
                    const std::shared_ptr<STLRobustness>&             stlEval,
                    double                                            alpha = 50.0)
        : FIRMCP(si, false)
        , stlEval_(stlEval)
        , alpha_(alpha)
    {
    }

protected:
    Edge generateRolloutPolicy(const Vertex currentVertex, const FIRM::Vertex goal) override;

private:
    std::shared_ptr<STLRobustness> stlEval_;
    double                         alpha_;
};

// Variant 2: FIRMCP whose rollout edge-selection follows an STLBOW-style policy.
// We score each candidate target by (a) STL robustness and (b) a UCB-style
// exploration bonus over the candidate count, mimicking the BOW acquisition
// without requiring a full BO loop per call.
class STLBOWFIRMCP : public FIRMCP
{
public:
    STLBOWFIRMCP(const firm::SpaceInformation::SpaceInformationPtr& si,
                 const std::shared_ptr<STLRobustness>&             stlEval,
                 double                                            beta = 1.0,
                 double                                            ucbC = 1.41)
        : FIRMCP(si, false)
        , stlEval_(stlEval)
        , beta_(beta)
        , ucbC_(ucbC)
    {
    }

protected:
    Edge generateRolloutPolicy(const Vertex currentVertex, const FIRM::Vertex goal) override;

private:
    std::shared_ptr<STLRobustness> stlEval_;
    double                         beta_;
    double                         ucbC_;
    int                            visitCount_ = 0;
};

} // namespace firmcp_stl

#endif // FIRMCP_STL_GUIDED_FIRMCP_H
