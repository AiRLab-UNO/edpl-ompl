#include "STL/STLGuidedFIRMCP.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include <armadillo>
#include <boost/foreach.hpp>

#include "Spaces/SE2BeliefSpace.h"
#include "Weight/FIRMWeight.h"

namespace firmcp_stl
{

namespace
{
arma::vec2 stateXY(const ompl::base::State* s)
{
    const auto* se2 = s->as<SE2BeliefSpace::StateType>();
    return arma::vec2{se2->getX(), se2->getY()};
}

// Linearly interpolate a (current -> target) trajectory at the given step count.
std::vector<arma::vec2> sampleSegment(const arma::vec2& a, const arma::vec2& b, int steps = 10)
{
    std::vector<arma::vec2> traj;
    traj.reserve(steps + 1);
    for (int i = 0; i <= steps; ++i)
    {
        double t = static_cast<double>(i) / steps;
        traj.push_back(a + t * (b - a));
    }
    return traj;
}
} // namespace

FIRM::Edge STLGuidedFIRMCP::generateRolloutPolicy(const FIRM::Vertex currentVertex,
                                                  const FIRM::Vertex goal)
{
    double minCost   = std::numeric_limits<double>::max();
    Edge   edgeToTake;
    bool   anyEdge = false;

    arma::vec2 here = stateXY(stateProperty_[currentVertex]);

    BOOST_FOREACH(Edge e, boost::out_edges(currentVertex, g_))
    {
        Vertex target = boost::target(e, g_);

        if (!isFeedbackPolicyValid(target, goal))
            updateCostToGoWithApproxStabCost(target);

        double nextNodeCostToGo = getCostToGoWithApproxStabCost(target);
        FIRMWeight w           = boost::get(boost::edge_weight, g_, e);
        double  pSucc           = w.getSuccessProbability();

        double stationaryPenalty = 0.0;
        auto itp = stationaryPenalties_.find(target);
        if (itp != stationaryPenalties_.end())
            stationaryPenalty = itp->second;

        double baseCost = pSucc * nextNodeCostToGo
                          + (1.0 - pSucc) * obstacleCostToGo_
                          + w.getCost()
                          + stationaryPenalty;

        // STL bias: trajectory from current pose to candidate target
        arma::vec2 there = stateXY(stateProperty_[target]);
        auto seg         = sampleSegment(here, there);
        double rob       = stlEval_ ? stlEval_->score(seg) : 0.0;
        if (!std::isfinite(rob))
            rob = -1e3;

        double biasedCost = baseCost - alpha_ * rob;

        if (biasedCost < minCost)
        {
            minCost    = biasedCost;
            edgeToTake = e;
            anyEdge    = true;
        }
    }

    if (!anyEdge)
        return Edge();
    return edgeToTake;
}

FIRM::Edge STLBOWFIRMCP::generateRolloutPolicy(const FIRM::Vertex currentVertex,
                                               const FIRM::Vertex goal)
{
    ++visitCount_;

    arma::vec2 here = stateXY(stateProperty_[currentVertex]);

    // Score every candidate edge by an STLBOW-style acquisition:
    //   acq(edge) = STL_rob(traj) + beta * cost_pressure + ucbC * sqrt(log(N+1)/(n+1))
    //
    // The cost pressure folds in the FIRMCP value estimate so the planner still
    // respects collision/odometry economics while sampling per-spec progress.
    struct Cand
    {
        Edge   edge;
        double rob;
        double base;
        int    visits = 0;
    };
    std::vector<Cand> candidates;

    BOOST_FOREACH(Edge e, boost::out_edges(currentVertex, g_))
    {
        Vertex target = boost::target(e, g_);

        if (!isFeedbackPolicyValid(target, goal))
            updateCostToGoWithApproxStabCost(target);

        double nextNodeCostToGo = getCostToGoWithApproxStabCost(target);
        FIRMWeight w           = boost::get(boost::edge_weight, g_, e);
        double pSucc            = w.getSuccessProbability();
        double stationaryPenalty = 0.0;
        auto itp = stationaryPenalties_.find(target);
        if (itp != stationaryPenalties_.end())
            stationaryPenalty = itp->second;

        double baseCost = pSucc * nextNodeCostToGo
                          + (1.0 - pSucc) * obstacleCostToGo_
                          + w.getCost()
                          + stationaryPenalty;

        arma::vec2 there = stateXY(stateProperty_[target]);
        auto seg         = sampleSegment(here, there);
        double rob       = stlEval_ ? stlEval_->score(seg) : 0.0;
        if (!std::isfinite(rob))
            rob = -1e3;

        candidates.push_back({e, rob, baseCost});
    }

    if (candidates.empty())
        return Edge();

    // Track per-candidate UCB visits within this planner instance.
    static thread_local std::map<Edge, int, bool(*)(const Edge&, const Edge&)> visits(
        [](const Edge& a, const Edge& b) { return &a < &b; });

    int total = 0;
    for (auto& c : candidates)
        total += (c.visits = visits[c.edge]);

    double bestScore = -std::numeric_limits<double>::infinity();
    Edge   bestEdge  = candidates.front().edge;
    for (auto& c : candidates)
    {
        double ucb = ucbC_ * std::sqrt(std::log(static_cast<double>(total + 1))
                                      / static_cast<double>(c.visits + 1));
        double score = c.rob - beta_ * c.base + ucb;
        if (score > bestScore)
        {
            bestScore = score;
            bestEdge  = c.edge;
        }
    }
    visits[bestEdge]++;
    return bestEdge;
}

} // namespace firmcp_stl
