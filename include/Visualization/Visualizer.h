/*
 * Visualizer stub — no Qt/GL dependencies.
 * All drawing calls are no-ops; robot path is recorded for SQLite export.
 */
#ifndef FIRM_OMPL_VISUALIZER_H
#define FIRM_OMPL_VISUALIZER_H

#include <armadillo>
#include <list>
#include <vector>
#include <string>
#include <boost/thread/mutex.hpp>
#include <boost/optional.hpp>
#include <ompl/geometric/PathGeometric.h>
#include "Spaces/SE2BeliefSpace.h"
#include "SpaceInformation/SpaceInformation.h"

class Visualizer
{
public:
    Visualizer(){}
    ~Visualizer(){}

    enum VZRStateType  { TrueState, BeliefState, GraphNodeState };
    enum VZRDrawingMode {
        NodeViewMode, FeedbackViewMode, PRMViewMode,
        FIRMMode, FIRMCPMode, RolloutMode, MultiModalMode
    };

    struct VZRFeedbackEdge {
        ompl::base::State *source;
        ompl::base::State *target;
        double cost;
    };

    static void addLandmarks(const std::vector<arma::colvec>&) {}
    static void addState(const ompl::base::State*) {}
    static void clearStates() {}
    static void addBeliefMode(ompl::base::State*) {}
    static void clearBeliefModes() {}
    static void addGraphEdge(const ompl::base::State*, const ompl::base::State*) {}
    static void addFeedbackEdge(ompl::base::State*, ompl::base::State*, double) {}
    static void addRolloutConnection(const ompl::base::State*, const ompl::base::State*) {}
    static void addMostLikelyPathEdge(const ompl::base::State*, const ompl::base::State*) {}
    static void setChosenRolloutConnection(const ompl::base::State*, const ompl::base::State*) {}
    static void setMode(VZRDrawingMode m) { mode_ = m; }
    static void ClearFeedbackEdges() {}
    static void clearRolloutConnections() {}
    static void clearMostLikelyPath() {}
    static void addOpenLoopRRTPath(const ompl::geometric::PathGeometric) {}
    static void clearOpenLoopRRTPaths() {}
    static void drawRobotPath() {}

    static void updateTrueState(const ompl::base::State* state)
    {
        if (si_)
        {
            boost::mutex::scoped_lock sl(drawMutex_);
            if (!trueState_) trueState_ = si_->allocState();
            si_->copyState(trueState_, state);
            robotPath_.push_back(si_->cloneState(state));
        }
    }

    static void updateCurrentBelief(const ompl::base::State* state)
    {
        if (si_)
        {
            boost::mutex::scoped_lock sl(drawMutex_);
            if (!currentBelief_) currentBelief_ = si_->allocState();
            si_->copyState(currentBelief_, state);
        }
    }

    static void updateSpaceInformation(const firm::SpaceInformation::SpaceInformationPtr si)
    {
        boost::mutex::scoped_lock sl(drawMutex_);
        si_ = si;
        trueState_    = si_->allocState();
        currentBelief_ = si_->allocState();
    }

    // Accept any renderer arguments (no-op)
    template<typename T, typename U>
    static void updateRenderer(const T&, const U&) {}

    static void clearRobotPath()
    {
        boost::mutex::scoped_lock sl(drawMutex_);
        for (auto s : robotPath_) si_->freeState(s);
        robotPath_.clear();
    }

    static bool saveVideo()    { return false; }
    static void doSaveVideo(bool) {}
    static void refresh()      {}

    static void printRobotPathToFile(const std::string& logPath);

private:
    static boost::mutex                  drawMutex_;
    static ompl::base::State*            trueState_;
    static ompl::base::State*            currentBelief_;
    static std::vector<ompl::base::State*> robotPath_;
    static firm::SpaceInformation::SpaceInformationPtr si_;
    static VZRDrawingMode                mode_;
    static bool                          saveVideo_;
};

#endif // FIRM_OMPL_VISUALIZER_H
