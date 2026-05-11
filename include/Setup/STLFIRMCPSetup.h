#ifndef FIRMCP_SETUP_STL_H
#define FIRMCP_SETUP_STL_H

#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <boost/filesystem.hpp>
#include <boost/math/constants/constants.hpp>

#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>

#include "Filters/ExtendedKF.h"
#include "Filters/LinearizedKF.h"
#include "MotionModels/OmnidirectionalMotionModel.h"
#include "MotionModels/OmnidirectionalStatePropagator.h"
#include "ObservationModels/HeadingBeaconObservationModel.h"
#include "Planner/FIRMCP.h"
#include "SeparatedControllers/RHCICreate.h"
#include "SeparatedControllers/StationaryLQR.h"
#include "SpaceInformation/SpaceInformation.h"
#include "Spaces/SE2BeliefSpace.h"

#include "STL/SpecLandmarks.h"
#include "STL/STLGuidedFIRMCP.h"
#include "STL/STLRobustness.h"
#include "STL/STLStateValidityChecker.h"
#include "Visualization/Visualizer.h"

#include "common/ParamManager.h"

// Lightweight, mesh-free setup: drives FIRMCP from an STL spec + yaml config.
// - Landmarks are sourced from the spec's gx*/gy* params.
// - Collision checking uses stl_bow_v2's QuadtreeCollisionChecker.
// - Motion / observation models still need an XML; we synthesise a tiny one
//   in the run folder from yaml/spec params.
class STLFIRMCPSetup
{
    typedef SE2BeliefSpace::StateType StateType;

public:
    enum class RolloutVariant { FIRMCP_DEFAULT, STL_GUIDED, STL_BOW };

    STLFIRMCPSetup(const std::string& yamlPath,
                   const std::string& specPath,
                   const std::string& runDir,
                   RolloutVariant     variant = RolloutVariant::FIRMCP_DEFAULT)
        : yamlPath_(yamlPath)
        , specPath_(specPath)
        , runDir_(runDir)
        , variant_(variant)
    {
        boost::filesystem::create_directories(runDir_);

        // Static controller / state-type parameters (copied from TwoDPointRobotSetup ctor)
        Controller<RHCICreate, ExtendedKF>::setNodeReachedAngle(10.0);
        Controller<RHCICreate, ExtendedKF>::setNodeReachedDistance(0.1);
        Controller<RHCICreate, ExtendedKF>::setMaxTries(30);
        Controller<RHCICreate, ExtendedKF>::setMaxTrajectoryDeviation(1.0);
        Controller<StationaryLQR, LinearizedKF>::setNodeReachedAngle(10.0);
        Controller<StationaryLQR, LinearizedKF>::setNodeReachedDistance(0.1);
        Controller<StationaryLQR, LinearizedKF>::setMaxTries(300);
        Controller<StationaryLQR, LinearizedKF>::setMaxTrajectoryDeviation(2.5);
        Controller<FiniteTimeLQR, ExtendedKF>::setNodeReachedAngle(10.0);
        Controller<FiniteTimeLQR, ExtendedKF>::setNodeReachedDistance(0.1);
        Controller<FiniteTimeLQR, ExtendedKF>::setMaxTries(30);
        Controller<FiniteTimeLQR, ExtendedKF>::setMaxTrajectoryDeviation(0.5);
        RHCICreate::setControlQueueSize(5);
        RHCICreate::setTurnOnlyDistance(0.01);

        StateType::covNormWeight_  = 1.0;
        StateType::meanNormWeight_ = 2.0;
        StateType::reachDist_      = 0.009;
        StateType::reachDistPos_   = 0.1;
        StateType::reachDistOri_   = 10.0 / 180.0 * boost::math::constants::pi<double>();
        StateType::reachDistCov_   = 0.0004;

        arma::colvec normWeights(3);
        normWeights(0) = 2.0 / std::sqrt(9);
        normWeights(1) = 2.0 / std::sqrt(9);
        normWeights(2) = 1.0 / std::sqrt(9);
        StateType::normWeights_ = normWeights;

        pm_       = std::make_shared<param_manager>(yamlPath_);
        spec_     = std::make_shared<firmcp_stl::SpecLandmarks>(specPath_);
        stlEval_  = std::make_shared<firmcp_stl::STLRobustness>(spec_->specText(),
                                                                pm_->get_param<double>("dt"),
                                                                spec_->obstacles());

        synthesizeBridgeXML();
    }

    void setup()
    {
        if (setup_)
            return;

        // ---- state / control space -------------------------------------------------
        auto se2 = std::make_shared<SE2BeliefSpace>();
        ompl::base::RealVectorBounds bounds(2);
        auto flat = pm_->get_param<std::vector<double>>("boundary"); // [xmin,xmax,ymin,ymax]
        if (flat.size() < 4)
            throw std::runtime_error("STLFIRMCPSetup: 'boundary' must have 4 numbers (xmin,xmax,ymin,ymax)");
        bounds.setLow(0, flat[0]);
        bounds.setHigh(0, flat[1]);
        bounds.setLow(1, flat[2]);
        bounds.setHigh(1, flat[3]);
        se2->setBounds(bounds);

        ss_ = se2;
        cs_ = std::make_shared<ompl::control::RealVectorControlSpace>(ss_, 3);

        siF_ = std::make_shared<firm::SpaceInformation>(ss_, cs_);

        // Visualizer is a static singleton; it only records the executed path
        // when its `si_` is set. Wire it up here so robot_path lands in the DB.
        Visualizer::updateSpaceInformation(siF_);

        // ---- collision (stl_bow_v2) ------------------------------------------------
        cc_  = firmcp_stl::makeQuadtreeChecker(pm_);
        auto svc = std::make_shared<firmcp_stl::STLStateValidityChecker>(siF_, pm_, cc_);
        siF_->setStateValidityChecker(svc);

        // ---- observation model: spec-derived landmarks ----------------------------
        auto obs = std::make_shared<HeadingBeaconObservationModel>(siF_);
        obs->setLandmarks(spec_->landmarks());
        obs->setObservationNoise(/*sigma_ss=*/0.005, /*sigma_heading=*/1e-6);
        siF_->setObservationModel(obs);

        // ---- motion model + propagator (use the synthesised bridge XML) -----------
        auto mm = std::make_shared<OmnidirectionalMotionModel>(siF_, bridgeXMLPath_.c_str());
        siF_->setMotionModel(mm);

        auto prop = std::make_shared<OmnidirectionalStatePropagator>(siF_);
        siF_->setStatePropagator(prop);
        siF_->setPropagationStepSize(pm_->get_param<double>("dt"));
        siF_->setStateValidityCheckingResolution(0.005);
        siF_->setMinMaxControlDuration(1, 100);

        // ---- start / goal ---------------------------------------------------------
        ompl::base::State* start = siF_->allocState();
        auto rxy = spec_->startPose();
        start->as<StateType>()->setXY(rxy(0), rxy(1));
        start->as<StateType>()->setYaw(rxy(2));

        const auto& goals = spec_->orderedGoals();
        if (goals.empty())
            throw std::runtime_error("STLFIRMCPSetup: spec has no targets (gx*, gy*)");

        ompl::base::State* goal = siF_->allocState();
        goal->as<StateType>()->setXY(goals.back()[0], goals.back()[1]);

        pdef_ = std::make_shared<ompl::base::ProblemDefinition>(siF_);
        pdef_->setStartAndGoalStates(start, goal, 1.0);

        // ---- planner (variant chooses which FIRMCP subclass) ----------------------
        switch (variant_)
        {
            case RolloutVariant::STL_GUIDED:
                planner_ = std::make_shared<firmcp_stl::STLGuidedFIRMCP>(siF_, stlEval_);
                break;
            case RolloutVariant::STL_BOW:
                planner_ = std::make_shared<firmcp_stl::STLBOWFIRMCP>(siF_, stlEval_);
                break;
            case RolloutVariant::FIRMCP_DEFAULT:
            default:
                planner_ = std::make_shared<FIRMCP>(siF_, false);
                break;
        }
        planner_->setProblemDefinition(pdef_);
        planner_->as<FIRMCP>()->setMinFIRMNodes(minNodes_);
        planner_->as<FIRMCP>()->setMaxFIRMNodes(maxNodes_);
        planner_->as<FIRMCP>()->loadParametersFromFile(bridgeXMLPath_.c_str());
        planner_->setup();

        siF_->freeState(start);
        siF_->freeState(goal);

        setup_ = true;
    }

    ompl::base::PlannerStatus solve(double maxTime = 60.0)
    {
        if (!setup_) setup();
        return planner_->solve(maxTime);
    }

    // Run the chosen rollout/execution policy. Returns true on success.
    bool runExecution()
    {
        if (!planner_) return false;
        if (maxExecutionSteps_ > 0)
            planner_->as<FIRMCP>()->setMaxExecutionSteps(maxExecutionSteps_);
        if (maxGraphVertices_ > 0)
            planner_->as<FIRMCP>()->setMaxGraphVertices(maxGraphVertices_);

        // Visualizer keeps the executed trajectory in a static buffer; clear
        // it so each variant only sees its own path.
        Visualizer::clearRobotPath();

        bool ok = false;
        switch (variant_)
        {
            case RolloutVariant::FIRMCP_DEFAULT:
                planner_->as<FIRMCP>()->executeFeedbackWithRollout();
                ok = true;
                break;
            case RolloutVariant::STL_GUIDED:
            case RolloutVariant::STL_BOW:
                planner_->as<FIRMCP>()->executeFeedbackWithPOMCP();
                ok = true;
                break;
        }

        // Dump the executed trajectory into the variant's results.db so
        // scripts/visualize.py finds a `robot_path` table to plot.
        const std::string logPath = planner_->as<FIRMCP>()->getLogFilePath();
        if (!logPath.empty())
            Visualizer::printRobotPathToFile(logPath);

        return ok;
    }

    void setMaxExecutionSteps(int n) { maxExecutionSteps_ = n; }
    void setMaxGraphVertices(int n)  { maxGraphVertices_  = n; }
    int  executionTimeStep() const
    {
        return planner_ ? planner_->as<FIRMCP>()->getExecutionTimeStep() : 0;
    }
    bool reachedGoal() const
    {
        return planner_ ? planner_->as<FIRMCP>()->reachedGoalDuringExecution() : false;
    }

    void saveRoadmap()
    {
        if (planner_) planner_->as<FIRMCP>()->savePlannerData();
    }

    const std::shared_ptr<firmcp_stl::SpecLandmarks>& spec() const { return spec_; }
    const std::shared_ptr<firmcp_stl::STLRobustness>& stlEval() const { return stlEval_; }
    const std::string& runDir() const { return runDir_; }

private:
    void synthesizeBridgeXML()
    {
        // Build an XML readable by OmnidirectionalMotionModel::loadParameters()
        // and FIRM/FIRMCP::loadParametersFromFile(). Values come from yaml/spec.
        const double dt          = pm_->get_param<double>("dt");
        const double maxV        = pm_->get_param<double>("max_speed");
        const double maxOmega    = pm_->get_param<double>("max_yawrate");

        std::ostringstream xml;
        xml << "<?xml version='1.0' encoding='UTF-8'?>\n";
        xml << "<MotionModels>\n";
        xml << "  <OmnidirectionalMotionModel sigmaV=\"0.0\" etaV=\"0.0\" sigmaOmega=\"0.0\" etaOmega=\"0.0\" "
            << "P_Wg_filename=\"\" wind_noise_pos=\"0.001\" wind_noise_ang=\"0.000001\" "
            << "min_linear_velocity=\"" << -maxV << "\" max_linear_velocity=\"" << maxV << "\" "
            << "max_angular_velocity=\"" << maxOmega << "\" dt=\"" << dt << "\" />\n";
        xml << "</MotionModels>\n";
        xml << "<FIRM>\n";
        xml << "  <Video save=\"0\" />\n";
        xml << "  <DataLog save=\"1\" folder=\"" << runDir_ << "/\" />\n";
        xml << "  <Roadmap save=\"1\" />\n";
        xml << "  <MCParticles numparticles=\"5\" />\n";
        xml << "  <RolloutSteps rolloutsteps=\"1\" />\n";
        xml << "  <NNRadius nnradius=\"2.25\" />\n";
        xml << "  <NumNN numnn=\"5\" />\n";
        xml << "  <DPDiscountFactor discountfac=\"1.0\" />\n";
        xml << "  <DistCostWeight distcostw=\"0.1\" />\n";
        xml << "  <StabilizationHack connectToFutureNodes=\"0\" applyStationaryPenalty=\"1\" borderBeliefSampling=\"0\" />\n";
        xml << "  <InfCostWeight infcostw=\"100.0\" />\n";
        xml << "  <TimeCostWeight timecostw=\"0.0\" />\n";
        xml << "  <StatCostInc statcostinc=\"1.0\" />\n";
        xml << "  <GoalCostToGo goalctg=\"0.0\" />\n";
        xml << "  <ObstCostToGo obsctg=\"20000.0\" />\n";
        xml << "  <InitCostToGo initctg=\"2.0\" />\n";
        xml << "  <MaxDPIter dpiter=\"1000\" />\n";
        xml << "  <DPConvergenceThreshold dpconvthresh=\"0.1\" />\n";
        xml << "</FIRM>\n";
        xml << "<FIRMCP>\n";
        xml << "  <numPOMCPParticles numPOMCPParticles=\"5\" />\n";
        xml << "  <maxPOMCPDepth maxPOMCPDepth=\"3\" />\n";
        xml << "  <maxFIRMReachDepth maxFIRMReachDepth=\"10\" />\n";
        xml << "  <nSigmaForPOMCPParticle nSigmaForPOMCPParticle=\"1.0\" />\n";
        xml << "  <cExplorationForSimulate cExplorationForSimulate=\"1.0\" />\n";
        xml << "  <cExploitationForRolloutOutOfReach cExploitationForRolloutOutOfReach=\"0.5\" />\n";
        xml << "  <cExploitationForRolloutWithinReach cExploitationForRolloutWithinReach=\"0.1\" />\n";
        xml << "  <costToGoRegulatorOutOfReach costToGoRegulatorOutOfReach=\"1.0\" />\n";
        xml << "  <costToGoRegulatorWithinReach costToGoRegulatorWithinReach=\"1.0\" />\n";
        xml << "  <nEpsilonForRolloutIsReached nEpsilonForRolloutIsReached=\"1.0\" />\n";
        xml << "  <heurPosStepSize heurPosStepSize=\"0.5\" />\n";
        xml << "  <heurOriStepSize heurOriStepSize=\"0.1\" />\n";
        xml << "  <heurCovStepSize heurCovStepSize=\"0.01\" />\n";
        xml << "  <covConvergenceRate covConvergenceRate=\"0.9\" />\n";
        xml << "  <scaleStabNumSteps scaleStabNumSteps=\"1\" />\n";
        xml << "  <nEpsilonForQVnodeMerging nEpsilonForQVnodeMerging=\"1.0\" />\n";
        xml << "  <inflationForApproxStabCost inflationForApproxStabCost=\"1\" />\n";
        xml << "</FIRMCP>\n";

        bridgeXMLPath_ = runDir_ + "/setup_bridge.xml";
        std::ofstream out(bridgeXMLPath_);
        out << xml.str();
    }

    std::string yamlPath_;
    std::string specPath_;
    std::string runDir_;
    std::string bridgeXMLPath_;
    RolloutVariant variant_;
    bool        setup_ = false;

    unsigned int minNodes_ = 100;
    unsigned int maxNodes_ = 250;
    int          maxExecutionSteps_ = 0;    // <=0 = unbounded
    int          maxGraphVertices_  = 5000; // OOM guard for POMCP tree growth

    std::shared_ptr<param_manager>             pm_;
    std::shared_ptr<firmcp_stl::SpecLandmarks> spec_;
    std::shared_ptr<firmcp_stl::STLRobustness> stlEval_;
    std::shared_ptr<BaseCollisionChecker>      cc_;

    ompl::base::StateSpacePtr             ss_;
    ompl::control::ControlSpacePtr        cs_;
    firm::SpaceInformation::SpaceInformationPtr siF_;
    ompl::base::ProblemDefinitionPtr      pdef_;
    ompl::base::PlannerPtr                planner_;
};

#endif // FIRMCP_SETUP_STL_H
