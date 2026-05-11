/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, Texas A&M University
*  All rights reserved.
*
*  [license text same as parent class]
*
*********************************************************************/

/* Authors: Sung Kyun Kim et al. */

#ifndef TWODPOINTROBOT_SETUP_STLFIRMCP_H
#define TWODPOINTROBOT_SETUP_STLFIRMCP_H

#include "Setup/TwoDPointRobotFIRMCPSetup.h"
#include "Planner/STLFIRMCP.h"

/**
 * \brief Concrete setup that wires a TwoDPointRobot environment to
 *        the STLFIRMCP planner.
 *
 * Inherits all sensor/motion-model setup from TwoDPointRobotFIRMCPSetup but
 * swaps the planner instance from FIRMCP → STLFIRMCP.
 */
class TwoDPointRobotSTLFIRMCPSetup : public TwoDPointRobotFIRMCPSetup
{

public:

    TwoDPointRobotSTLFIRMCPSetup() : TwoDPointRobotFIRMCPSetup()
    {
    }

    virtual ~TwoDPointRobotSTLFIRMCPSetup(void)
    {
    }

    virtual void setup() override
    {
        if (!setup_)
        {
            this->loadParameters();

            if (pathToSetupFile_.length() == 0)
                throw ompl::Exception("Path to setup file not set!");

            if (!hasEnvironment() || !hasRobot())
                throw ompl::Exception("Robot/Environment mesh files not setup!");

            ss_->as<SE2BeliefSpace>()->setBounds(inferEnvironmentBounds());

            const ompl::base::StateValidityCheckerPtr &fclSVC =
                this->allocStateValidityChecker(siF_, getGeometricStateExtractor(), false);
            siF_->setStateValidityChecker(fclSVC);

            ObservationModelMethod::ObservationModelPointer om(
                new HeadingBeaconObservationModel(siF_, pathToSetupFile_.c_str()));
            siF_->setObservationModel(om);

            MotionModelMethod::MotionModelPointer mm(
                new OmnidirectionalMotionModel(siF_, pathToSetupFile_.c_str()));
            siF_->setMotionModel(mm);

            ompl::control::StatePropagatorPtr prop(
                ompl::control::StatePropagatorPtr(new OmnidirectionalStatePropagator(siF_)));
            statePropagator_ = prop;
            siF_->setStatePropagator(statePropagator_);
            siF_->setPropagationStepSize(0.1);
            siF_->setStateValidityCheckingResolution(0.005);
            siF_->setMinMaxControlDuration(1, 100);

            if (!start_ || goalList_.size() == 0)
                throw ompl::Exception("Start/Goal not set");

            pdef_->setStartAndGoalStates(start_, goalList_[0], 1.0);

            // ----------------------------------------------------------------
            // Create STLFIRMCP planner (instead of base FIRMCP)
            // ----------------------------------------------------------------
            ompl::base::PlannerPtr plnr(new STLFIRMCP(siF_, false));
            planner_ = plnr;
            planner_->setProblemDefinition(pdef_);

            planner_->as<STLFIRMCP>()->setMinFIRMNodes(minNodes_);
            planner_->as<STLFIRMCP>()->setMaxFIRMNodes(maxNodes_);
            planner_->as<STLFIRMCP>()->setKidnappedState(kidnappedState_);
            planner_->as<STLFIRMCP>()->loadParametersFromFile(pathToSetupFile_.c_str());

            planner_->setup();

            Visualizer::updateSpaceInformation(this->getSpaceInformation());
            Visualizer::updateRenderer(
                *dynamic_cast<const ompl::app::RigidBodyGeometry *>(this),
                this->getGeometricStateExtractor());

            if (useSavedRoadMap_ == 1)
                planner_->as<STLFIRMCP>()->loadRoadMapFromFile(pathToRoadMapFile_.c_str());

            setup_ = true;
        }
    }

    virtual void executeSolution(int choice = 0) override
    {
        switch (choice)
        {
            case 0:
                planner_->as<STLFIRMCP>()->executeFeedback();
                break;
            case 1:
                planner_->as<STLFIRMCP>()->executeFeedbackWithRollout();
                break;
            case 2:
                planner_->as<STLFIRMCP>()->executeFeedbackWithKidnapping();
                break;
            case 3:
                planner_->as<STLFIRMCP>()->executeFeedbackWithPOMCP();
                break;
            default:
                OMPL_ERROR("PlanningMode method %d is not valid... Check the setup file!", choice);
                exit(0);
                break;
        }
    }

    virtual void updateEnvironmentMesh(int obindx = 0) override
    {
        if (dynamicObstacles_)
        {
            if (!this->setEnvironmentMesh(dynObstList_[obindx]))
                OMPL_ERROR("Couldn't set mesh with path: %s", dynObstList_[obindx]);

            const ompl::base::StateValidityCheckerPtr &svc =
                std::make_shared<ompl::app::FCLStateValidityChecker<ompl::app::Motion_2D>>(
                    siF_, getGeometrySpecification(), getGeometricStateExtractor(), false);

            siF_->setStateValidityChecker(svc);
            planner_->as<STLFIRMCP>()->updateCollisionChecker(svc);
        }
    }

    virtual void saveRoadmap() override
    {
        planner_->as<STLFIRMCP>()->savePlannerData();
    }
};

#endif  // TWODPOINTROBOT_SETUP_STLFIRMCP_H
