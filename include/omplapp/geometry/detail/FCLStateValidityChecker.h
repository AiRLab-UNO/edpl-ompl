/*
 * Minimal omplapp stub: FCLStateValidityChecker
 * Performs SE2 collision detection using FCL BVH models.
 */
#ifndef OMPLAPP_FCL_STATE_VALIDITY_CHECKER_H
#define OMPLAPP_FCL_STATE_VALIDITY_CHECKER_H

#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/SpaceInformation.h>
#include <fcl/fcl.h>
#include <fcl/narrowphase/collision.h>
#include <cmath>

// SE2BeliefSpace is included via the main chain, but we need the state type.
// Use a raw pointer cast approach: SE2 state layout is [x, y, theta, cov...]
// Specifically, SE2BeliefSpace::StateType stores values via setXY/setYaw.
// We access x=values[0], y=values[1], theta=values[2] via CompoundStateSpace internals,
// or better: include the header here.
#include "Spaces/SE2BeliefSpace.h"

namespace ompl { namespace app {

template<MotionModel MM>
class FCLStateValidityChecker : public ompl::base::StateValidityChecker
{
public:
    FCLStateValidityChecker(const ompl::base::SpaceInformationPtr& si,
                            const RigidBodyGeometry::GeomSpec& spec,
                            const GeometricStateExtractor& /*ext*/,
                            bool /*selfCollision*/)
        : ompl::base::StateValidityChecker(si)
        , envModel_(spec.envModel)
        , robotModel_(spec.robotModel)
    {
    }

    bool isValid(const ompl::base::State* state) const override
    {
        if (!envModel_ || !robotModel_)
            return true;

        const SE2BeliefSpace::StateType* s =
            state->as<SE2BeliefSpace::StateType>();
        double x     = s->getX();
        double y     = s->getY();
        double theta = s->getYaw();

        ::fcl::Transform3<double> envTf   = ::fcl::Transform3<double>::Identity();
        ::fcl::Transform3<double> robotTf = ::fcl::Transform3<double>::Identity();

        ::fcl::AngleAxis<double> rot(theta, ::fcl::Vector3<double>(0, 0, 1));
        robotTf.linear()      = rot.toRotationMatrix();
        robotTf.translation() = ::fcl::Vector3<double>(x, y, 0);

        ::fcl::CollisionObject<double> envObj(envModel_,   envTf);
        ::fcl::CollisionObject<double> robObj(robotModel_, robotTf);

        ::fcl::CollisionRequest<double> req;
        ::fcl::CollisionResult<double>  res;
        ::fcl::collide(&robObj, &envObj, req, res);

        return !res.isCollision();
    }

private:
    std::shared_ptr<::fcl::BVHModel<::fcl::OBBRSS<double>>> envModel_;
    std::shared_ptr<::fcl::BVHModel<::fcl::OBBRSS<double>>> robotModel_;
};

} } // namespace ompl::app

#endif
