#include "STL/STLStateValidityChecker.h"

#include "collision/CollisionChecker.hh"
#include "common/ParamManager.h"

namespace firmcp_stl
{

STLStateValidityChecker::STLStateValidityChecker(const ompl::base::SpaceInformationPtr&       si,
                                                 const std::shared_ptr<param_manager>&        pm,
                                                 const std::shared_ptr<BaseCollisionChecker>& checker)
    : ompl::base::StateValidityChecker(si)
    , pm_(pm)
    , checker_(checker)
{
}

bool STLStateValidityChecker::isValid(const ompl::base::State* state) const
{
    if (!checker_)
        return true;

    const auto* se2 = state->as<SE2BeliefSpace::StateType>();
    Eigen::VectorXd x(5);
    x << se2->getX(), se2->getY(), se2->getYaw(), 0.0, 0.0;

    std::vector<Eigen::VectorXd> traj{x};
    return !checker_->isCollision(traj);
}

std::shared_ptr<BaseCollisionChecker> makeQuadtreeChecker(const std::shared_ptr<param_manager>& pm)
{
    return std::make_shared<quadtree::QuadtreeCollisionChecker>(pm);
}

} // namespace firmcp_stl
