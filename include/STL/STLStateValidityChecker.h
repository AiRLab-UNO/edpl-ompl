#ifndef FIRMCP_STL_STATE_VALIDITY_CHECKER_H
#define FIRMCP_STL_STATE_VALIDITY_CHECKER_H

#include <Eigen/Dense>
#include <memory>
#include <vector>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateValidityChecker.h>

#include "Spaces/SE2BeliefSpace.h"

class BaseCollisionChecker;
class param_manager;

namespace firmcp_stl
{

// Adapts stl_bow_v2's BaseCollisionChecker (yaml-driven, e.g. Quadtree backend)
// into an OMPL state validity checker for the SE2BeliefSpace.
//
// We treat any state as a single-point trajectory; the underlying checker
// validates a Eigen::VectorXd[5] = [x, y, theta, v, omega] sequence.
class STLStateValidityChecker : public ompl::base::StateValidityChecker
{
public:
    STLStateValidityChecker(const ompl::base::SpaceInformationPtr&         si,
                            const std::shared_ptr<param_manager>&          pm,
                            const std::shared_ptr<BaseCollisionChecker>&   checker);

    bool isValid(const ompl::base::State* state) const override;

private:
    std::shared_ptr<param_manager>        pm_;
    std::shared_ptr<BaseCollisionChecker> checker_;
};

// Helper: build a default Quadtree-backed collision checker from a yaml file.
std::shared_ptr<BaseCollisionChecker> makeQuadtreeChecker(const std::shared_ptr<param_manager>& pm);

} // namespace firmcp_stl

#endif // FIRMCP_STL_STATE_VALIDITY_CHECKER_H
