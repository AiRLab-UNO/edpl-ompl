/*
 * Minimal omplapp stub: RigidBodyGeometry
 * Uses assimp for mesh loading and FCL for collision detection.
 */
#ifndef OMPLAPP_RIGID_BODY_GEOMETRY_H
#define OMPLAPP_RIGID_BODY_GEOMETRY_H

#include <string>
#include <vector>
#include <memory>
#include <functional>
#include <stdexcept>
#include <boost/bind/bind.hpp>
using namespace boost::placeholders;

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/base/State.h>

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

#include <fcl/fcl.h>
#include <fcl/geometry/bvh/BVH_model.h>
#include <fcl/narrowphase/collision.h>

namespace ompl { namespace app {

enum MotionModel { Motion_2D, Motion_3D };
enum CollisionChecker { FCL, PQP };

using GeometricStateExtractor =
    std::function<const ompl::base::State*(const ompl::base::State*, unsigned int)>;

using FclBVH = ::fcl::BVHModel<::fcl::OBBRSS<double>>;

inline std::shared_ptr<FclBVH> loadMeshFCL(const std::string& filename)
{
    Assimp::Importer importer;
    const aiScene* scene = importer.ReadFile(filename,
        aiProcess_Triangulate | aiProcess_JoinIdenticalVertices);
    if (!scene || !scene->HasMeshes())
        throw std::runtime_error("Failed to load mesh: " + filename);

    auto model = std::make_shared<FclBVH>();
    model->beginModel();
    for (unsigned m = 0; m < scene->mNumMeshes; ++m)
    {
        const aiMesh* mesh = scene->mMeshes[m];
        for (unsigned f = 0; f < mesh->mNumFaces; ++f)
        {
            const aiFace& face = mesh->mFaces[f];
            if (face.mNumIndices != 3) continue;
            ::fcl::Vector3<double> v0(mesh->mVertices[face.mIndices[0]].x,
                                      mesh->mVertices[face.mIndices[0]].y,
                                      mesh->mVertices[face.mIndices[0]].z);
            ::fcl::Vector3<double> v1(mesh->mVertices[face.mIndices[1]].x,
                                      mesh->mVertices[face.mIndices[1]].y,
                                      mesh->mVertices[face.mIndices[1]].z);
            ::fcl::Vector3<double> v2(mesh->mVertices[face.mIndices[2]].x,
                                      mesh->mVertices[face.mIndices[2]].y,
                                      mesh->mVertices[face.mIndices[2]].z);
            model->addTriangle(v0, v1, v2);
        }
    }
    model->endModel();
    return model;
}

inline ompl::base::RealVectorBounds meshBounds(const std::string& filename)
{
    Assimp::Importer importer;
    const aiScene* scene = importer.ReadFile(filename, aiProcess_Triangulate);
    ompl::base::RealVectorBounds b(2);
    double minX = 1e9, maxX = -1e9, minY = 1e9, maxY = -1e9;
    if (scene && scene->HasMeshes())
    {
        for (unsigned m = 0; m < scene->mNumMeshes; ++m)
        {
            const aiMesh* mesh = scene->mMeshes[m];
            for (unsigned v = 0; v < mesh->mNumVertices; ++v)
            {
                minX = std::min(minX, (double)mesh->mVertices[v].x);
                maxX = std::max(maxX, (double)mesh->mVertices[v].x);
                minY = std::min(minY, (double)mesh->mVertices[v].y);
                maxY = std::max(maxY, (double)mesh->mVertices[v].y);
            }
        }
        b.setLow(0, minX); b.setHigh(0, maxX);
        b.setLow(1, minY); b.setHigh(1, maxY);
    }
    else
    {
        b.setLow(0.0);
        b.setHigh(20.0);
    }
    return b;
}

class RigidBodyGeometry
{
public:
    RigidBodyGeometry(MotionModel /*mm*/ = Motion_2D,
                      CollisionChecker /*cc*/ = FCL) {}

    bool addEnvironmentMesh(const std::string& file)
    {
        envFile_ = file;
        try { envModel_ = loadMeshFCL(file); } catch(...) {}
        return true;
    }

    bool setEnvironmentMesh(const std::string& file)
    {
        return addEnvironmentMesh(file);
    }

    void setRobotMesh(const std::string& file)
    {
        robotFile_ = file;
        try { robotModel_ = loadMeshFCL(file); } catch(...) {}
    }

    bool hasEnvironment() const { return !envFile_.empty(); }
    bool hasRobot()       const { return !robotFile_.empty(); }

    ompl::base::RealVectorBounds inferEnvironmentBounds() const
    {
        if (!envFile_.empty())
            try { return meshBounds(envFile_); } catch(...) {}
        ompl::base::RealVectorBounds b(2);
        b.setLow(0.0); b.setHigh(20.0);
        return b;
    }

    ompl::base::StateValidityCheckerPtr allocStateValidityChecker(
        const ompl::base::SpaceInformationPtr& si,
        const GeometricStateExtractor& /*ext*/,
        bool /*selfCollision*/) const;

    struct GeomSpec
    {
        std::shared_ptr<FclBVH> robotModel;
        std::shared_ptr<FclBVH> envModel;
    };

    GeomSpec getGeometrySpecification() const
    {
        return { robotModel_, envModel_ };
    }

    std::string getEnvFile()   const { return envFile_; }
    std::string getRobotFile() const { return robotFile_; }

protected:
    std::string envFile_;
    std::string robotFile_;
    std::shared_ptr<FclBVH> envModel_;
    std::shared_ptr<FclBVH> robotModel_;
};

} } // namespace ompl::app

#include "omplapp/geometry/detail/FCLStateValidityChecker.h"

namespace ompl { namespace app {

inline ompl::base::StateValidityCheckerPtr
RigidBodyGeometry::allocStateValidityChecker(
    const ompl::base::SpaceInformationPtr& si,
    const GeometricStateExtractor& ext,
    bool selfCollision) const
{
    return std::make_shared<FCLStateValidityChecker<Motion_2D>>(
        si, getGeometrySpecification(), ext, selfCollision);
}

} }

#endif
