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

/* Authors:  Saurav Agarwal, Ali-akbar Agha-mohammadi */


#include <yaml-cpp/yaml.h>
#include "Spaces/R2BeliefSpace.h"
#include "MotionModels/TwoDPointMotionModel.h"
#include "Utils/FIRMUtils.h"

#include<cassert>

//Produce the next state, given the current state, a control and a noise
void TwoDPointMotionModel::Evolve(const ompl::base::State *state, const ompl::control::Control *control, const NoiseType& w, ompl::base::State *result)
{

    using namespace arma;

    typedef typename MotionModelMethod::StateType StateType;

    arma::colvec u = OMPL2ARMA(control);

    const colvec& Un = w.subvec(0, this->controlDim_-1);

    const colvec& Wg = w.subvec(this->controlDim_, this->noiseDim_-1);

    colvec x = state->as<StateType>()->getArmaData();

    colvec Un2(2);
    Un2 << Un[0] << Un[1] << endr;

    x += (u*this->dt_) + (Un2*sqrt(this->dt_)) + (Wg*sqrt(this->dt_));

    result->as<StateType>()->setXY(x[0],x[1]);
}


void TwoDPointMotionModel::generateOpenLoopControls(const ompl::base::State *startState,
                                                  const ompl::base::State *endState,
                                                  std::vector<ompl::control::Control*> &openLoopControls)
{

    using namespace arma;
    typedef typename MotionModelMethod::StateType StateType;

    colvec start = startState->as<StateType>()->getArmaData(); // turn into colvec (in radian)
    colvec target = endState->as<StateType>()->getArmaData(); // turn into colvec (in radian)

    double delta_disp = 0;

    double translation_steps = 0;

    translation_steps = floor( std::max( fabs((target[0]-start[0])/(maxLinearVelocity_*this->dt_)), fabs((target[1]-start[1])/(maxLinearVelocity_*this->dt_))) );

    colvec u_const;
    u_const << (target[0]-start[0]) / (translation_steps*this->dt_) << endr
            << (target[1]-start[1]) / (translation_steps*this->dt_) << endr;

    for(int i=0; i<translation_steps; i++)
    {
      ompl::control::Control *tempControl = si_->allocControl();
      ARMA2OMPL(u_const, tempControl);
      openLoopControls.push_back(tempControl);
    }


}

void TwoDPointMotionModel::generateOpenLoopControlsForPath(const ompl::geometric::PathGeometric path, std::vector<ompl::control::Control*> &openLoopControls)
{
    for(int i=0;i<path.getStateCount()-1;i++)
    {
        std::vector<ompl::control::Control*> olc;

        this->generateOpenLoopControls(path.getState(i),path.getState(i+1),olc) ;

        openLoopControls.insert(openLoopControls.end(),olc.begin(),olc.end());
    }
}


typename TwoDPointMotionModel::NoiseType
TwoDPointMotionModel::generateNoise(const ompl::base::State *state, const ompl::control::Control* control)
{

    using namespace arma;

    NoiseType noise(this->noiseDim_);

    colvec indepUn = randn(this->controlDim_,1);
    
    mat P_Un = controlNoiseCovariance(control);
    
    colvec Un = indepUn % sqrt((P_Un.diag()));

    colvec Wg = sqrt(P_Wg_) * randn(this->stateDim_,1);
    
    noise = join_cols(Un, Wg);

    return noise;
}

typename TwoDPointMotionModel::JacobianType
TwoDPointMotionModel::getStateJacobian(const ompl::base::State *state, const ompl::control::Control* control, const NoiseType& w)
{

    using namespace arma;

    colvec xData = state->as<StateType>()->getArmaData();

    assert (xData.n_rows == (size_t)stateDim);

    JacobianType A = eye(this->stateDim_,this->stateDim_) ;

    return A;
}

typename TwoDPointMotionModel::JacobianType
TwoDPointMotionModel::getControlJacobian(const ompl::base::State *state, const ompl::control::Control* control, const NoiseType& w)
{

    using namespace arma;
    typedef typename MotionModelMethod::StateType StateType;

    colvec xData = state->as<StateType>()->getArmaData();
    assert (xData.n_rows == (size_t)this->stateDim_);

    mat B(this->stateDim_,this->controlDim_);

    B <<  1 <<   0  <<  endr
      <<  0  <<  1  << endr;

    B *= this->dt_;

    return B;

}


typename TwoDPointMotionModel::JacobianType
TwoDPointMotionModel::getNoiseJacobian(const ompl::base::State *state, const ompl::control::Control* control, const NoiseType& w)
{

    using namespace arma;
    typedef typename MotionModelMethod::StateType StateType;

    colvec xData = state->as<StateType>()->getArmaData();

    assert (xData.n_rows == (size_t)this->stateDim_);

    mat G(this->stateDim_,this->noiseDim_);

    G   << 1 << 0 << 1 << 0 << endr
        << 0 << 1 << 0 << 1 << endr;

    G *= sqrt(this->dt_);
    return G;

}

arma::mat TwoDPointMotionModel::processNoiseCovariance(const ompl::base::State *state, const ompl::control::Control* control)
{

    using namespace arma;

    mat P_Un = controlNoiseCovariance(control);

    mat Q_processNoise = zeros<mat>(P_Un.n_rows + P_Wg_.n_rows, P_Un.n_cols + P_Wg_.n_cols);

    Q_processNoise.submat(0, 0, P_Un.n_rows-1, P_Un.n_cols-1) = P_Un;
    Q_processNoise.submat(P_Un.n_rows, P_Un.n_cols,
                  P_Un.n_rows + P_Wg_.n_rows -1,
                  P_Un.n_cols + P_Wg_.n_cols -1) = P_Wg_;

    return Q_processNoise;

}


arma::mat TwoDPointMotionModel::controlNoiseCovariance(const ompl::control::Control* control)
{

    using namespace arma;

    arma::colvec u = OMPL2ARMA(control);

    colvec uStd = eta_ % u + sigma_;

    mat P_Un = diagmat(square(uStd));

    return P_Un;
}

void TwoDPointMotionModel::loadParameters(const char *pathToSetupFile)
{
    using namespace arma;

    YAML::Node config = YAML::LoadFile(pathToSetupFile);
    const auto& mm = config["motion_model"];

    double sigmaV            = mm["sigma_v"].as<double>();
    double etaV              = mm["eta_v"].as<double>();
    double windNoisePos      = mm["wind_noise_pos"].as<double>();
    double maxLinearVelocity = mm["max_linear_velocity"].as<double>();
    double dt                = mm["dt"].as<double>();

    this->sigma_ << sigmaV << sigmaV << endr;
    this->eta_   << etaV   << etaV   << endr;

    rowvec Wg_root_vec(2);
    Wg_root_vec << windNoisePos << windNoisePos << endr;
    P_Wg_ = diagmat(square(Wg_root_vec));

    maxLinearVelocity_ = maxLinearVelocity;
    dt_                = dt;

    OMPL_INFORM("TwoDPointMotionModel: max linear vel = %f, dt = %f", maxLinearVelocity_, dt_);
}
