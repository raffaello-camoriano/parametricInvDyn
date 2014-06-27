// Kdl_codyco libraries
#include <kdl_codyco/utils.hpp>
#include <kdl/rigidbodyinertia.hpp>
#include <kdl/frames_io.hpp>
#include <kdl_codyco/regressors/dataset/DynamicDatasetFile.hpp>
#include <kdl_codyco/regressors/dynamicRegressorGenerator.hpp>

//#include "iDynTree/iCubTree.h"


// C++ standard libraries
#include <cstdlib>

// icub libraries
#include <iCub/iDynTree/iCubTree.h>
#include <iCub/ctrl/filters.h>
#include <iCub/ctrl/adaptWinPolyEstimator.h>

#include <kdl_codyco/regressor_loops.hpp>
#include <kdl_codyco/regressor_utils.hpp>

// Gurls libraries necessary for machine learning algorithms
#include <gurls++/gurls.h>
#include <gurls++/exceptions.h>
#include <gurls++/gmat2d.h>
#include <gurls++/options.h>
#include <gurls++/optlist.h>

// Standard libraries
#include <iostream>
#include <fstream>

// yarp libraries
#include <yarp/os/Property.h>

// Linear estimator libararies
#include "multitaskRecursiveLinearEstimator.h"
#include "multitaskSVDLinearEstimator.h"

using namespace KDL;
using namespace KDL::CoDyCo;
using namespace KDL::CoDyCo::Regressors;
using namespace gurls;
using namespace std;

// defining some constant parameters

#define MODEL_DOFS 32
#define DATASET_DOFS 41

#define LEFT_ELBOW_TORQUE_INDEX 2
#define RIGHT_ELBOW_TORQUE_INDEX 8

#define RIGHT_SHOULDER_PITCH_TORQUE_INDEX 6

#define LEFT_ARM_FT_INDEX 0
#define RIGHT_ARM_FT_INDEX 1

#define RIGHT_ELBOW_GAIN 3.5290

#define RIGHT_SHOULDER_PITCH_GAIN 2.4191

/** Tolerance for considering two values equal */
const double ZERO_TOL = 1e-5;

// classes for reading the data file, modelling data and copying required information
bool convertJointsFromDatasetToModel(const KDL::JntArray & q_dataset, KDL::JntArray & q_model, bool verbose = false)
{
    if( q_dataset.rows() != DATASET_DOFS ) {
        if( verbose ) std::cout << "Q from dataset has size " << q_dataset.rows() << std::endl;
    }

    assert(q_dataset.rows() == DATASET_DOFS);

    assert(q_model.rows() == MODEL_DOFS);
    //Joints in dataset are : Torso (3), head(6), left arm (16), right_arm (16)
    //Joints in the model are : Torso (3), head(3), left arm (7), right arm (7), left_leg (6), right_leg(6)
    
    SetToZero(q_model);
    
    //Torso should be inverted
    q_model(0) = q_dataset(2);
    q_model(1) = q_dataset(1);
    q_model(2) = q_dataset(0);
    
    //Copyng head
    for(int i = 0; i < 3; i++ ) {
        q_model(3+i) = q_dataset(3+i);
    }
    
    //Copyng left arm
    for(int i = 0; i < 7; i++ ) {
        q_model(3+3+i) = q_dataset(3+6+i);
    }
    
    //Copyng right arm 
    for(int i = 0; i < 7; i++ ) {
        q_model(3+3+7+i) = q_dataset(3+6+16+i);
    }
    
    return true;
}
std::cout << q_model << std::endl;

bool toYarp(const KDL::Wrench & ft, yarp::sig::Vector & ft_yrp)
{
    if( ft_yrp.size() != 6 ) { ft_yrp.resize(6); }
    for(int i=0; i < 6; i++ ) {
        ft_yrp[i] = ft[i];
    }
}

bool toKDL(const yarp::sig::Vector & ft_yrp, KDL::Wrench & ft)
{
    for(int i=0; i < 6; i++ ) {
        ft[i] = ft_yrp[i];
    }
}

bool toYarp(const Eigen::VectorXd & vec_eigen, yarp::sig::Vector & vec_yrp)
{
    if( vec_yrp.size() != vec_eigen.size() ) { vec_yrp.resize(vec_eigen.size()); }
    if( memcpy(vec_yrp.data(),vec_eigen.data(),sizeof(double)*vec_eigen.size()) != NULL ) {
        return true;
    } else {
        return false;
    }
}

bool toEigen(const yarp::sig::Vector & vec_yrp, Eigen::VectorXd & vec_eigen)
{
 if( vec_yrp.size() != vec_eigen.size() ) { vec_eigen.resize(vec_yrp.size()); }
    if( memcpy(vec_eigen.data(),vec_yrp.data(),sizeof(double)*vec_eigen.size()) != NULL ) {
        return true;
    } else {
        return false;
    }
}


int main(int argc, char ** argv)
{
    yarp::os::Property opt;
    
    opt.fromCommand(argc,argv);
   
    if( !opt.check("dataset") ) {
        std::cout << "iCubParis02_data_analysis_simple: " << std::endl;
        std::cout << "usage: ./iCubParis02_data_analysis --dataset dataset.csv --results results.csv" << std::endl;
        std::cout << "additional options: --n_samples n the number of random regressors to generate for the numerical identifable subspace computation" << std::endl;
        std::cout << "additional options: --cut_freq the cut frequency to use for filtering torques and ft measures (in Hz, default 3 Hz)" << std::endl;
        std::cout << "additional options: --sampleTime the sample time of the data (in s, default: 0.01 s)" << std::endl;
        std::cout << "additional options: --lambda regularization parameter" << std::endl;
        std::cout << "additional options: --skip number of initial samples to skip" << std::endl;
        std::cout << "additional options: --verbose print debug information" << std::endl;
        std::cout << "additional options: --parameters param.csv output a file of trajectoreis of the estimated parameters" << std::endl;
        return 0;
    }
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //// Parameter parsing
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////
    int n_samples;
    if(!opt.check("n_samples")) {
        n_samples = 1000;
    } else {
        n_samples = opt.find("n_samples").asInt();
    }
    
    double sampleTime;
    if(!opt.check("sampleTime")) {
        sampleTime = 0.01;
    } else {
        sampleTime = opt.find("sampleTime").asDouble();
    }
    
        
    double cut_freq;
    if(!opt.check("cut_freq")) {
        cut_freq = 3;
    } else {
        cut_freq = opt.find("cut_freq").asDouble();
    }
    // for computing lambda has to use gurls
    double lambda;
    if(!opt.check("lambda")) {
        lambda = 1;
    } else {
        lambda = opt.find("lambda").asDouble();
    }
    
    int skip;
    if(!opt.check("skip")) {
        skip = 1;
    } else {
        skip = opt.find("skip").asDouble();
    }
    
    bool verbose;
    if(!opt.check("verbose")) {
        verbose = false;
    } else {
        verbose = true;
    }
    
