#include <iostream>
#include "gurls++/gurls.h"
#include "gurls++/exceptions.h"
#include "gurls++/gmat2d.h"
#include "gurls++/options.h"
#include "gurls++/optlist.h"
#include "gurls++/norm.h"
#include "gurls++/normzscore.h"
#include "gurls++/normtestzscore.h"
#include "gurls++/norml2.h"
#include "gurls++/gmath.h"
#include <kdl_codyco/utils.hpp>
#include <kdl/rigidbodyinertia.hpp>
#include <kdl/frames_io.hpp>
#include <kdl_codyco/regressors/dataset/DynamicDatasetFile.hpp>
#include <kdl_codyco/regressors/dynamicRegressorGenerator.hpp>
#include <cstdlib>
#include <iCub/iDynTree/iCubTree.h>
#include <iCub/ctrl/filters.h>
#include <iCub/ctrl/adaptWinPolyEstimator.h>

#include <kdl_codyco/regressor_loops.hpp>
#include <kdl_codyco/regressor_utils.hpp>

#include <iostream>
#include <fstream>

#include <yarp/os/Property.h>

#include "multitaskRecursiveLinearEstimator.h"
#include "multitaskSVDLinearEstimator.h"



using namespace KDL;
using namespace KDL::CoDyCo;
using namespace KDL::CoDyCo::Regressors;
using namespace gurls;
using namespace std;

typedef double T; ///< Data type of the matrices elements

/**
  * Main function
  */
int main(int argc, char *argv[])
{
  yarp::os::Property opt;
    
  opt.fromCommand(argc,argv);
    string xtr_file, xte_file, ytr_file, yte_file;

    // check that all inputs are given
    if(argc<4)
    {
        std::cout << "========================================================================================"<< std::endl;
        std::cout << " Wrong parameters number ("<<argc <<")." << std::endl;
        std::cout << " Provide a valid path for training, test and output files using the following syntax:" << std::endl;
        std::cout << " \n\n\t " << argv[0] << " xtr xte ytr yte" << std::endl;
	//std::cout << "iCubParis02_data_analysis_simple: " << std::endl;
        //std::cout << "usage: ./iCubParis02_data_analysis --dataset dataset.csv --results results.csv" << std::endl;
        std::cout << "additional options: --n_samples n the number of random regressors to generate for the numerical identifable subspace computation" << std::endl;
        std::cout << "additional options: --cut_freq the cut frequency to use for filtering torques and ft measures (in Hz, default 3 Hz)" << std::endl;
        std::cout << "additional options: --sampleTime the sample time of the data (in s, default: 0.01 s)" << std::endl;
//        std::cout << "additional options: --lambda regularization parameter" << std::endl;
        std::cout << "additional options: --skip number of initial samples to skip" << std::endl;
        std::cout << "additional options: --verbose print debug information" << std::endl;
        std::cout << "additional options: --parameters param.csv output a file of trajectoreis of the estimated parameters" << std::endl;
	std::cout << "========================================================================================" << std::endl << std::endl;
	
        return 0;
    }
    
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
    /*
    // for computing lambda has to use gurls
    double lambda;
    if(!opt.check("lambda")) {
        lambda = 1;
    } else {
        lambda = opt.find("lambda").asDouble();
    }
    */
    
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
    

    // get file names from input
    xtr_file = argv[1];
    xte_file = argv[2];
    ytr_file = argv[3];
    yte_file = argv[4];

    try
    {
        gMat2D<double> Xtr, Xte, ytr, yte;

        // load data from file
        Xtr.readCSV(xtr_file);
        Xte.readCSV(xte_file);
        ytr.readCSV(ytr_file);
        yte.readCSV(yte_file);
	
// 	gMat2D<long unsigned int> X_normtr;
	OptTaskSequence *seq = new OptTaskSequence();
	*seq << "paramsel:fixlambda";
	GurlsOptionsList *process = new GurlsOptionsList("processes", true);
	OptProcess* process0 = new OptProcess();
	*process0 << GURLS::compute;
	process->addOpt("zero", process0);
	GurlsOptionsList* opt = new GurlsOptionsList("Gurls_PM",true);
	opt->addOpt("seq",seq);
	opt->addOpt("processes", process);
	
	GURLS G;

        string jobId0("zero");
        //string jobId1("two");

        // run gurls for training
        //G.run(Xtr, ytr, *opt2, jobId0);

        // run gurls for testing
        G.run(Xte, yte, *opt, jobId0);

    }
    catch (gException& e)
    {
        cout << e.getMessage() << endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;

}
	