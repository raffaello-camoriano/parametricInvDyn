The objective is to estimate the ineetial parameters like mass, center of mass & inertia matrx using icub left hand dataset. 

Package includes c++ code for parametric modeling using GURLS package. However first, Install GURLS package with necessary dependencies from https://github.com/LCSL/GURLS. More detail information can be found on GURLS manual. In addition to GURLS you have to install other packages like ICUB, Yarp, kdl_codyco, orocos_kdl, iDynTree. 

iCub & Yarp installation can be found on http://wiki.icub.org/wiki/Linux:Installation_from_sources

kdl_codyco package installation https://github.com/traversaro/kdl_codyco

orocos_kdl package https://github.com/orocos/orocos_kinematics_dynamics/tree/master/orocos_kdl

iDynTree package installation https://github.com/robotology-playground/iDynTree

How to Install Parametric Modeling Package : 

Step 1 : Install all necessary packages & respective dependencies.

Step 2 : root to parametric modeling package version 0.1
	#### cd Parametric_Modeling/Version0.1 ####
Step 3 : make build directory 
	#### mkdir build ####
	#### cd build ####
Step 4 : Cmake
	#### ccmake../ ####
Step 5 : then make it	
	#### make ####
Step 6 : Run the code you want according to your choise for estimating parameters. 

	#### ./iCubParis02_simple_analysis --dataset ../Data_Sets/part1-left.csv --results results.csv #### 

Note : These tests were made on iCub left hand. some tests were made when new package were included for getting regularization parameter. There are certain options to change approach for getting $lambda$. Please go through the code for more details.



This package is simple extension of exisiting work showing the influence of ragularization parameter in estimating inertial parameters. 

Detail explaination of parametric modeling and theory behind this can be found from my thesis which is included in the package. 

Any problems with code or questions can be asked through email prabhukumar.chalikonda@gmail.com

