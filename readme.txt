The objective is to estimate the inertial parameters of the iCub limbs.

Dependendies:

- GURLS: https://github.com/LCSL/GURLS. More detail information can be found on GURLS manual.
- iCub & Yarp installation can be found on http://wiki.icub.org/wiki/Linux:Installation_from_sources
- kdl_codyco package installation https://github.com/traversaro/kdl_codyco
- orocos_kdl package https://github.com/orocos/orocos_kinematics_dynamics/tree/master/orocos_kdl
- iDynTree package installation https://github.com/robotology-playground/iDynTree

How to build: 

Step 1 : Install all necessary packages & respective dependencies.

Step 2 : make build directory 
	#### mkdir build ####
	#### cd build ####
Step 4 : Cmake
	#### ccmake ../src ####
Step 5 : then make it	
	#### make ####
Step 6 : Run the code you want according to your choise for estimating parameters. 

	#### ./iCubParis02_simple_analysis --dataset ../datasets/part1-left.csv --results results.csv --parameters params.csv --lambda 1.0 --skip 100 --verbose

