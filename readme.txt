The objective is to estimate the inertial parameters of the iCub limbs.

Dependendies:

- GURLS: https://github.com/LCSL/GURLS. More detail information can be found on GURLS manual.

- iCub & Yarp installation can be found on http://wiki.icub.org/wiki/Linux:Installation_from_sources

- kdl_codyco package installation https://github.com/traversaro/kdl_codyco

- orocos_kdl package https://github.com/orocos/orocos_kinematics_dynamics/tree/master/orocos_kdl

- iDynTree package installation https://github.com/robotology-playground/iDynTree

How to build: 

Step 1 : Install all dependencies.

Step 2 : make build directory 
	#### mkdir build
	#### cd build
	
Step 4 : Generate make file
	#### ccmake ../src
	
Step 5 : Make
	#### make
	
Step 6 : Run
	#### ./iCubParis02_simple_analysis --dataset ../datasets/part1-left.csv --results results.csv --parameters params.csv --lambda 1.0 --skip 100 --verbose
	
Step 7 (Optional): Analyse results using the MATLAB script provided in "scripts"
