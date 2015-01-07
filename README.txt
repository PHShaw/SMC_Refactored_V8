This folder contains the code for building maps, basic vision processing, and learning mappings.

To compile all the software you will require:
YARP
BOOST (www.boost.org/users/download/)
Xerces (http://xerces.apache.org/xerces-c/)

Folders
=======
data: Stores learnt maps and log files.  This is initially empty.

kinematics: Provides some functionality for calculating some kinematic changes

maps: Provides functionality for representing maps, saving and loading maps in xml format, extracting data from the maps.  Not all classes listed are fully implemented, and some methods are known to contain errors.  For examples of how to use the maps, please see please see the source code in smc for eyeSacadding.cpp.  A fieldExtract program is provided for extracting the coordinates of learnt fields in the various maps.  These can be plotted in Excel or a separate Matlab script, not provided.

smc: Provides the main body of the software "Sensori-Motor Control".  This contains the main methods, and the learning algorithms.  Some of the functions make use of matlab scripts, which are not included here.

vision: Basic vision processing functionality to identify targets.



To compile
==========
Cmake scripts are provided for compiling the various modules.  Cmake modules are provided for locating boost and xerces libraries.

The kinematics and maps modules should be made before making the smc modules.  These compile into libraries that are used by the smc.  The vision is an independant module, so could be run on a separate machine if desired.

Compilation instructions for each module are:
In module directory:

mkdir build
cd build
cmake ..
make

For vision, you can also run "make install" to put the binaries in $ICUB_ROOT/bin in order to use the yarp application manager.


To run
======
First run the vision module, using the yarp application manager.  Then run learningMenu in /smc/build.  Additional instructions are provided in /smc.

