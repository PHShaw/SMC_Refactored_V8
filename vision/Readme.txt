To compile:

mkdir build
cd build
cmake ..
make
sudo make install


To run, use yarp manager
For cameras from real robot:
$MANAGER colourVision.xml

For cameras from simulator
$MANAGER colourSim.xml

Please note, the colour processing for the real robot has been tuned for the cameras and lighting of the Aberystwyth iCub.


Usage:
The code identifies pixels matching particular colour ranges.  These pixels are highlighted on the output.  For each colour, the center is calculated based on the pixel coordinates.  If pixels from multiple parts of the image are identified, then this can throw off the centering for the target.  The source file vision_multi.cpp does allow for multiple targets of the same colour, however it is not possible to then identify if the coloured blob is the same blob you were looking at previously.

A white rectangle with a shaded circular fovea highlights the foveal region of the image.

Coordinates that are identified are sent to the port /target/[right|left]/data
The format of the data is [colour] xCoord yCoord noPix
Colours identified from the real cameras: [ red | yellow | green | blue | white | gray ] for the real cameras
Colours identified from the simulator: [ red | yellow | green | blue | cyan | magenta ]

noPix is the number of pixels identified of that colour and can be used to give an indication of the size of an object.

