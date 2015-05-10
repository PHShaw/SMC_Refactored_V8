/**
 *
 * phs 19th November 2010
 * phs 1st May 2015 revisiting, as a way to visualise maps as they are learnt.
 * 				This is no longer going to try and overlay the fields on the camera image,
 * 				but instead show the input and output maps with the connections between them.
 * 				Connections will be illustrated by colour.
 * 				This should ideally be implemented using an observer design pattern.
 * 				Need to get the map dimensions at the start, this could be given as parameters
 *
 * 				Ports:
 * 					/yarpview/map:i			display of map
 * 					/map/in					port to which connections can be posted input<x,y> output<x,y>
 *
 *
 */

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/sig/all.h>
#include <iostream>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <vector>

#include "FieldFieldMapping.h"
#include "FFM_IO.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;

typedef FieldFieldMapping ffm;

ffm* ppm;

void drawfield(int x0, int y0, int radius,  ImageOf<PixelRgb>* imgout, int r, int g, int b, bool inputMap=true);
void drawBackground(ImageOf<PixelRgb>* imgout, bool clear=false);
bool rangeCheck(int x, int y);
//int width,height;

BufferedPort < ImageOf<PixelRgb> > portout;
BufferedPort <Bottle> portin;
//Bottle* link;

int ixmin,ixmax,iymin,iymax,oxmin,oxmax,oymin,oymax;	//Dimensions of the input and output maps
int iWidth, iHeight, oWidth, oHeight;
int spacer = 10;
int imageWidth, imageHeight;

void quit()
{
	portin.close();
	portout.close();
}



int main(int argc, char* argv[])
{
	Property options;
	if(argc>=2)
		options.fromCommand(argc,argv);
	else
	{
		cout << "usage:" <<
					"\n\t --name name" <<
					"\n\t --inputXMin xmin" <<
					"\n\t --inputXMax xmax" <<
					"\n\t --inputYMin ymin" <<
					"\n\t --inputYMax ymax" <<
					"\n\t --outputXMin xmin" <<
					"\n\t --outputXMax xmax" <<
					"\n\t --outputYMin ymin" <<
					"\n\t --outputYMax ymax";
		quit();
	}

	string mapname;
	Value* val;
	if(options.check("name",val))
		mapname = val->asString().c_str();
	else
		mapname = "eyes";

	Network yarp;

	string portname = "/map/";
	portname += mapname;
	portname += "/";

	string ending = portname;
	ending += "in";

	portin.open(ending.c_str());

	ending = portname;
	ending += "out";
	portout.open(ending.c_str());

	atexit(quit);

	yarp.connect("/map/eyes/out", "/yarpview/map:i");

//TODO: have option to load an existing mapping or listen to port in.
	//Load map
	string path ="../../data/";
	cout << "Enter the filename to load the links from e.g. eye_BB.xml:" <<endl;
	string filename;
	cin >> filename;
	string fullpath = path + filename;
	FFM_IO io;
	ppm = io.loadMappingFromXML(fullpath.c_str());

	ixmin = (int)(ppm->getInputMinX());
	ixmax = (int)(ppm->getInputMaxX());
	iymin = (int)(ppm->getInputMinY());
	iymax = (int)(ppm->getInputMaxY());

	oxmin = (int)(ppm->getOutputMinX());
	oxmax = (int)(ppm->getOutputMaxX());
	oymin = (int)(ppm->getOutputMinY());
	oymax = (int)(ppm->getOutputMaxY());

	iWidth = ixmax - ixmin;
	iHeight = iymax - iymin;
	oWidth = oxmax - oxmin;
	oHeight = oymax - oymin;

	imageWidth = iWidth + spacer + oWidth;
	if(iHeight>oHeight)
		imageHeight = iHeight;
	else
		imageHeight = oHeight;


	size_t numFields;
	vector<FieldLink*> linkedFields = ppm->getLinkedFields();
	numFields = linkedFields.size();

//	ImageOf < PixelRgb > &imgout = portout.prepare();
//	imgout.resize(imageWidth, imageHeight);

	size_t counter;
	size_t i=0;
	cout << "There are " << numFields << " fields" << endl;
	while(i < numFields)
	{
		ImageOf < PixelRgb > &imgout = portout.prepare();
		imgout.resize(imageWidth, imageHeight);
		drawBackground(&imgout);

		PolarField* inputField = (PolarField*)(linkedFields.at(i)->input);
		PolarField* outputField = (PolarField*)(linkedFields.at(i)->output);

		int x,y,radius;
		x = (int)inputField->getXcoord();
		y = (int)inputField->getYcoord();
		radius = (int)inputField->getRadius();
		printf("writing field %i: [%i, %i, %i]\n",i, x, y, radius);

		int r,g,b;
		//Colour calculations from Matlab code:
		//rValue=(EyeDataOut(i,1)-EyeAxisMin(1))/EyeRange(1);	//based on x axis of output field
        //gValue=(EyeDataOut(i,2)-EyeAxisMin(2))/EyeRange(2);	//based on y axis of output field
        //bValue=1-rValue;

		//255 -> white
		//0 -> black
		r = (int)(((outputField->getXcoord()-oxmin)/oWidth) * 255.0);
		g = (int)(((outputField->getYcoord()-oymin)/oHeight) * 255.0);
		b = 255-r;
		printf("r,g,b: [%i,%i,%i]\n", r, g, b);



		drawfield(x, y, radius,  &imgout, r,g,b,true);

		x = (int)outputField->getXcoord();
		y = (int)outputField->getYcoord();
		radius = (int)outputField->getRadius();
		drawfield(x, y, radius,  &imgout, r,g,b,false);

		portout.write();
		Time::delay(0.05);
		i++;



	}


	portin.close();
	portout.close();



}

/*
 * Algorithm from Wikipedia: Midpoint_circle_algorithm
 * 20th March 2012
 */
void drawfield(int x0, int y0, int radius,  ImageOf<PixelRgb>* imgout, int r, int g, int b, bool inputMap)
{
//TODO: If scales go negative, on input map or output map, and if origin not at mid point,
//	then the points will not be plotted correctly.

	if(!inputMap)
	{
		x0 += (iWidth + spacer + oWidth/2);
		y0 += oHeight/2;
	}

	int f = 1-radius;
	int ddF_x = 1;
	int ddF_y = -2 * radius;
	int x = 0;
	int y = radius;

	int cx, cy;


	if(rangeCheck(x0,y0+radius))
	{
		imgout->pixel(x0,y0+radius).r = r;
		imgout->pixel(x0,y0+radius).g = g;
		imgout->pixel(x0,y0+radius).b = b;
	}
	if(rangeCheck(x0,y0-radius))
	{
		imgout->pixel(x0,y0-radius).r = r;
		imgout->pixel(x0,y0-radius).g = g;
		imgout->pixel(x0,y0-radius).b = b;
	}
	if(rangeCheck(x0+radius,y0))
	{
		imgout->pixel(x0+radius,y0).r = r;
		imgout->pixel(x0+radius,y0).g = g;
		imgout->pixel(x0+radius,y0).b = b;
	}
	if(rangeCheck(x0-radius,y0))
	{
		imgout->pixel(x0-radius,y0).r = r;
		imgout->pixel(x0-radius,y0).g = g;
		imgout->pixel(x0-radius,y0).b = b;
	}


	while(x<y)
	{
		// ddF_x == 2*x +1;
		// ddF_y ==-2*y;
		// f==x*x + y*y - radius*radius + 2*x - y +1

		if(f>=0)
		{
			y--;
			ddF_y += 2;
			f += ddF_y;
		}
		x++;
		ddF_x += 2;
		f += ddF_x;
		if(rangeCheck(x0+x,y0+y))
		{
			imgout->pixel(x0+x,y0+y).r = r;
			imgout->pixel(x0+x,y0+y).g = g;
			imgout->pixel(x0+x,y0+y).b = b;
		}
		if(rangeCheck(x0-x,y0+y))
		{
			imgout->pixel(x0-x,y0+y).r = r;
			imgout->pixel(x0-x,y0+y).g = g;
			imgout->pixel(x0-x,y0+y).b = b;
		}
		if(rangeCheck(x0+x,y0-y))
		{
			imgout->pixel(x0+x,y0-y).r = r;
			imgout->pixel(x0+x,y0-y).g = g;
			imgout->pixel(x0+x,y0-y).b = b;
		}
		if(rangeCheck(x0-x,y0-y))
		{
			imgout->pixel(x0-x,y0-y).r = r;
			imgout->pixel(x0-x,y0-y).g = g;
			imgout->pixel(x0-x,y0-y).b = b;
		}
		if(rangeCheck(x0+y,y0+x))
		{
			imgout->pixel(x0+y,y0+x).r = r;
			imgout->pixel(x0+y,y0+x).g = g;
			imgout->pixel(x0+y,y0+x).b = b;
		}
		if(rangeCheck(x0-y,y0+x))
		{
			imgout->pixel(x0-y,y0+x).r = r;
			imgout->pixel(x0-y,y0+x).g = g;
			imgout->pixel(x0-y,y0+x).b = b;
		}
		if(rangeCheck(x0+y,y0-x))
		{
			imgout->pixel(x0+y,y0-x).r = r;
			imgout->pixel(x0+y,y0-x).g = g;
			imgout->pixel(x0+y,y0-x).b = b;
		}
		if(rangeCheck(x0-y,y0-x))
		{
			imgout->pixel(x0-y,y0-x).r = r;
			imgout->pixel(x0-y,y0-x).g = g;
			imgout->pixel(x0-y,y0-x).b = b;
		}
	}
}


bool rangeCheck(int x, int y)
{
	if(x<0 || x>=imageWidth || y<0 || y>=imageHeight)
		return false;
	else
		return true;
}


void drawBackground(ImageOf<PixelRgb>* imgout, bool clear)
{
	if(clear){
		//Set background to white
		for(int j=0; j<imageWidth; j++)
		{
			for(int k=0; k<imageHeight; k++)
			{
				imgout->pixel(j, k).r = 0;
				imgout->pixel(j, k).g = 0;
				imgout->pixel(j, k).b = 0;
			}
		}
	}

	//Insert axis
	//input map y axis
	for(int i=0; i<iHeight; i++)
	{
		imgout->pixel(iWidth/2,i).r = 255;
		imgout->pixel(iWidth/2,i).g = 255;
		imgout->pixel(iWidth/2,i).b = 255;
	}

	//input map x axis
	for(int i=0; i<iWidth; i++)
	{
		imgout->pixel(i,iHeight/2).r = 255;
		imgout->pixel(i,iHeight/2).g = 255;
		imgout->pixel(i,iHeight/2).b = 255;
	}

	//output map y axis
	for(int i=0; i<oHeight;i++)
	{
		imgout->pixel(iWidth+spacer+oWidth/2,i).r = 255;
		imgout->pixel(iWidth+spacer+oWidth/2,i).g = 255;
		imgout->pixel(iWidth+spacer+oWidth/2,i).b = 255;
	}

	//output map x axis
	for(int i=iWidth+spacer; i<oWidth+iWidth+spacer; i++)
	{
		imgout->pixel(i,oHeight/2).r = 255;
		imgout->pixel(i,oHeight/2).g = 255;
		imgout->pixel(i,oHeight/2).b = 255;
	}
}
