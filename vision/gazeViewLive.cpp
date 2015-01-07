/*
 * gazeViewLive.cpp
 *
 * TODO: could reprint the gaze fields each time, gradually fading away through time...
 * 			Also, could introduce excitation, to show which is most exciting... (could get quite flickery)
 *  Created on: 9 Aug 2012
 *      Author: icub
 *
 * 	Note: Before running this, run:
 *	  yarpview /yarpview/gaze:i
 */

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/sig/all.h>

#include <stdlib.h>
#include <string>
#include <vector>


using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;

struct gazePoint{
	gazePoint(int px, int py, int pradius, string pcol)
	{
		x = px; y = py; radius = pradius;
		colour = pcol;
	}
	int x,y,radius;
	string colour;

};

void drawBackground(ImageOf<PixelRgb>* imgout);
bool drawfield(int x0, int y0, int radius,  ImageOf<PixelRgb>* imgout);
bool rangeCheck(int x, int y);
void manageHistory(ImageOf<PixelRgb>* imgout);
void setColour(string colour);
int width,height;
int markerx, markery;
int r,g,b;

vector<gazePoint*> history;
int historySize = 10;

int main(int argc, char* argv[])
{
	width = 160;
	height = 200;
	markerx = markery = 20;

	Network yarp;
	BufferedPort < ImageOf<PixelRgb> > portout;
	BufferedPort<yarp::os::Bottle> gazin;

	portout.open("/gaze/out");
	yarp.connect("/gaze/out", "/yarpview/gaze:i");

	gazin.open("/gaze/live/in");

	{
	ImageOf < PixelRgb > &imgout = portout.prepare();
	imgout.resize(width,height);

	drawBackground(&imgout);

	portout.write();
	Time::delay(0.1);

	}

	Bottle* input;
	while(true)
	{
		if(input = gazin.read(1))
		{
			double gazeX, gazeY, radius;
			gazeX = input->get(0).asDouble();
			gazeY = input->get(1).asDouble();
			radius = input->get(2).asDouble();
			string col = input->get(3).asString().c_str();

			setColour(col);

			ImageOf < PixelRgb > &imgout = portout.prepare();
			imgout.resize(width,height);
			int x,y;
			x = (int) gazeX + width/2;
			y = (int) gazeY + height/2;
			x = width-x;
	//		y = height-y;

			history.push_back(new gazePoint(x,y,radius,col));

	//		if(x<0 || y<0)
	//			printf("%i %i\n", x,y);

			imgout.pixel(x,y).r = r;
			imgout.pixel(x,y).g = g;
			imgout.pixel(x,y).b = b;

			drawfield(x, y, radius,  &imgout);

			manageHistory(&imgout);

			portout.write();
		}
		Time::delay(0.1);
	}

	yarp.disconnect("/gaze/out", "/yarpview/gaze:i");
	portout.close();
}


void drawBackground(ImageOf<PixelRgb>* imgout)
{
	//Set background to white
	for(int j=0; j<width; j++)
	{
		for(int k=0; k<height; k++)
		{
			imgout->pixel(j, k).r = 255;
			imgout->pixel(j, k).g = 255;
			imgout->pixel(j, k).b = 255;
		}
	}

	//Insert axis
	for(int i=0; i<height; i++)
	{
		if(i%markerx==0)
		{
			imgout->pixel(width/2-3,i).r = 0;
			imgout->pixel(width/2-3,i).g = 0;
			imgout->pixel(width/2-3,i).b = 0;

			imgout->pixel(width/2-2,i).r = 0;
			imgout->pixel(width/2-2,i).g = 0;
			imgout->pixel(width/2-2,i).b = 0;

			imgout->pixel(width/2-1,i).r = 0;
			imgout->pixel(width/2-1,i).g = 0;
			imgout->pixel(width/2-1,i).b = 0;

			//------------

			imgout->pixel(width/2+1,i).r = 0;
			imgout->pixel(width/2+1,i).g = 0;
			imgout->pixel(width/2+1,i).b = 0;

			imgout->pixel(width/2+2,i).r = 0;
			imgout->pixel(width/2+2,i).g = 0;
			imgout->pixel(width/2+2,i).b = 0;

			imgout->pixel(width/2+3,i).r = 0;
			imgout->pixel(width/2+3,i).g = 0;
			imgout->pixel(width/2+3,i).b = 0;
		}

		imgout->pixel(width/2,i).r = 0;
		imgout->pixel(width/2,i).g = 0;
		imgout->pixel(width/2,i).b = 0;
	}
	for(int i=0; i<width; i++)
	{
		if(i%markery==0)
		{
			imgout->pixel(i,height/2-3).r = 0;
			imgout->pixel(i,height/2-3).g = 0;
			imgout->pixel(i,height/2-3).b = 0;

			imgout->pixel(i,height/2-2).r = 0;
			imgout->pixel(i,height/2-2).g = 0;
			imgout->pixel(i,height/2-2).b = 0;

			imgout->pixel(i,height/2-1).r = 0;
			imgout->pixel(i,height/2-1).g = 0;
			imgout->pixel(i,height/2-1).b = 0;

			//------------

			imgout->pixel(i,height/2+1).r = 0;
			imgout->pixel(i,height/2+1).g = 0;
			imgout->pixel(i,height/2+1).b = 0;

			imgout->pixel(i,height/2+2).r = 0;
			imgout->pixel(i,height/2+2).g = 0;
			imgout->pixel(i,height/2+2).b = 0;

			imgout->pixel(i,height/2+3).r = 0;
			imgout->pixel(i,height/2+3).g = 0;
			imgout->pixel(i,height/2+3).b = 0;
		}

		imgout->pixel(i,height/2).r = 0;
		imgout->pixel(i,height/2).g = 0;
		imgout->pixel(i,height/2).b = 0;
	}
}


/*
 * Algorithm from Wikipedia: Midpoint_circle_algorithm
 * 20th March 2012
 */
bool drawfield(int x0, int y0, int radius,  ImageOf<PixelRgb>* imgout)
{

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
	if(x<0 || x>=width || y<0 || y>=height)
		return false;
	else
		return true;
}

void manageHistory(ImageOf<PixelRgb>* imgout)
{
//	printf("History size: %i: ", history.size());

	if(history.size()>historySize)
	{
		gazePoint* gp = history.at(0);
		printf("Removing gaze point: %i, %i, %s\n", gp->x, gp->y, gp->colour.c_str());
		setColour("white");
		drawfield(gp->x, gp->y, gp->radius, imgout);
		history.erase(history.begin());

	}
}


void setColour(string colour)
{
	if(colour.compare("red")==0)
	{
		r = 255;
		g = b = 0;
	}
	else if(colour.compare("green")==0)
	{
		r = b = 0;
		g = 255;
	}
	else if(colour.compare("blue")==0)
	{
		r = g = 0;
		b = 255;
	}
	else if(colour.compare("yellow")==0)
	{
		r = g = 255;
		b = 0;
	}
	else if(colour.compare("white")==0)
	{
		r = g = b = 255;
	}
	else if(colour.compare("black")==0)
	{
		r = g = b = 0;
	}
	else		//gray
	{
		r = g = b = 200;
	}
}
