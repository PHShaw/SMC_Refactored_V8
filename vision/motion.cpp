/**
 * Based on the distort example from the iCub utilities
 *
 * Looks for the difference between one image and the next from the same eye.
 *
 * phs 11th March 2011
 */

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/sig/all.h>
#include <iostream>
#include <math.h>
#include <string.h>
#include <stdlib.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;

void findTarget(ImageOf<PixelRgb>* newImage, ImageOf<PixelRgb>* imgout);

//need min of 9 if using with imageSteady, or 20 if using direct
int errorMargin = 20;

void quit()
{

	printf("Quit.\n\n");
}
BufferedPort < Bottle > porttargets;

ImageOf<PixelRgb> *lastImage;

int main(int argc, char* argv[])
{
	Network yarp;
	BufferedPort < ImageOf<PixelRgb> > portin;
	BufferedPort < ImageOf<PixelRgb> > portout;

	ImageOf < PixelRgb > *newImage;

	atexit(quit);

	portin.open("/motion/in");
	portout.open("/motion/out");
	porttargets.open("/target/data");

	yarp.connect("/icub/cam/right", "/motion/in");
	yarp.connect("/motion/out", "/yarpview/motion");


	lastImage = portin.read(1);

	while (1)
	{
		if (newImage = portin.read(1))
		{
			ImageOf < PixelRgb > &imgout = portout.prepare();
			imgout.resize(*newImage);

			findTarget(newImage, &imgout);
			portout.write();
			lastImage = newImage;
		}
		else
		{
			printf("no newImage\n");
			Time::delay(1);
		}
		Time::delay(0.1);
	}
}

void findTarget(ImageOf<PixelRgb>* newImage, ImageOf<PixelRgb>* imgout)
{
	int xin, yin, xout, yout;
	float normxin, normyin, normxout, normyout;


	bool motion = false;
	unsigned long motionX = 0, motionY = 0, counter=0;

	for (int x = 0; x < newImage->width(); x++)
	{
		for (int y = 0; y < newImage->height(); y++)
		{
			//Salt and pepper filter
			if(y%2 == 0)
			{
				if(x%2!=0)
				{
					imgout->pixel(x, y).r = newImage->pixel(x,y).r;
					imgout->pixel(x, y).g = newImage->pixel(x,y).g;
					imgout->pixel(x, y).b = newImage->pixel(x,y).b;
					continue;
				}
			}
			else
			{
				if(x%2 == 0)
				{
					imgout->pixel(x, y).r = newImage->pixel(x,y).r;
					imgout->pixel(x, y).g = newImage->pixel(x,y).g;
					imgout->pixel(x, y).b = newImage->pixel(x,y).b;
					continue;
				}
			}


			if((newImage->pixel(x,y).r <= lastImage->pixel(x,y).r+errorMargin && newImage->pixel(x,y).r >= lastImage->pixel(x,y).r-errorMargin) &&
				(newImage->pixel(x,y).g <= lastImage->pixel(x,y).g+errorMargin && newImage->pixel(x,y).g >= lastImage->pixel(x,y).g-errorMargin) &&
				(newImage->pixel(x,y).b <= lastImage->pixel(x,y).b+errorMargin && newImage->pixel(x,y).b >= lastImage->pixel(x,y).b-errorMargin))
			{
				imgout->pixel(x, y).r = newImage->pixel(x,y).r;
				imgout->pixel(x, y).g = newImage->pixel(x,y).g;
				imgout->pixel(x, y).b = newImage->pixel(x,y).b;
			}
			else
			{
				imgout->pixel(x, y).r = 255;
				imgout->pixel(x, y).g = 255;
				imgout->pixel(x, y).b = 255;
				motion = true;
				motionX += x;
				motionY += y;
				counter ++;
			}

		}
	}


	if(motion)
	{
		double avgX = (double) motionX / counter;
		double avgY = (double) motionY / counter;

		//add cross head into the output image, locating target
		for (int x = ((avgX - 10) < 0 ? 0 : avgX - 10); x < ((avgX + 10)
				>= imgout->width() ? imgout->width() : avgX + 10); x++)
		{
			imgout->pixel(x, avgY).r = 255;
			imgout->pixel(x, avgY).g = 0;
			imgout->pixel(x, avgY).b = 255;
		}
		for (int y = ((avgY - 10) < 0 ? 0 : avgY - 10); y < ((avgY + 10)
				>= imgout->height() ? imgout->height() : avgY + 10); y++)
		{
			imgout->pixel(avgX, y).r = 255;
			imgout->pixel(avgX, y).g = 0;
			imgout->pixel(avgX, y).b = 255;
		}

		Bottle& target = porttargets.prepare();
				target.clear();
				target.addString("motion");
				target.addDouble(avgX);
				target.addDouble(avgY);
				porttargets.write();

	}


}

