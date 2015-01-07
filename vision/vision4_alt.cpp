/**
 *	Vision.cpp
 *	This program identifies particular coloured pixels in an image and generates target locations
 *	based on the averages of the pixel locations, one target per colour.
 *
 *	Specifically designed for use in the simulator only!
 *
 * Latest update: phs 26th May 2010
 *
 * Version history:
 * 	v1: very basic vision processing used for initial calibration to identify a wide range
 * 		of red objects, without too much background noise.  Also some blue and green, but weak
 *  v2: added visual circular fovea in the centre of the image
 *  v3: modified approach for producing output image, also added added yellow colour detection
 *  	and enhanced the identification of green objects
 *  v4: general tidy up of the code and colour calibration to buttons on board
 *
 *
 */

#include <yarp/os/all.h>	//bottle, bufferedPort
#include <yarp/sig/all.h>	//imageOf, PixelPgb
#include <algorithm>
#include <iostream>
#include <math.h>
#include <string.h>
#include <stdlib.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;

void findTarget(ImageOf<PixelRgb>* imgin, ImageOf<PixelRgb>* imgout);
bool fovea(int x, int y, double* dist);


void quit()
{
	printf("Quit.\n\n");
}
BufferedPort < Bottle > porttargets;
int main(int argc, char* argv[])
{
	Network yarp;
	BufferedPort < ImageOf<PixelRgb> > portin;
	BufferedPort < ImageOf<PixelRgb> > portout;

	ImageOf < PixelRgb > *imgin;

	atexit(quit);

	Property options;
	if(argc>=2)
		options.fromCommand(argc,argv);

	string cam;
	Value* val;
	if(options.check("cam",val))
	{
		cam = val->asString().c_str();
	}
	else
	{
		cam = "right";
	}

	string portname = "/targetSim/";
	portname += cam;
	portname += "/";

	string ending = portname;
	ending += "in";

	portin.open(ending.c_str());

	ending = portname;
	ending += "out";

	portout.open(ending.c_str());

	ending = portname;
	ending += "data";
	porttargets.open(ending.c_str());


	while (1)
	{
		if (imgin = portin.read(1))
		{
			ImageOf < PixelRgb > &imgout = portout.prepare();
			imgout.resize(*imgin);
			findTarget(imgin, &imgout);
			portout.write();
		}
		else
		{
			printf("no imgin\n");
			Time::delay(1);
		}
		Time::delay(0.1);
	}
}

void findTarget(ImageOf<PixelRgb>* imgin, ImageOf<PixelRgb>* imgout)
{
	int xout, yout;


	//	saliency and coordinate counts/sums
	int green = 0, blue = 0, red = 0, yellow = 0, white=0, cyan = 0, magenta=0;
	unsigned long redx = 0, redy = 0, greenx = 0, greeny = 0, cyanx=0, cyany=0, magentax=0, magentay=0,
			yellowx = 0, yellowy = 0, bluex = 0, bluey = 0, whitex = 0, whitey = 0;
	bool coloured = false;
	for (int x = 0; x < imgin->width(); x++)
	{
		for (int y = 0; y < imgin->height(); y++)
		{
			//Salt and pepper filter
//			if(y%2 == 0)
//			{
//				if(x%2!=0)
//				{
//					imgout->pixel(x,y).r = imgin->pixel(x,y).r;
//					imgout->pixel(x,y).g = imgin->pixel(x,y).g;
//					imgout->pixel(x,y).b = imgin->pixel(x,y).b;
//					continue;
//				}
//			}
//			else
//			{
//				if(x%2 == 0)
//				{
//					imgout->pixel(x,y).r = imgin->pixel(x,y).r;
//					imgout->pixel(x,y).g = imgin->pixel(x,y).g;
//					imgout->pixel(x,y).b = imgin->pixel(x,y).b;
//					continue;
//				}
//			}
			coloured = false;

			double r = imgin->pixel(x,y).r;
			double g = imgin->pixel(x,y).g;
			double b = imgin->pixel(x,y).b;


			// yellow
			if(r>=163 && g >=163 && b==0)
			{
				yellowx += x;
				yellowy += y;
				yellow ++;
				imgout->pixel(x,y).r = 128;
				imgout->pixel(x,y).g = 128;
				imgout->pixel(x,y).b = 0;
				coloured = true;
			}


//			// white - same colour as clouds
//			if(r >= 163 && g >= 163 && b >= 163)
//			{
//				whitex += x;
//				whitey += y;
//				white ++;
//				imgout->pixel(x,y).r = 128;
//				imgout->pixel(x,y).g = 128;
//				imgout->pixel(x,y).b = 128;
//				coloured = true;
//			}

			//green
			else if(r==0 && g>=163 && b==0)
			{
				greenx += x;
				greeny += y;
				green ++;
				imgout->pixel(x,y).r = 0;
				imgout->pixel(x,y).g = 128;
				imgout->pixel(x,y).b = 0;
				coloured = true;
			}

			//red
			else if (r>=163 && g==0 && b==0)
			{
				redx += x;
				redy += y;
				red++;
				imgout->pixel(x,y).r = 128;
				imgout->pixel(x,y).g = 0;
				imgout->pixel(x,y).b = 0;
				coloured = true;
			}

			//blue
			else if(r==0 && g==0 && b>=163)
			{
				bluex += x;
				bluey += y;
				blue++;
				imgout->pixel(x,y).r = 0;
				imgout->pixel(x,y).g = 0;
				imgout->pixel(x,y).b = 128;
				coloured = true;
			}

			//cyan
			else if(r==0 && g>=163 && b>=163)
			{
				cyanx += x;
				cyany += y;
				cyan++;
				imgout->pixel(x,y).r = 0;
				imgout->pixel(x,y).g = 128;
				imgout->pixel(x,y).b = 128;
				coloured = true;
			}

			//magenta
			else if(r>=163 && g==0 && b==163)
			{
				magentax += x;
				magentay += y;
				magenta++;
				imgout->pixel(x,y).r = 128;
				imgout->pixel(x,y).g = 0;
				imgout->pixel(x,y).b = 128;
				coloured = true;
			}

			if(!coloured)
			{
				imgout->pixel(x,y).r = imgin->pixel(x,y).r;
				imgout->pixel(x,y).g = imgin->pixel(x,y).g;
				imgout->pixel(x,y).b = imgin->pixel(x,y).b;
			}
		}
	}

	for (xout = 0; xout < imgout->width(); xout++)
	{
		for (yout = 0; yout < imgout->height(); yout++)
		{

			//shade fovea
			double dist = 0;
			if(fovea(xout, yout, &dist))
			{
				imgout->pixel(xout,yout).g = min((imgout->pixel(xout,yout).g+2*(int)dist),255);
				imgout->pixel(xout,yout).b = min((imgout->pixel(xout,yout).b+2*(int)dist),255);
			}
			else if(dist<=32)
			{
				imgout->pixel(xout,yout).b = min((imgout->pixel(xout,yout).b+(int)dist),255);
			}

		}
	}


	Bottle& target = porttargets.prepare();
	target.clear();

	if (red)
	{
		double avrx = (double) redx / red;
		double avry = (double) redy / red;

		//add cross head into the output image, locating target
		for (int x = ((avrx - 10) < 0 ? 0 : avrx - 10); x < ((avrx + 10)
				>= imgout->width() ? imgout->width() : avrx + 10); x++)
		{
			imgout->pixel(x, avry).r = 255;
			imgout->pixel(x, avry).g = 0;
			imgout->pixel(x, avry).b = 0;
		}
		for (int y = ((avry - 10) < 0 ? 0 : avry - 10); y < ((avry + 10)
				>= imgout->height() ? imgout->height() : avry + 10); y++)
		{
			imgout->pixel(avrx, y).r = 255;
			imgout->pixel(avrx, y).g = 0;
			imgout->pixel(avrx, y).b = 0;
		}

		printf("The red stimulus is visible at coordinates (%.3f,%.3f)\n",
				avrx, avry);


		target.addString("red");
		target.addDouble(avrx);
		target.addDouble(avry);
		target.addInt(red);

	}

	if (green)
	{
		float avrx = (float) greenx / green;
		float avry = (float) greeny / green;

		//add cross head into the output image, locating target
		for (int x = ((avrx - 10) < 0 ? 0 : avrx - 10); x < ((avrx + 10)
				>= imgout->width() ? imgout->width() : avrx + 10); x++)
		{
			imgout->pixel(x, avry).r = 0;
			imgout->pixel(x, avry).g = 255;
			imgout->pixel(x, avry).b = 0;
		}
		for (int y = ((avry - 10) < 0 ? 0 : avry - 10); y < ((avry + 10)
				>= imgout->height() ? imgout->height() : avry + 10); y++)
		{
			imgout->pixel(avrx, y).r = 0;
			imgout->pixel(avrx, y).g = 255;
			imgout->pixel(avrx, y).b = 0;
		}

		printf(
				"The green stimulus is visible at coordinates (%.3f,%.3f)\n",
				avrx, avry);

		target.addString("green");
		target.addDouble(avrx);
		target.addDouble(avry);
		target.addInt(green);
	}


	if (yellow)
	{
		float avrx = (float) yellowx / yellow;
		float avry = (float) yellowy / yellow;

		//add cross head into the output image, locating target
		for (int x = ((avrx - 10) < 0 ? 0 : avrx - 10); x < ((avrx + 10)
				>= imgout->width() ? imgout->width() : avrx + 10); x++)
		{
			imgout->pixel(x, avry).r = 255;
			imgout->pixel(x, avry).g = 255;
			imgout->pixel(x, avry).b = 0;
		}
		for (int y = ((avry - 10) < 0 ? 0 : avry - 10); y < ((avry + 10)
				>= imgout->height() ? imgout->height() : avry + 10); y++)
		{
			imgout->pixel(avrx, y).r = 255;
			imgout->pixel(avrx, y).g = 255;
			imgout->pixel(avrx, y).b = 0;
		}

		printf(
				"The yellow stimulus is visible at coordinates (%.3f,%.3f)\n",
				avrx, avry);

		target.addString("yellow");
		target.addDouble(avrx);
		target.addDouble(avry);
		target.addInt(yellow);
	}

	if (blue)
	{
		float avrx = (float) bluex / blue;
		float avry = (float) bluey / blue;

		//add cross head into the output image, locating target
		for (int x = ((avrx - 10) < 0 ? 0 : avrx - 10); x < ((avrx + 10)
				>= imgout->width() ? imgout->width() : avrx + 10); x++)
		{
			imgout->pixel(x, avry).r = 0;
			imgout->pixel(x, avry).g = 0;
			imgout->pixel(x, avry).b = 255;
		}
		for (int y = ((avry - 10) < 0 ? 0 : avry - 10); y < ((avry + 10)
				>= imgout->height() ? imgout->height() : avry + 10); y++)
		{
			imgout->pixel(avrx, y).r = 0;
			imgout->pixel(avrx, y).g = 0;
			imgout->pixel(avrx, y).b = 255;
		}

		printf(
				"The blue stimulus is visible at coordinates (%.3f,%.3f)\n",
				avrx, avry);
		target.addString("blue");
		target.addDouble(avrx);
		target.addDouble(avry);
		target.addInt(blue);
	}

	if (cyan)
	{
		float avrx = (float) cyanx / cyan;
		float avry = (float) cyany / cyan;

		//add cross head into the output image, locating target
		for (int x = ((avrx - 10) < 0 ? 0 : avrx - 10); x < ((avrx + 10)
				>= imgout->width() ? imgout->width() : avrx + 10); x++)
		{
			imgout->pixel(x, avry).r = 0;
			imgout->pixel(x, avry).g = 255;
			imgout->pixel(x, avry).b = 255;
		}
		for (int y = ((avry - 10) < 0 ? 0 : avry - 10); y < ((avry + 10)
				>= imgout->height() ? imgout->height() : avry + 10); y++)
		{
			imgout->pixel(avrx, y).r = 0;
			imgout->pixel(avrx, y).g = 255;
			imgout->pixel(avrx, y).b = 255;
		}

		printf(
				"The cyan stimulus is visible at coordinates (%.3f,%.3f)\n",
				avrx, avry);
		target.addString("cyan");
		target.addDouble(avrx);
		target.addDouble(avry);
		target.addInt(cyan);
	}


	if (magenta)
	{
		float avrx = (float) magentax / magenta;
		float avry = (float) magentay / magenta;

		//add cross head into the output image, locating target
		for (int x = ((avrx - 10) < 0 ? 0 : avrx - 10); x < ((avrx + 10)
				>= imgout->width() ? imgout->width() : avrx + 10); x++)
		{
			imgout->pixel(x, avry).r = 255;
			imgout->pixel(x, avry).g = 0;
			imgout->pixel(x, avry).b = 255;
		}
		for (int y = ((avry - 10) < 0 ? 0 : avry - 10); y < ((avry + 10)
				>= imgout->height() ? imgout->height() : avry + 10); y++)
		{
			imgout->pixel(avrx, y).r = 255;
			imgout->pixel(avrx, y).g = 0;
			imgout->pixel(avrx, y).b = 255;
		}

		printf(
				"The magenta stimulus is visible at coordinates (%.3f,%.3f)\n",
				avrx, avry);
		target.addString("magenta");
		target.addDouble(avrx);
		target.addDouble(avry);
		target.addInt(magenta);
	}

	if (white)
	{
		float avrx = (float) whitex / white;
		float avry = (float) whitey / white;

		//add cross head into the output image, locating target
		for (int x = ((avrx - 10) < 0 ? 0 : avrx - 10); x < ((avrx + 10)
				>= imgout->width() ? imgout->width() : avrx + 10); x++)
		{
			imgout->pixel(x, avry).r = 0;
			imgout->pixel(x, avry).g = 0;
			imgout->pixel(x, avry).b = 0;
		}
		for (int y = ((avry - 10) < 0 ? 0 : avry - 10); y < ((avry + 10)
				>= imgout->height() ? imgout->height() : avry + 10); y++)
		{
			imgout->pixel(avrx, y).r = 0;
			imgout->pixel(avrx, y).g = 0;
			imgout->pixel(avrx, y).b = 0;
		}

		printf(
				"The white stimulus is visible at coordinates (%.3f,%.3f)\n",
				avrx, avry);
		target.addString("white");
		target.addDouble(avrx);
		target.addDouble(avry);
		target.addInt(white);
	}



	if(red || green || blue || yellow || white || cyan || magenta)
		porttargets.write();


	//add rectangle highlighting forvea box onto image
	for (int x = 0; x<32 ; x++)
	{
		int px = imgout->width()/2-16+x;
		int topy = imgout->height()/2-16;
		int boty = imgout->height()/2+16;

		imgout->pixel(px, topy).r = 255;
		imgout->pixel(px, topy).g = 255;
		imgout->pixel(px, topy).b = 255;

		imgout->pixel(px, boty).r = 255;
		imgout->pixel(px, boty).g = 255;
		imgout->pixel(px, boty).b = 255;

	}
	for (int y = 0; y<32 ; y++)
	{
		int py = imgout->height()/2-16+y;
		int topx = imgout->width()/2-16;
		int botx = imgout->width()/2+16;

		imgout->pixel(topx, py).r = 255;
		imgout->pixel(topx, py).g = 255;
		imgout->pixel(topx, py).b = 255;

		imgout->pixel(botx, py).r = 255;
		imgout->pixel(botx, py).g = 255;
		imgout->pixel(botx, py).b = 255;

	}

}


bool fovea(int x, int y, double* dist)
{
	int centreX = 320/2;
	int centreY = 240/2;


	*dist = sqrt((x - centreX)*(x - centreX) + (y - centreY)*(y - centreY));
	if(*dist <= 16)
		return true;
	else
		return false;
}
