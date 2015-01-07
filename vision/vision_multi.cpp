/**
 *	Vision.cpp
 *	This program identifies particular coloured pixels in an image and generates target locations
 *	based on the averages of the pixel locations, one target per colour.
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
 *  v5: separating coloured blobs
 *
 *
 *
 *
 *
 *
 *  blob detection algorithm (www.societyofrobots.com/programming_computer_vision_tutorial_pt3.shtml
 *  	go through each pixel in the array
 *  	if the pixel is a blob colour, label it '1'
 *  		otherwise label it '0'
 *
 *  	go to the next pixel
 *  		if it is also a blob colour
 *  			and it is adjacent to blob 1
 *  				label it '1'
 *  			else
 *  				label it '2' (or more)
 *
 *  	repeat until all pixels are done
 *
 *  blob struct would need to store pixel pairs of all pixels in that blob to test if another
 *   pixel is in that blob.  Could define a bounding box? with max min and threshold. This
 *   generic threshold would work for blobs that are clearly separated.  If they are close,
 *   they could get confused.
 *
 *   Works for separating out different blobs quite nicely, with a limit on the minimum size of
 *   a blob to report, however no guaranteed way to distinguish between two blobs for target tracking.
 *   NOTE: also reporting on the size of the target.  This will affect target.cpp
 *
 */

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/sig/all.h>
#include <algorithm>
#include <iostream>
#include <math.h>
#include <string.h>
#include <stdlib.h>		//provides atexit
#include <vector>

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;


struct blob{

	blob(string col, int x, int y)
	{
		colour = col;
		xSum = x;
		ySum = y;
		pixelCount = 1;
		minX = x; maxX = x;
		minY = y; maxY = y;
		threshold = 5;
	}

	void add(int x, int y)
	{
		xSum += x;
		ySum += y;
		pixelCount ++;

		if(x>maxX)
			maxX = x;
		else if(x<minX)
			minX = x;

		if(y>maxY)
			maxY = y;
		else if(y<minY)
			minY = y;
	}

	bool inThreshold(int x, int y)
	{
		return x<(maxX+threshold)&& x>(minX-threshold)
				&& y<(maxY+threshold) && y>(minY-threshold);
	}

	void blobCentre(double* x, double* y)
	{
		*x = xSum/pixelCount;
		*y = ySum/pixelCount;
	}


	string colour;
	int pixelCount;
	int xSum, ySum;

	int minX,maxX, minY,maxY;	//basic bounding box
	int threshold;
};







void findTarget(ImageOf<PixelRgb>* imgin, ImageOf<PixelRgb>* imgout);
bool fovea(int x, int y, double* dist);



void quit()
{
	printf("Quit.\n\n");
}
BufferedPort < Bottle > porttargets;
int main(int argc, char* argv[])
{
//	Network yarp;
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

	string portname = "/target/";
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
	vector<blob*> blobs;
	blobs.reserve(5);

	int xout, yout;

	int centreX = imgin->width()/2;
	int centreY = imgin->height()/2;
	cout << "Central colour: " << (int)(imgin->pixel(centreX, centreY).r) << " "
							   << (int)(imgin->pixel(centreX, centreY).g) << " "
							   << (int)(imgin->pixel(centreX, centreY).b) << endl;



	//	saliency and coordinate counts/sums
	int green = 0, blue = 0, red = 0, yellow = 0, white=0;
	unsigned long redx = 0, redy = 0, greenx = 0, greeny = 0,
			yellowx = 0, yellowy = 0, bluex = 0, bluey = 0, whitex = 0, whitey = 0;
	bool coloured = false;
	for (int x = 0; x < imgin->width(); x++)
	{
		for (int y = 0; y < imgin->height(); y++)
		{
			//Salt and pepper filter
			if(y%2 == 0)
			{
				if(x%2!=0)
				{
					imgout->pixel(x,y).r = imgin->pixel(x,y).r;
					imgout->pixel(x,y).g = imgin->pixel(x,y).g;
					imgout->pixel(x,y).b = imgin->pixel(x,y).b;
					continue;
				}
			}
			else
			{
				if(x%2 == 0)
				{
					imgout->pixel(x,y).r = imgin->pixel(x,y).r;
					imgout->pixel(x,y).g = imgin->pixel(x,y).g;
					imgout->pixel(x,y).b = imgin->pixel(x,y).b;
					continue;
				}
			}
			coloured = false;

			double r = imgin->pixel(x,y).r;
			double g = imgin->pixel(x,y).g;
			double b = imgin->pixel(x,y).b;

//			//bright yellow
//			if(     r >= 250 &&
//					g >= 230 &&
//					b >= 105 && b<=120)
//			{
////				check for matching neighbours
//				if((x<imgout->width()-1 &&
//						imgin->pixel(x+1, y).r >= 250 &&
//						imgin->pixel(x+1, y).g >= 230 &&
//						imgin->pixel(x+1, y).b >= 105 && imgin->pixel(x+1, y).b<=120) &&
//					(x>0 &&
//						imgin->pixel(x-1, y).r >= 250 &&
//						imgin->pixel(x-1, y).g >= 230 &&
//						imgin->pixel(x-1, y).b >= 105 && imgin->pixel(x-1, y).b<=120) &&
//					(y<imgout->height()-1 &&
//						imgin->pixel(x, y+1).r >= 250 &&
//						imgin->pixel(x, y+1).g >= 230 &&
//						imgin->pixel(x, y+1).b >= 105 && imgin->pixel(x, y+1).b<=120) &&
//					(y>0 &&
//						imgin->pixel(x, y-1).r >= 250 &&
//						imgin->pixel(x, y-1).g >= 230 &&
//						imgin->pixel(x, y-1).b >= 105 && imgin->pixel(x, y-1).b<=120))
//				{
//					yellowx += x;
//					yellowy += y;
//					yellow ++;
//					imgout->pixel(x,y).r = 255;
//					imgout->pixel(x,y).g = 255;
//					imgout->pixel(x,y).b = 0;
//					coloured = true;
//				}
//			}


			//bright yellow
			if(     imgin->pixel(x,y).r > (imgin->pixel(x,y).g + 20) &&
					imgin->pixel(x,y).g > 180 &&
					imgin->pixel(x,y).b > 0 && imgin->pixel(x,y).b<100)
			{
				//check for matching neighbours
//				if((x<imgout->width()-1 &&
//						imgin->pixel(x+1, y).r > (imgin->pixel(x+1, y).g + 20) &&
//						imgin->pixel(x+1, y).g > 180 &&
//						imgin->pixel(x+1, y).b > 0 && imgin->pixel(x+1, y).b<100) &&
//					(x>0 &&
//						imgin->pixel(x-1, y).r > (imgin->pixel(x-1, y).g + 20) &&
//						imgin->pixel(x-1, y).g > 180 &&
//						imgin->pixel(x-1, y).b > 0 && imgin->pixel(x-1, y).b<100) &&
//					(y<imgout->height()-1 &&
//						imgin->pixel(x, y+1).r > (imgin->pixel(x, y+1).g + 20) &&
//						imgin->pixel(x, y+1).g > 180 &&
//						imgin->pixel(x, y+1).b > 0 && imgin->pixel(x, y+1).b<100) &&
//					(y>0 &&
//						imgin->pixel(x, y-1).r > (imgin->pixel(x, y-1).g + 20) &&
//						imgin->pixel(x, y-1).g > 180 &&
//						imgin->pixel(x, y-1).b > 0 && imgin->pixel(x, y-1).b<100))
//				{

					bool found = false;
					for(size_t i=0; i<blobs.size(); i++)
					{
						blob* b = blobs.at(i);
						if(b->colour.compare("yellow")==0 && b->inThreshold(x,y))
						{
							found = true;
							b->add(x,y);
						}
					}
					if(!found)
					{
						blobs.push_back(new blob("yellow",x,y));
					}

//					yellowx += x;
//					yellowy += y;
//					yellow ++;
					imgout->pixel(x,y).r = 255;
					imgout->pixel(x,y).g = 255;
					imgout->pixel(x,y).b = 0;
					coloured = true;
//				}
			}


			//bright white
			else if(     r >= 253 &&
					g >= 253 &&
					b >= 253)
			{
//				check for matching neighbours
//				if((x<imgout->width()-1 &&
//						imgin->pixel(x+1, y).r >= 253 &&
//						imgin->pixel(x+1, y).g >= 253 &&
//						imgin->pixel(x+1, y).b >= 253) &&
//					(x>0 &&
//						imgin->pixel(x-1, y).r >= 253 &&
//						imgin->pixel(x-1, y).g >= 253 &&
//						imgin->pixel(x-1, y).b >= 253) &&
//					(y<imgout->height()-1 &&
//						imgin->pixel(x, y+1).r >= 253 &&
//						imgin->pixel(x, y+1).g >= 253 &&
//						imgin->pixel(x, y+1).b >= 253) &&
//					(y>0 &&
//						imgin->pixel(x, y-1).r >= 253 &&
//						imgin->pixel(x, y-1).g >= 253 &&
//						imgin->pixel(x, y-1).b >= 253))
//				{

					bool found = false;
					for(size_t i=0; i<blobs.size(); i++)
					{
						blob* b = blobs.at(i);
						if(b->colour.compare("white")==0 && b->inThreshold(x,y))
						{
							found = true;
							b->add(x,y);
						}
					}
					if(!found)
					{
						blobs.push_back(new blob("white",x,y));
					}

//					whitex += x;
//					whitey += y;
//					white ++;
					imgout->pixel(x,y).r = 128;
					imgout->pixel(x,y).g = 128;
					imgout->pixel(x,y).b = 128;
					coloured = true;
//				}
			}

			//green button

			else if(g >= 80 &&
					(r / g) >= 0.70 && (r / g) <= 0.85 &&  	//relative values
					(b / g) >= 0.50 && (b / g) <= 0.58)
			{
//			else if(imgin->pixel(x,y).r > 94 && imgin->pixel(x,y).r < 130 &&	//fixed values
//					imgin->pixel(x,y).g > 123 && imgin->pixel(x,y).g < 165 &&
//					imgin->pixel(x,y).b > 66  && imgin->pixel(x,y).b < 93)
//			{
//
				//check for matching neighbours
				bool neighbourCheck = true;
				double r1,g1,b1;

//				if(x < (imgout->width()-1))
//				{
//					r1 = imgin->pixel(x+1,y).r;
//					g1 = imgin->pixel(x+1,y).g;
//					b1 = imgin->pixel(x+1,y).b;
//					neighbourCheck &= (g1 >= 80 &&
//									(r1 /g1) >= 0.70 && (r1 /g1) <= 0.85 &&
//									(b1 / g1) >= 0.50 && (b1 / g1) <= 0.58);
//				}
//
//				if(neighbourCheck && y<imgout->height()-1)
//				{
//					r1 = imgin->pixel(x,y+1).r;
//					g1 = imgin->pixel(x,y+1).g;
//					b1 = imgin->pixel(x,y+1).b;
//					neighbourCheck &= (g1 >= 80 &&
//										(r1 /g1) >= 0.70 && (r1 /g1) <= 0.85 &&
//										(b1 / g1) >= 0.50 && (b1 / g1) <= 0.58);
//				}
//
//				if(neighbourCheck && x>0)
//				{
//					r1 = imgin->pixel(x-1,y).r;
//					g1 = imgin->pixel(x-1,y).g;
//					b1 = imgin->pixel(x-1,y).b;
//					neighbourCheck &= (g1 >= 80 &&
//										(r1 /g1) >= 0.70 && (r1 /g1) <= 0.85 &&
//										(b1 / g1) >= 0.50 && (b1 / g1) <= 0.58);
//				}
//
//				if(neighbourCheck && y>0)
//				{
//					r1 = imgin->pixel(x,y-1).r;
//					g1 = imgin->pixel(x,y-1).g;
//					b1 = imgin->pixel(x,y-1).b;
//					neighbourCheck &= (g1 >= 80 &&
//										(r1 /g1) >= 0.70 && (r1 /g1) <= 0.85 &&
//										(b1 / g1) >= 0.50 && (b1 / g1) <= 0.58);
//				}
//
//				if(neighbourCheck)
//				{
					bool found = false;
					for(size_t i=0; i<blobs.size(); i++)
					{
						blob* b = blobs.at(i);
						if(b->colour.compare("green")==0 && b->inThreshold(x,y))
						{
							found = true;
							b->add(x,y);
						}
					}
					if(!found)
					{
						blobs.push_back(new blob("green",x,y));
					}

//					greenx += x;
//					greeny += y;
//					green ++;
					imgout->pixel(x,y).r = 0;
					imgout->pixel(x,y).g = 255;
					imgout->pixel(x,y).b = 0;
					coloured = true;
//				}
			}



			//general red / pink / orange
			else if (r > (b + 80) &&
					 r > (g + 80) &&
					 r > 100)
			{
//
//				if((x<imgout->width()-1 && imgin->pixel(x+1, y).r > (imgin->pixel(x+1, y).b + 80)
//					&& imgin->pixel(x+1, y).r > (imgin->pixel(x+1, y).g + 80)
//					&& imgin->pixel(x+1, y).r > 100) &&
//					(x>0 && imgin->pixel(x-1, y).r > (imgin->pixel(x-1, y).b + 80)
//					&& imgin->pixel(x-1, y).r > (imgin->pixel(x-1, y).g + 80)
//					&& imgin->pixel(x-1, y).r > 100) &&
//					(y<imgout->height()-1 && imgin->pixel(x, y+1).r > (imgin->pixel(x, y+1).b + 80)
//					&& imgin->pixel(x, y+1).r > (imgin->pixel(x, y+1).g + 80)
//					&& imgin->pixel(x, y+1).r > 100) &&
//					(y>0 && imgin->pixel(x, y-1).r > (imgin->pixel(x, y-1).b + 80)
//					&& imgin->pixel(x, y-1).r > (imgin->pixel(x, y-1).g + 80)
//					&& imgin->pixel(x, y-1).r > 100))
//				{
					bool found = false;
					for(size_t i=0; i<blobs.size(); i++)
					{
						blob* b = blobs.at(i);
						if(b->colour.compare("red")==0 && b->inThreshold(x,y))
						{
							found = true;
							b->add(x,y);
						}
					}
					if(!found)
					{
						blobs.push_back(new blob("red",x,y));
					}

//					redx += x;
//					redy += y;
//					red++;
					imgout->pixel(x,y).r = 255;
					imgout->pixel(x,y).g = 0;
					imgout->pixel(x,y).b = 0;
					coloured = true;
//				}
			}

			//look for blue

			else if(b>= 80 &&
					r/b >= 0.80 && r/b <= 0.97 &&
					g/b >= 1.09 && g/b <= 1.28)
			{
				//check for matching neighbours
				bool neighbourCheck = true;
				double r1,g1,b1;

//				if(x < (imgout->width()-1))
//				{
//					r1 = imgin->pixel(x+1,y).r;
//					g1 = imgin->pixel(x+1,y).g;
//					b1 = imgin->pixel(x+1,y).b;
//					neighbourCheck &= (b1>= 80 &&
//										r1/b1 >= 0.80 && r1/b1 <= 0.97 &&
//										g1/b1 >= 1.09 && g1/b1 <= 1.28);
//				}
//
//				if(neighbourCheck && y<imgout->height()-1)
//				{
//					r1 = imgin->pixel(x,y+1).r;
//					g1 = imgin->pixel(x,y+1).g;
//					b1 = imgin->pixel(x,y+1).b;
//					neighbourCheck &= (b1>= 80 &&
//										r1/b1 >= 0.80 && r1/b1 <= 0.97 &&
//										g1/b1 >= 1.09 && g1/b1 <= 1.28);
//				}
//
//				if(neighbourCheck && x>0)
//				{
//					r1 = imgin->pixel(x-1,y).r;
//					g1 = imgin->pixel(x-1,y).g;
//					b1 = imgin->pixel(x-1,y).b;
//					neighbourCheck &= (b1>= 80 &&
//										r1/b1 >= 0.80 && r1/b1 <= 0.97 &&
//										g1/b1 >= 1.09 && g1/b1 <= 1.28);
//				}
//
//				if(neighbourCheck && y>0)
//				{
//					r1 = imgin->pixel(x,y-1).r;
//					g1 = imgin->pixel(x,y-1).g;
//					b1 = imgin->pixel(x,y-1).b;
//					neighbourCheck &= (b1>= 80 &&
//										r1/b1 >= 0.80 && r1/b1 <= 0.97 &&
//										g1/b1 >= 1.09 && g1/b1 <= 1.28);
//				}
//
//				if(neighbourCheck)
//				{

					bool found = false;
					for(size_t i=0; i<blobs.size(); i++)
					{
						blob* b = blobs.at(i);
						if(b->colour.compare("blue")==0 && b->inThreshold(x,y))
						{
							found = true;
							b->add(x,y);
						}
					}
					if(!found)
					{
						blobs.push_back(new blob("blue",x,y));
					}

//					bluex += x;
//					bluey += y;
//					blue++;
					imgout->pixel(x,y).r = 0;
					imgout->pixel(x,y).g = 0;
					imgout->pixel(x,y).b = 255;
					coloured = true;
//				}
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


	int tooSmall = 0;
	for(size_t i = 0; i<blobs.size(); i++)
	{
		blob* b = blobs.at(i);
		if(b->pixelCount < 10)
		{
			//want to remove this as well
			tooSmall ++;
			continue;
		}

		double avrx, avry;
		b->blobCentre(&avrx, &avry);
		string colour = b->colour;
		int red,green,blue;
		if(colour.compare("red")==0)
		{
			red = 255;
			green = 128;
			blue = 0;
		}
		else if(colour.compare("green")==0)
		{
			red = 0;
			green = 255;
			blue = 128;
		}
		else if(colour.compare("yellow")==0)
		{
			red = 255;
			green = 255;
			blue = 128;
		}
		else if(colour.compare("blue")==0)
		{
			red = 128;
			green = 0;
			blue = 255;
		}
		else if(colour.compare("white")==0)
		{
			red = 0;
			green = 0;
			blue = 0;
		}
		else
		{
			red = 255;
			green = 255;
			blue = 255;
		}

		//add cross head into the output image, locating target
		for (int x = ((avrx - 10) < 0 ? 0 : avrx - 10); x < ((avrx + 10)
				>= imgout->width() ? imgout->width() : avrx + 10); x++)
		{
			imgout->pixel(x, avry).r = red;
			imgout->pixel(x, avry).g = green;
			imgout->pixel(x, avry).b = blue;
		}
		for (int y = ((avry - 10) < 0 ? 0 : avry - 10); y < ((avry + 10)
				>= imgout->height() ? imgout->height() : avry + 10); y++)
		{
			imgout->pixel(avrx, y).r = red;
			imgout->pixel(avrx, y).g = green;
			imgout->pixel(avrx, y).b = blue;
		}

		target.addString(colour.c_str());
		target.addDouble(avrx);
		target.addDouble(avry);
		target.addInt(b->pixelCount);	//size

	}

	if((blobs.size()-tooSmall)>0)
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
