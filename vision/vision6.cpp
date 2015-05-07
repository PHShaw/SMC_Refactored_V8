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
 *
 *
 */

#include "vision6.h"

using namespace std;
//using namespace yarp::os;		//conflicts with signal
using namespace yarp::sig;

Vision vision;

Vision :: Vision()
{
	checkRed = true;
	checkYellow = true;
	checkBlue = false;
	checkGreen = true;
	checkWhite = false;
	checkGrey = false;

	setAcuity(3);
	setFov(100);


}

void Vision::init(string pcam)
{
	yarp::os::Network yarp;
	cam = pcam;

	string portname = "/target/";
	portname += cam;
	portname += "/";

	string ending = portname;
	ending += "in";

	camPortin.open(ending.c_str());

	ending = portname;
	ending += "out";

	camPortout.open(ending.c_str());

	ending = portname;
	ending += "data";
	porttargets.open(ending.c_str());

	visionParamsIn.open("/target/parameter");
	visionParamsIn.useCallback(*this);

}

void Vision::onRead(yarp::os::Bottle& b) {
	 std::cout<<"[BOTTLE] received: '"<<b.toString()<<"' size: "<<b.size()<<std::endl;
	 if(b.size()%2 != 0)
	 {
		 cout << "Un-even bottle size" << endl;
		 return;
	 }

	 for(int i=0; i<b.size(); i+=2)
	 {
		 string name = b.get(i).asString().c_str();
		 int value = b.get(i+1).asInt();

		 if(name == "acuity") setAcuity(value);
		 else if(name == "fov") setFov(value);
//		 else if(name == "brightness") setAcuity(value);
//		 else if(name == "threshold") setTreshold(value);
		 else std::cout<<"Unknown message type"<<std::endl;
	 }

}

void Vision::setAcuity(int value)
{
	if(value>0 && value<=100)
		acuity = value;
	if(acuity%2==0)
		acuity++;
}
void Vision::setFov(int value)
{
	if(value>0 && value<=100)
		fov = value;
}


void Vision::closePorts()
{
	porttargets.close();
	camPortin.close();
	camPortout.close();
	visionParamsIn.close();
	cvReleaseImage(&cvImage);
}

void Vision::run()
{
	ImageOf < PixelBgr > *imgin;
	while (1)
	{
		if (imgin = camPortin.read(1))
		{
			ImageOf < PixelBgr > &imgout = camPortout.prepare();
			imgout.resize(*imgin);
			height = imgin->height();
			width = imgin->width();

			ImageOf<PixelBgr> yarpImage = colourSwitch(imgin);

			yarpImage = adjustAccuity(acuity, &yarpImage);
			yarpImage = adjustFOV(fov, &yarpImage);

			findTarget(&yarpImage, &imgout);
			camPortout.write();
		}
		else
		{
			printf("no imgin\n");
			yarp::os::Time::delay(1);
		}
//		yarp::os::Time::delay(0.05);
	}
}
void quit(int param)
{
	vision.closePorts();
	printf("Quit.\n\n");
	std::exit(param);
}


int main(int argc, char* argv[])
{
	yarp::os::Property options;
	if(argc>=2)
		options.fromCommand(argc,argv);

	string cam;
	yarp::os::Value* val;
	if(options.check("cam",val))
	{
		cam = val->asString().c_str();
	}
	else
	{
		cam = "right";
	}

	vision.init(cam);
	signal(SIGINT, quit);

	vision.run();

}

void Vision::findTarget(ImageOf<PixelBgr>* imgin, ImageOf<PixelBgr>* imgout)
{
	int xout, yout;
//	printf("Entering find target\n");

	//	saliency and coordinate counts/sums
	int green = 0, blue = 0, red = 0, yellow = 0, white=0, gray = 0;
	unsigned long redx = 0, redy = 0, greenx = 0, greeny = 0, grayx=0, grayy=0,
			yellowx = 0, yellowy = 0, bluex = 0, bluey = 0, whitex = 0, whitey = 0;
	bool coloured = false;
	for (int x = 0; x < imgin->width(); x++)
	{
		for (int y = 1; y < imgin->height(); y++)
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
			if(checkYellow && !coloured)
			{

				if(     ((imgin->pixel(x,y).r > (imgin->pixel(x,y).g + 20)) ||
						imgin->pixel(x,y).r ==255)&& imgin->pixel(x,y).r>230 &&
						imgin->pixel(x,y).g > 180 &&
						imgin->pixel(x,y).b > 0 && imgin->pixel(x,y).b<108)
				{
					//check for matching neighbours
					if((x<imgout->width()-1 &&
							((imgin->pixel(x+1, y).r > (imgin->pixel(x+1, y).g + 20)) ||
							imgin->pixel(x+1,y).r ==255) &&
							imgin->pixel(x+1, y).g > 180 &&
							imgin->pixel(x+1, y).b > 0 && imgin->pixel(x+1, y).b<108) &&
						(x>0 &&
							((imgin->pixel(x-1, y).r > (imgin->pixel(x-1, y).g + 20)) ||
							imgin->pixel(x-1,y).r ==255) &&
							imgin->pixel(x-1, y).g > 180 &&
							imgin->pixel(x-1, y).b > 0 && imgin->pixel(x-1, y).b<108) &&
						(y<imgout->height()-1 &&
							((imgin->pixel(x, y+1).r > (imgin->pixel(x, y+1).g + 20)) ||
							imgin->pixel(x,y+1).r ==255) &&
							imgin->pixel(x, y+1).g > 180 &&
							imgin->pixel(x, y+1).b > 0 && imgin->pixel(x, y+1).b<108) &&
						(y>0 &&
							((imgin->pixel(x, y-1).r > (imgin->pixel(x, y-1).g + 20)) ||
							imgin->pixel(x,y-1).r ==255) &&
							imgin->pixel(x, y-1).g > 180 &&
							imgin->pixel(x, y-1).b > 0 && imgin->pixel(x, y-1).b<108))
					{
						yellowx += x;
						yellowy += y;
						yellow ++;
						imgout->pixel(x,y).r = 255;
						imgout->pixel(x,y).g = 255;
						imgout->pixel(x,y).b = 0;
						coloured = true;
					}
				}
			}

			//bright white
			if(checkWhite && !coloured)
			{
				if(     r >= 253 &&
						g >= 253 &&
						b >= 253)
				{
	//				check for matching neighbours
					if((x<imgout->width()-1 &&
							imgin->pixel(x+1, y).r >= 253 &&
							imgin->pixel(x+1, y).g >= 253 &&
							imgin->pixel(x+1, y).b >= 253) &&
						(x>0 &&
							imgin->pixel(x-1, y).r >= 253 &&
							imgin->pixel(x-1, y).g >= 253 &&
							imgin->pixel(x-1, y).b >= 253) &&
						(y<imgout->height()-1 &&
							imgin->pixel(x, y+1).r >= 253 &&
							imgin->pixel(x, y+1).g >= 253 &&
							imgin->pixel(x, y+1).b >= 253) &&
						(y>0 &&
							imgin->pixel(x, y-1).r >= 253 &&
							imgin->pixel(x, y-1).g >= 253 &&
							imgin->pixel(x, y-1).b >= 253))
					{
						whitex += x;
						whitey += y;
						white ++;
						imgout->pixel(x,y).r = 128;
						imgout->pixel(x,y).g = 128;
						imgout->pixel(x,y).b = 128;
						coloured = true;
					}
				}
			}

			//green button
			if(checkGreen && !coloured)
			{
				if(g >= 80 &&
					(r / g) >= 0.60 && (r / g) <= 0.92 &&  	//relative values
					(b / g) >= 0.40 && (b / g) <= 0.58)
				{
	//			else if(imgin->pixel(x,y).r > 94 && imgin->pixel(x,y).r < 130 &&	//fixed values
	//					imgin->pixel(x,y).g > 123 && imgin->pixel(x,y).g < 165 &&
	//					imgin->pixel(x,y).b > 66  && imgin->pixel(x,y).b < 93)
	//			{
	//
					//check for matching neighbours
					bool neighbourCheck = true;
					double r1,g1,b1;

					if(x < (imgout->width()-1))
					{
						r1 = imgin->pixel(x+1,y).r;
						g1 = imgin->pixel(x+1,y).g;
						b1 = imgin->pixel(x+1,y).b;
						neighbourCheck &= (g1 >= 80 &&
										(r1 /g1) >= 0.60 && (r1 /g1) <= 0.92 &&
										(b1 / g1) >= 0.40 && (b1 / g1) <= 0.58);
					}

					if(neighbourCheck && y<imgout->height()-1)
					{
						r1 = imgin->pixel(x,y+1).r;
						g1 = imgin->pixel(x,y+1).g;
						b1 = imgin->pixel(x,y+1).b;
						neighbourCheck &= (g1 >= 80 &&
											(r1 /g1) >= 0.60 && (r1 /g1) <= 0.92 &&
											(b1 / g1) >= 0.40 && (b1 / g1) <= 0.58);
					}

					if(neighbourCheck && x>0)
					{
						r1 = imgin->pixel(x-1,y).r;
						g1 = imgin->pixel(x-1,y).g;
						b1 = imgin->pixel(x-1,y).b;
						neighbourCheck &= (g1 >= 80 &&
											(r1 /g1) >= 0.60 && (r1 /g1) <= 0.92 &&
											(b1 / g1) >= 0.40 && (b1 / g1) <= 0.58);
					}

					if(neighbourCheck && y>0)
					{
						r1 = imgin->pixel(x,y-1).r;
						g1 = imgin->pixel(x,y-1).g;
						b1 = imgin->pixel(x,y-1).b;
						neighbourCheck &= (g1 >= 80 &&
											(r1 /g1) >= 0.60 && (r1 /g1) <= 0.92 &&
											(b1 / g1) >= 0.40 && (b1 / g1) <= 0.58);
					}

					if(neighbourCheck)
					{
						greenx += x;
						greeny += y;
						green ++;
						imgout->pixel(x,y).r = 0;
						imgout->pixel(x,y).g = 255;
						imgout->pixel(x,y).b = 0;
						coloured = true;
					}
				}
			}

			if(checkRed && !coloured)
			{
				//general red / pink / orange
				if (r > (b + 80) &&
						 r > (g + 80) &&
						 r > 100)
				{

					if((x<imgout->width()-1 && imgin->pixel(x+1, y).r > (imgin->pixel(x+1, y).b + 80)
						&& imgin->pixel(x+1, y).r > (imgin->pixel(x+1, y).g + 80)
						&& imgin->pixel(x+1, y).r > 100) &&
						(x>0 && imgin->pixel(x-1, y).r > (imgin->pixel(x-1, y).b + 80)
						&& imgin->pixel(x-1, y).r > (imgin->pixel(x-1, y).g + 80)
						&& imgin->pixel(x-1, y).r > 100) &&
						(y<imgout->height()-1 && imgin->pixel(x, y+1).r > (imgin->pixel(x, y+1).b + 80)
						&& imgin->pixel(x, y+1).r > (imgin->pixel(x, y+1).g + 80)
						&& imgin->pixel(x, y+1).r > 100) &&
						(y>0 && imgin->pixel(x, y-1).r > (imgin->pixel(x, y-1).b + 80)
						&& imgin->pixel(x, y-1).r > (imgin->pixel(x, y-1).g + 80)
						&& imgin->pixel(x, y-1).r > 100))
					{

						redx += x;
						redy += y;
						red++;
						imgout->pixel(x,y).r = 255;
						imgout->pixel(x,y).g = 0;
						imgout->pixel(x,y).b = 0;
						coloured = true;
					}
				}
			}

			//look for blue
			if(checkBlue && !coloured)
			{
				if(b>= 70 &&
						r/b >= 0.70 && r/b <= 0.97 &&
						g/b >= 1.09 && g/b <= 1.37)
				{
					//check for matching neighbours
					bool neighbourCheck = true;
					double r1,g1,b1;

					if(x < (imgout->width()-1))
					{
						r1 = imgin->pixel(x+1,y).r;
						g1 = imgin->pixel(x+1,y).g;
						b1 = imgin->pixel(x+1,y).b;
						neighbourCheck &= (b1>= 70 &&
											r1/b1 >= 0.70 && r1/b1 <= 0.97 &&
											g1/b1 >= 1.09 && g1/b1 <= 1.37);
					}

					if(neighbourCheck && y<imgout->height()-1)
					{
						r1 = imgin->pixel(x,y+1).r;
						g1 = imgin->pixel(x,y+1).g;
						b1 = imgin->pixel(x,y+1).b;
						neighbourCheck &= (b1>= 70 &&
											r1/b1 >= 0.70 && r1/b1 <= 0.97 &&
											g1/b1 >= 1.09 && g1/b1 <= 1.37);
					}

					if(neighbourCheck && x>0)
					{
						r1 = imgin->pixel(x-1,y).r;
						g1 = imgin->pixel(x-1,y).g;
						b1 = imgin->pixel(x-1,y).b;
						neighbourCheck &= (b1>= 70 &&
											r1/b1 >= 0.70 && r1/b1 <= 0.97 &&
											g1/b1 >= 1.09 && g1/b1 <= 1.37);
					}

					if(neighbourCheck && y>0)
					{
						r1 = imgin->pixel(x,y-1).r;
						g1 = imgin->pixel(x,y-1).g;
						b1 = imgin->pixel(x,y-1).b;
						neighbourCheck &= (b1>= 70 &&
											r1/b1 >= 0.70 && r1/b1 <= 0.97 &&
											g1/b1 >= 1.09 && g1/b1 <= 1.37);
					}

					if(neighbourCheck)
					{
						bluex += x;
						bluey += y;
						blue++;
						imgout->pixel(x,y).r = 0;
						imgout->pixel(x,y).g = 0;
						imgout->pixel(x,y).b = 255;
						coloured = true;
					}
				}
			}



			//look for blue tape (gray)
			if(checkGrey && !coloured)
			{
				if(r>105 && r<118 && g>110 && g<128 && b>90 && b<120 &&
						r/b >= 0.90 && r/b <= 1.25 &&
						g/b >= 1.02 && g/b <= 1.37)
				{
					//check for matching neighbours
					bool neighbourCheck = true;
					double r1,g1,b1;

					if(x < (imgout->width()-1))
					{
						r1 = imgin->pixel(x+1,y).r;
						g1 = imgin->pixel(x+1,y).g;
						b1 = imgin->pixel(x+1,y).b;
						neighbourCheck &= (	r1/b1 >= 0.96 && r1/b1 <= 1.25 &&
											g1/b1 >= 1.02 && g1/b1 <= 1.37);
					}

					if(neighbourCheck && y<imgout->height()-1)
					{
						r1 = imgin->pixel(x,y+1).r;
						g1 = imgin->pixel(x,y+1).g;
						b1 = imgin->pixel(x,y+1).b;
						neighbourCheck &= ( r1/b1 >= 0.96 && r1/b1 <= 1.25 &&
											g1/b1 >= 1.02 && g1/b1 <= 1.37);
					}

					if(neighbourCheck && x>0)
					{
						r1 = imgin->pixel(x-1,y).r;
						g1 = imgin->pixel(x-1,y).g;
						b1 = imgin->pixel(x-1,y).b;
						neighbourCheck &= ( r1/b1 >= 0.96 && r1/b1 <= 1.25 &&
											g1/b1 >= 1.02 && g1/b1 <= 1.37);
					}

					if(neighbourCheck && y>0)
					{
						r1 = imgin->pixel(x,y-1).r;
						g1 = imgin->pixel(x,y-1).g;
						b1 = imgin->pixel(x,y-1).b;
						neighbourCheck &= ( r1/b1 >= 0.96 && r1/b1 <= 1.25 &&
											g1/b1 >= 1.02 && g1/b1 <= 1.37);
					}

					if(neighbourCheck)
					{
						grayx += x;
						grayy += y;
						gray++;
						imgout->pixel(x,y).r = 153;
						imgout->pixel(x,y).g = 153;
						imgout->pixel(x,y).b = 153;
						coloured = true;
					}
				}
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


	yarp::os::Bottle& target = porttargets.prepare();
	target.clear();

	if (yellow)
	{
		float avrx = (float) yellowx / yellow;
		float avry = (float) yellowy / yellow;

		//add cross head into the output image, locating target
		for (int x = ((avrx - 10) < 0 ? 0 : avrx - 10); x < ((avrx + 10)
				>= imgout->width() ? imgout->width() : avrx + 10); x++)
		{
			imgout->pixel(x, avry).r = 255;
			imgout->pixel(x, avry).g = 189;
			imgout->pixel(x, avry).b = 136;
		}
		for (int y = ((avry - 10) < 0 ? 0 : avry - 10); y < ((avry + 10)
				>= imgout->height() ? imgout->height() : avry + 10); y++)
		{
			imgout->pixel(avrx, y).r = 255;
			imgout->pixel(avrx, y).g = 189;
			imgout->pixel(avrx, y).b = 136;
		}

		printf(
				"The yellow stimulus is visible at coordinates (%.3f,%.3f)\n",
				avrx, avry);

		target.addString("yellow");
		target.addDouble(avrx);
		target.addDouble(avry);
		target.addInt(yellow);
	}


	if (red)
	{
		double avrx = (double) redx / red;
		double avry = (double) redy / red;

		//add cross head into the output image, locating target
		for (int x = ((avrx - 10) < 0 ? 0 : avrx - 10); x < ((avrx + 10)
				>= imgout->width() ? imgout->width() : avrx + 10); x++)
		{
			imgout->pixel(x, avry).r = 255;
			imgout->pixel(x, avry).g = 102;
			imgout->pixel(x, avry).b = 0;
		}
		for (int y = ((avry - 10) < 0 ? 0 : avry - 10); y < ((avry + 10)
				>= imgout->height() ? imgout->height() : avry + 10); y++)
		{
			imgout->pixel(avrx, y).r = 255;
			imgout->pixel(avrx, y).g = 102;
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
			imgout->pixel(x, avry).r = 51;
			imgout->pixel(x, avry).g = 153;
			imgout->pixel(x, avry).b = 51;
		}
		for (int y = ((avry - 10) < 0 ? 0 : avry - 10); y < ((avry + 10)
				>= imgout->height() ? imgout->height() : avry + 10); y++)
		{
			imgout->pixel(avrx, y).r = 51;
			imgout->pixel(avrx, y).g = 153;
			imgout->pixel(avrx, y).b = 51;
		}

		printf(
				"The green stimulus is visible at coordinates (%.3f,%.3f)\n",
				avrx, avry);

		target.addString("green");
		target.addDouble(avrx);
		target.addDouble(avry);
		target.addInt(green);
	}




	if (blue)
	{
		float avrx = (float) bluex / blue;
		float avry = (float) bluey / blue;

		//add cross head into the output image, locating target
		for (int x = ((avrx - 10) < 0 ? 0 : avrx - 10); x < ((avrx + 10)
				>= imgout->width() ? imgout->width() : avrx + 10); x++)
		{
			imgout->pixel(x, avry).r = 128;
			imgout->pixel(x, avry).g = 0;
			imgout->pixel(x, avry).b = 255;
		}
		for (int y = ((avry - 10) < 0 ? 0 : avry - 10); y < ((avry + 10)
				>= imgout->height() ? imgout->height() : avry + 10); y++)
		{
			imgout->pixel(avrx, y).r = 128;
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


	if (gray)
	{
		float avrx = (float) grayx / gray;
		float avry = (float) grayy / gray;

		//add cross head into the output image, locating target
		for (int x = ((avrx - 10) < 0 ? 0 : avrx - 10); x < ((avrx + 10)
				>= imgout->width() ? imgout->width() : avrx + 10); x++)
		{
			imgout->pixel(x, avry).r = 51;
			imgout->pixel(x, avry).g = 102;
			imgout->pixel(x, avry).b = 204;
		}
		for (int y = ((avry - 10) < 0 ? 0 : avry - 10); y < ((avry + 10)
				>= imgout->height() ? imgout->height() : avry + 10); y++)
		{
			imgout->pixel(avrx, y).r = 51;
			imgout->pixel(avrx, y).g = 102;
			imgout->pixel(avrx, y).b = 204;
		}

		printf(
				"The gray stimulus is visible at coordinates (%.3f,%.3f)\n",
				avrx, avry);
		target.addString("gray");
		target.addDouble(avrx);
		target.addDouble(avry);
		target.addInt(gray);
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



	if(red || green || blue || yellow || gray || white)
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


bool Vision::fovea(int x, int y, double* dist)
{
	int centreX = width/2;		//320 or 640
	int centreY = height/2;		//240 or 480


	*dist = sqrt((x - centreX)*(x - centreX) + (y - centreY)*(y - centreY));
	if(*dist <= 16)
		return true;
	else
		return false;
}

/**
 * For some reason, openCV uses BGR colour channels.  Can use this with yarp,
 * but the R and B channels get switched, so this switches them back.
 */
ImageOf<PixelBgr> Vision::colourSwitch(ImageOf<PixelBgr>* imgin)
{
//	printf("Entering colour Switch\n");
	ImageOf<PixelBgr> yarpImage;
	yarpImage.resize(width, height);

	for(int x=0; x<width; x++)
	{
		for(int y=0; y<height; y++)
		{
			yarpImage.pixel(x,y).r = imgin->pixel(x,y).b;
			yarpImage.pixel(x,y).g = imgin->pixel(x,y).g;
			yarpImage.pixel(x,y).b = imgin->pixel(x,y).r;
		}
	}
	return yarpImage;
}


/**
 *
 * percent: 100 = full field of view, 0 = no field of view
 */
ImageOf<PixelBgr> Vision::adjustFOV(int percent, ImageOf<PixelBgr>* imgin)
{
	ImageOf<PixelBgr> yarpImage;
	yarpImage.resize(width, height);

//	cout << "height: " << imgin->height() << endl;


//	printf("Entering adjust fov\n");
	if (percent <= 0  || percent>=100)
	{
		yarpImage = *imgin;
//		printf("No fov adjustments to be made\n");
		return yarpImage;
	}
	percent = 100-percent;

	int xPixels = width*percent/100;
	xPixels /=2;

	int yPixels = height*percent/100;
	yPixels /=2;

	for (int x = 0; x < width; x++)
	{
		for (int y = 1; y < height; y++)
		{
			if((x<xPixels || x>(width-xPixels)) ||
					(y<yPixels || y>(height-yPixels)))
			{
				yarpImage.pixel(x,y).r = 0;
				yarpImage.pixel(x,y).g = 0;
				yarpImage.pixel(x,y).b = 0;
			}
			else
			{
				yarpImage.pixel(x,y).r = imgin->pixel(x,y).r;
				yarpImage.pixel(x,y).g = imgin->pixel(x,y).g;
				yarpImage.pixel(x,y).b = imgin->pixel(x,y).b;
			}
		}

	}
	return yarpImage;


}

/**
 * Attempts to blur the image by specified percentage
 *
 * percent - 0 is maximum resolution, 100 is nothing
 */
ImageOf<PixelBgr> Vision::adjustAccuity(int percent, ImageOf<PixelBgr>* imgin)
{
	ImageOf<PixelBgr> yarpImage;
	yarpImage.resize(width, height);

//	printf("Entering adjust acuity\n");
	if (percent <= 0  || percent>=100)
	{
		yarpImage = *imgin;
//		printf("No acuity adjustments to be made\n");
		return yarpImage;
	}
//	printf("calculating block size\n");

	int hBlockSize = width*percent/100;
	int vBlockSize = height*percent/100;

//	printf("block size: %i, %i\n", hBlockSize, vBlockSize);
//	printf("image size: %i, %i\n", width, height);
	double avgr = 0;
	double avgg = 0;
	double avgb = 0;
	int count=0;

	/*

	for (int x = 0; x < width; x+=hBlockSize)
	{
		for (int y = 0; y < height; y+=vBlockSize)
		{
			avgr=0;
			avgg=0;
			avgb=0;
			count=0;


//			printf("(%i,%i)",x,y);

			for(int a=x; a<(hBlockSize+x) && a<width; a++)
			{
				for (int b=y; b<(vBlockSize+y) && b<height; b++)
				{
//					printf("(%i,%i)",a,b);
					double r,g,b;
					r=imgin->pixel(a,b).r;
					g=imgin->pixel(a,b).g;
					b=imgin->pixel(a,b).b;
					avgr += r;
					avgg += g;
					avgb += b;
//					avgr += (imgin->pixel(a,b).r);
//					avgg += imgin->pixel(a,b).g;
//					avgb += imgin->pixel(a,b).b;
					count ++;
				}
			}

			avgr /= count;
			avgg /= count;
			avgb /= count;

			for(int a=x; a<(hBlockSize+x) && a<width; a++)
			{
				for (int b=y; b<(vBlockSize+y) && b<height; b++)
				{
					imgout->pixel(a,b).r = avgr;
					imgout->pixel(a,b).g = avgg;
					imgout->pixel(a,b).b = avgb;
				}
			}

		}
	}
	*/



	//Create a new openCV image of correct size
	cvImage = cvCreateImage(cvSize(width,height),
                                      IPL_DEPTH_8U, 3 );
	//copy yarp image to an openCV/IPL image
	cvCvtColor((IplImage*)imgin->getIplImage(), cvImage, CV_RGB2BGR);



	//make sure block sizes are odd:
	if(hBlockSize%2 ==0)
		hBlockSize ++;
	if(vBlockSize%2 ==0)
		vBlockSize ++;
	//Blur the image
	cvSmooth(cvImage, cvImage, CV_GAUSSIAN,acuity,acuity);//hBlockSize,vBlockSize);

	//display image
//	cvNamedWindow("test",1);
//  cvShowImage("test",cvImage);
//	cvWaitKey(3000);
//	cvDestroyWindow("test");

    //return image to yarp
    yarpImage.wrapIplImage(cvImage);

    //cvReleaseImage(&cvImage);		//cvImage must continue to exist whilst yarpImage exists.

//	printf("processed image\n");
	return yarpImage;
}

