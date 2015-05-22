/*
 * BB_SimWorld.cpp
 *
 *  Created on: 20 May 2015
 *      Author: icub
 */

#include <iostream>
#include <stdlib.h>
#include <time.h>

#include <yarp/os/BufferedPort.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Network.h>
#include <yarp/os/Time.h>

yarp::os::BufferedPort<yarp::os::Bottle> worldCommands;

//world x: +left
//world y: +up
//world z: +forward
double x = 0.0; //left		//initial position: central 30cm in front of eyes.
double y = 0.9; //up
double z = 0.3; //forward

//Centre position
const double c_x = x;
const double c_y = y;
const double c_z = z;

const double lMax = 0.3;
const double rMax = -0.3;

const double fast = 0.002;
const double slow = 0.001;

void mkBox()
{
	yarp::os::Bottle& b = worldCommands.prepare();
		b.clear();
		//world mk sbox (three params for size) (three params for pos) (three params for colour)  ------- This creates a static box
		b.addString("world");
		b.addString("mk");
		b.addString("sbox");

		//dimensions
		b.addDouble(0.04);
		b.addDouble(0.04);
		b.addDouble(0.04);

		//position
		//world x: +left
		//world y: +up
		//world z: +forward
		b.addDouble(x);
		b.addDouble(y);
		b.addDouble(z);	//30 cm in front of eyes?

		//colour
		b.addInt(1);
		b.addInt(0);
		b.addInt(0);

		//remove collisins with robot
		b.addInt(0);
	worldCommands.write(true);
}



/**
 * Gradually shift object to target, rather than jumping
 */
void mvBox(const double targX, const double targY, const double targZ, const double speed, bool irregular=false)
{
	bool reached = false;
	int direction;
	if(x<targX)
		direction = 1;
	else if(x>targX)
		direction = -1;
	else direction = 0;

	time_t startSeconds;
	startSeconds = time(NULL);
	time_t timeTaken = startSeconds-startSeconds;

	while(!reached)
	{
		reached = true;
		//if x<targX -> x++ x>(targX+speed)
		//if x>targX -> x-- unless x<(targX-speed)
		//if


		if(x<targX && direction==1)
		{
//			std::cout << "x<targX" << std::endl;
			x+= speed;
			reached = false;
		}
		else if(x>targX && direction==-1) //todo
		{
//			std::cout << "x>targX" << std::endl;
			x-= speed;
			reached = false;
		}

		if(y<targY)
		{
//			std::cout << "y<targY" << std::endl;
			y+=speed;
			reached = false;
		}
		else if(y>targY)
		{
//			std::cout << "y>targY" << std::endl;
			y-=speed;
			reached = false;
		}

		if(z<targZ)
		{
//			std::cout << "z<targZ" << std::endl;
			z+=speed;
			reached = false;
		}
		else if(z>targZ)
		{
//			std::cout << "z>targZ" << std::endl;
			z-=speed;
			reached = false;
		}

		yarp::os::Bottle& b = worldCommands.prepare();
			b.clear();
			b.addString("world");
			b.addString("set");
			b.addString("sbox");

			b.addInt(1);	//box index
			//position
			b.addDouble(x);
			b.addDouble(y);
			b.addDouble(z);	//30 cm in front of eyes?
		worldCommands.write(true);		//Need to force strict to get smooth movement

		if(irregular)
		{
			time_t current = time(NULL);
			timeTaken=current - startSeconds;
			if(timeTaken>=1)
				yarp::os::Time::delay(1);
			startSeconds = time(NULL);
		}
	}
}


void goToMaxLeft(double speed, bool irregular = false)
{
	std::cout <<"going to left max" <<std::endl;
	mvBox(lMax, y, z, speed, irregular);
}

void goToMaxRight(double speed, bool irregular = false)
{
	std::cout <<"going to right max" <<std::endl;
	mvBox(rMax, y, z, speed, irregular);
}

void goToCentre(double speed, bool irregular = false)
{
//	std::cout<< "going to centre" << std::endl;
	mvBox(c_x, c_y, c_z, speed, irregular);
}

void jiggle(time_t duration)
{
	std::cout<< "jiggling" << std::endl;

	srand( time(0));

	time_t startSeconds;
	startSeconds = time(NULL);
	time_t timeTaken = startSeconds-startSeconds;
	while(timeTaken < duration)
	{
		double randX = (rand()%5)/100.0;
		double randY = 0;//(rand()%5)/100.0;
		double randZ = 0;//(rand()%5)/100.0;

		//shift left
		mvBox(x+randX, y+randY, z+randZ, fast);
		goToCentre(fast);

		randX = -1*(rand()%5)/100.0;
		randY = 0;//-1*(rand()%5)/100.0;
		randZ = 0;//-1*(rand()%5)/100.0;
		//shift right
		mvBox(x+randX, y+randY, z+randZ, fast);
		goToCentre(fast);

		time_t current = time(NULL);
		timeTaken=current - startSeconds;
	}

}

void hide()
{
	mvBox(0.5, y, z, fast);
}


void block(time_t duration, double speed)
{
	goToCentre(speed);
	time_t startSeconds;
	startSeconds = time(NULL);
	time_t timeTaken = startSeconds-startSeconds;
	bool irregular = true;
//	while(timeTaken < duration)
//	{
	goToMaxRight(speed);
	yarp::os::Time::delay(5);
	std::cout<<"start experiment" << std::endl;
	goToMaxLeft(speed);
		goToMaxRight(speed, irregular);
		goToCentre(speed);
		yarp::os::Time::delay(10);
		jiggle(10);
		yarp::os::Time::delay(10);
		goToMaxLeft(speed);
//		goToCentre(speed);


		time_t current = time(NULL);
		timeTaken=current - startSeconds;
//	}
		hide();
}

void delAll()
{
	yarp::os::Bottle& b = worldCommands.prepare();
		b.clear();
		//world mk sbox (three params for size) (three params for pos) (three params for colour)  ------- This creates a static box
		b.addString("world");
		b.addString("del");
		b.addString("all");
	worldCommands.write(true);
}



int main(int argc, char* argv[])
{
	yarp::os::Network yarp;

	worldCommands.open("/BB/worldCmd");

	std::cout << "Please connect /BB/worldCmd to /icubSim/world then press any key"  << std::endl;
	char c;
	std::cin >> c;

	delAll();

	mkBox();

	int option;
	do{
		std::cout << "Select a block: \n" <<
			"\t0. Make block\n"<<
			"\t1. Slow block\n"<<
			"\t2. Fast block\n"<<
			"\t3. Object absent\n"<<
			"\t4. Centre object\n" <<
			"\t5. Jiggle object\n" <<
			"\t6. Exit" << std::endl;

	std::cin >> option;

		switch(option)
		{
			case 0:
				mkBox();
				break;
			case 1:
				block(90, slow);
				break;
			case 2:
				block(60, fast);
				break;
			case 3:
				hide();
				break;
			case 4:
				goToCentre(fast);
				break;
			case 5:
				jiggle(10);
				break;
			case 6:
				std::cout << "exiting" << std::endl;
				break;
			default:
				std::cout << "unknown option" << std::endl;
				break;

		}
	}while(option<6 && option>=0);

	delAll();
	worldCommands.close();

}
