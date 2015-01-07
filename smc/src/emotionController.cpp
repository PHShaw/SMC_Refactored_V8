/*
 * emotionController.cpp
 *
 *	Remember to start the emotion yarpdev thing first!
 *
 *
 *  Created on: 6 Dec 2011
 *      Author: icub
 */
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Time.h>
#include <yarp/os/Network.h>

#include <iostream>

using namespace std;
using namespace yarp::os;

enum emote {neu,hap1,hap2,sad,sur,ang,evi,shy,cun};

class emotionController{
	public:
		bool initYarp();
		string hexMath(int dec);
		void write(string str);
//		void setEmote(emote e);
		void setEmote(int i);
		void read();
	private:

		BufferedPort<Bottle> portOut;
		BufferedPort<Bottle> portIn;
		Bottle* data;




};
const int NumEmotes = 9;
//Values specified in decimal
const int emotions[NumEmotes][3] = {
	// leb	reb	mou
	{	2,	2,	8},		//neu
	{	2,	2,	11},	//hap1
	{	8,	8,	11},	//hap2
	{	2,	2,	56},	//sad
	{	4,	4,	22},	//sur
	{	1,	1,	56},	//ang
	{	1,	1,	11},	//evi
	{	4,	4,	56},	//shy
	{	4,	1,	11}		//cun
};



//		   leb reb mou eli
//E1   neu 02h 02h 08h 38h
//E2   ta1 **h **h 08h **h
//E3   ta2 **h **h 16h **h
//E4   hap 02h 02h 0Bh 70h
//E5   sad 02h 02h 38h 38h
//E6   sur 08h 08h 16h 38h
//E7   evi 01h 01h 38h 70h
//E8   ang 01h 01h 0Bh 70h
//E9   shy 04h 04h 38h 70h
//E10  cun 04h 01h 38h 38h


bool emotionController::initYarp()
{
	Network yarp;

	portOut.open("/emote/face");
	portIn.open("/emote/emotion");

	yarp.connect("/emote/face", "/icub/face/raw/in");
}




int main()
{
	emotionController* em = new emotionController();
	em->initYarp();


	em->read();


//	for(int i=0; i<64; i++)
//	{
//		write("M"+hexMath(i));
//		write("L"+hexMath(i/4));
//		write("R"+hexMath((63-i)/4));
//
//
//		Time::delay(0.3);
//
//	}


//	for(int i=0; i<NumEmotes; i++)
//	{
//		setEmote(static_cast<emote>(i));
//		Time::delay(3);
//	}

}

//void emotionController::setEmote(emote e)
//{
//	setEmote((int)e);
//}

void emotionController::setEmote(int e)
{
	if(e<NumEmotes && e>=0)
	{
		write("L"+hexMath(emotions[e][0]));
		write("R"+hexMath(emotions[e][1]));
		write("M"+hexMath(emotions[e][2]));
	}
}


void emotionController::write(string str)
{
	Bottle& out = portOut.prepare();
	out.clear();
	out.addString(str.c_str());
	portOut.write(true);
	Time::delay(0.001);
}


void emotionController::read()
{
	while(true)
	{
		data=portIn.read(true);	//wait until a message is received
		int i = data->get(0).asInt();
		cout << "received int " << i << endl;
		setEmote(i);
	}
}



string emotionController::hexMath(int dec)
{
	string hex = "";
	int tens = (int)(dec/16);
	int units = dec%16;

	if(tens<10) hex += (tens + 48);
	else
	{
		switch(tens)
		{
			case 10:
				hex +='A';
				break;

			case 11:
				hex +='B';
				break;

			case 12:
				hex +='C';
				break;

			case 13:
				hex +='D';
				break;

			case 14:
				hex +='E';
				break;

			case 15:
				hex +='F';
				break;

			default:
				hex +='0';
				break;
		}
	}
	if(units<10) hex += (units + 48);
	else
	{
		switch(units)
		{
			case 10:
				hex+='A';
				break;

			case 11:
				hex+='B';
				break;

			case 12:
				hex+='C';
				break;

			case 13:
				hex+='D';
				break;

			case 14:
				hex+='E';
				break;

			case 15:
				hex+='F';
				break;

			default:
				hex+='0';
				break;
		}
	}

	return hex;

}
