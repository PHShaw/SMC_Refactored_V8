/*
 * Gaze_IO.h
 *
 *  Created on: 4 Apr 2011
 *      Author: icub
 */

#ifndef GAZE_IO_H_
#define GAZE_IO_H_

#include <string>
#include <vector>

#include "CXmlMap.h"
#include "GazeMap.h"


class Gaze_IO
{
public:
	Gaze_IO();
	virtual ~Gaze_IO();

	bool saveMappingToXML(GazeMap *gm, string filename);
	GazeMap* loadMappingFromXML(string filename);

};

#endif /* GAZE_IO_H_ */
