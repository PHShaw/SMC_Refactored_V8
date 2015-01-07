/*
    Definition of the base class for mappings, called CBaseMap.
     
    Copyright (C) 2009  Martin Huelse, 
    msh@aber.ac.uk
    http://aml.somebodyelse.de
    Intelligent Robotics Group
    Department of Computer Science
    Aberystwyth University
    Wales, UK, 
    SY23 3DB

	This file is part of the IRG Mapping Library.

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation version 3 of the License.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "CBaseMap.h"
#include <iostream>
#include <sstream>
#include <math.h>

CBaseMap::CBaseMap(){
	listOfLinks_ = NULL;
	maxNmbLinks_ = 0;
	indexLastLink_ = -1;
	inpDim_ = 0;
	outDim_ = 0;
	attrDim_ = 0;
};

CBaseMap::~CBaseMap(){

	if(maxNmbLinks_ != 0){
		delete [] listOfLinks_;
	};
};

void CBaseMap::init(int inpDim, int outDim, int attrDim, int initSize) 
throw (IMapException){
	if(maxNmbLinks_ != 0){
		string msg("Map is already initalized!\n");
		throw IMapException(msg,-1);
	};
	
	if((inpDim < 1) || (outDim < 1)){
		string msg("Input and/or output space dimension is less than 1.\n");
		throw IMapException(msg,-2);
	};

	if(attrDim < 1){
		string msg("Dimension of the attribute value is less than 1.\n");
		throw IMapException(msg,-2);
	};
	
	if(initSize < 1){
		string msg("Init size is less than 1.\n");
		throw IMapException(msg,-3);
	};
	

	
	try{
		listOfLinks_ = new float[(inpDim + outDim + attrDim)*initSize];
	}catch(bad_alloc e){
		string msg("1: List is full, and unable to get additional storage. No new link added.\n");
		throw(IMapException(msg,-11));			
	}catch(...){
		string msg("Cannot get enought memory to initalize map.\n");
		throw IMapException(msg,-4);
	};
	
	if(listOfLinks_ == NULL){
		string msg("Cannot get enought memory to initalize map.\n");
		throw IMapException(msg,-4);
	};
	
	initSize_ = initSize;
	maxNmbLinks_ = initSize;
	inpDim_ = inpDim;
	outDim_ = outDim;
	attrDim_ = attrDim;
	indexLastLink_ = -1;	
};

void CBaseMap::deinit(){
	if(maxNmbLinks_ != 0){
		delete [] listOfLinks_;		
	};
	maxNmbLinks_ = 0;
	listOfLinks_ = NULL;
	indexLastLink_ = -1;
}

void CBaseMap::clean(){
	indexLastLink_ = -1;
}

void CBaseMap::addLink(float *inpNode, float *outNode) throw (IMapException){
	if(attrDim_ < 1){
		addLink(inpNode,outNode,NULL);
	}else{
		float *attrValues = new float[attrDim_];
		for(int i = 0; i < attrDim_;i++){
			attrValues[i] = 0.0;
		};
		addLink(inpNode,outNode,attrValues);
		delete [] attrValues;
	};
};


void CBaseMap::addLink(float *inpNode, float *outNode, float *linkAttributes)
throw (IMapException){
	if((maxNmbLinks_ == 0) || 
		(listOfLinks_ == NULL)){
			string msg("Map not initialized yet, cannot  store links.\n");
			throw(IMapException(msg,-6));
	};
		
	if((indexLastLink_ + 1) == maxNmbLinks_){
		// list full create new storage	
		float *fullList = listOfLinks_;
		int    oldSize = maxNmbLinks_*(inpDim_+outDim_+attrDim_);
		int    additionalSize = initSize_*(inpDim_ + outDim_ + attrDim_);
		float *newList;
		
		try{
			newList = new float[oldSize + additionalSize];
		}catch(bad_alloc e){
			string msg("1: List is full, and unable to get additional storage. No new link added.\n");
			throw(IMapException(msg,-11));			
		}catch(...){
			string msg("2: List is full, and unable to get additional storage. No new link added.\n");
			throw(IMapException(msg,-11));			
		}
		

		
		// recopy elements	
		reCopy(newList,fullList,(maxNmbLinks_*(inpDim_ + outDim_ + attrDim_)));
		maxNmbLinks_ = maxNmbLinks_ + initSize_;
		listOfLinks_ = newList;
		delete [] fullList;

	};
	
	// add new link
	indexLastLink_++;
	int baseIdx = (outDim_ + inpDim_ + attrDim_)*(indexLastLink_);
	for(int i = 0; i < inpDim_; i++){
		listOfLinks_[baseIdx + i] = inpNode[i]; 
	};
	for(int i = 0; i < outDim_; i++){
		listOfLinks_[baseIdx + inpDim_ + i] = outNode[i];
	};
	for(int i = 0; i < attrDim_; i++){
		listOfLinks_[baseIdx + inpDim_ + outDim_ + i] = linkAttributes[i];
	};
	
}

void CBaseMap::removeLink(int nodeIdx){
	if(nodeIdx < 0) return;
	if(nodeIdx > indexLastLink_) return;
	
	if(nmbOfLinks() == 1){
		clean();
		return;
	}
	
	// swap values of i-th link with 
	// last link in this list
	float *tmpLinkStorage = new float[inpDim_ + outDim_ + attrDim_];
	float *ptrLastLink = &(listOfLinks_[(inpDim_ + outDim_ + attrDim_)*indexLastLink_]);
	float *ptrIthLink = &(listOfLinks_[(inpDim_ + outDim_ + attrDim_)*nodeIdx]);
	
	for(int i = 0; i < (inpDim_ + outDim_ + attrDim_);i++){
		tmpLinkStorage[i] = ptrLastLink[i];	
	};

	for(int i = 0; i < (inpDim_ + outDim_ + attrDim_);i++){
		ptrLastLink[i] =  ptrIthLink[i];	
	};
	for(int i = 0; i < (inpDim_ + outDim_ + attrDim_);i++){
		ptrIthLink[i] = tmpLinkStorage[i];	
	};

	
	// remove last link 
	indexLastLink_ = indexLastLink_ - 1;
};



void CBaseMap::reCopy(float *destination, float *source, int nmb){
	for(int i = 0; i < nmb; i++){
		destination[i] = source[i];
	}
}

int  CBaseMap::nmbOfLinks(){
	return (indexLastLink_ + 1);  
}









float* CBaseMap::ptrLinkByIndex(int i){
	if(i < 0) return NULL;
	if(i > indexLastLink_) return NULL;
	return (&(listOfLinks_[i*(inpDim_ + outDim_ + attrDim_)]));
}





/**
 * 
 * 
 * 
 */
int CBaseMap::getInpDim(){
	if(listOfLinks_ == NULL){
		return(0);
	}else{
		return(inpDim_);
	};
}

/**
 * 
 * 
 * 
 */
int CBaseMap::getOutDim(){
	if(listOfLinks_ == NULL){
		return (0);
	}else{
		return(outDim_);
	};
}

int CBaseMap::getAttrDim(){
	if(listOfLinks_ == NULL){
		return (-1);
	}else{
		return(attrDim_);
	};
}


/*
 * 
 * IMapException implementation
 * 
 * 
 */
IMapException::IMapException(string s, int code){msg_ = string(s);code_ = code;};
IMapException::~IMapException(){};
string IMapException::getMsg(){return msg_;};
string IMapException::getSupport(){return support_;};
int IMapException::getCode(){return code_;};


