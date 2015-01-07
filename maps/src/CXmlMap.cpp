/*
    Definition of the xml-write/read class for mappings, called CXmlMap.
     
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
#include "CXmlMap.h"
#include <iostream>
#include <stdexcept>


CXmlMap::CXmlMap() : CBaseMap(){
	comments_.clear();
};


CXmlMap::~CXmlMap(){
};


void CXmlMap::readXmlFromString(char *xmlStr) throw (IMapException){
	try {
		XMLPlatformUtils::Initialize();
	}catch (const XMLException& toCatch) {
		char* message = XMLString::transcode(toCatch.getMessage());
		cout << "Error during initialization! :\n";
		cout << "Exception message is: \n"
		<< message << "\n";
		XMLString::release(&message);
		return;
	};

	SAX2XMLReader* parser;
	CMapSAX2Handler* mapHandler;
	MemBufInputSource* memBufIS;

	parser = XMLReaderFactory::createXMLReader();
	parser->setFeature(XMLUni::fgSAX2CoreValidation, true);   
	parser->setFeature(XMLUni::fgSAX2CoreNameSpaces, true);   // optional
	
	mapHandler = new CMapSAX2Handler();
	parser->setContentHandler(mapHandler);
	parser->setErrorHandler(mapHandler);
	
	mapHandler->init(this);
		
	
	// deinitialize the current state of the map
	deinit();

	
	
	static const char*  gMemBufId = "CXmlMap-String-XML-parsing";
	
	memBufIS = new MemBufInputSource
	(
		(const XMLByte*)xmlStr
	       	, strlen(xmlStr)
	        , gMemBufId
	        , false
	    );

	
	try {
			parser->parse(* memBufIS);
	}catch (const XMLException& toCatch) {
		char* message = XMLString::transcode(toCatch.getMessage());
		string msg = string("Exception message is: \n") + 
			string(message) + string("\n");
			XMLString::release(&message);
		throw IMapException(msg,-1);
	}catch (const SAXParseException& toCatch) {
		char* message = XMLString::transcode(toCatch.getMessage());
		string msg = string("Exception message is: \n") + 
			string(message) + string("\n");
		XMLString::release(&message);
		throw IMapException(msg,-1);
	}catch(runtime_error &e){
		string msg = string("check structure and semantics of values in xml-strin: ") +
			xmlStr  + string("\n");
		throw IMapException(msg,-1);
	}catch (...) {
		string msg = string("Unexpected Exception \n");
		throw IMapException(msg,-1);
	};

    delete parser;
    delete mapHandler;
    delete memBufIS;

    // And call the termination method                                                                                                                       
    XMLPlatformUtils::Terminate();

	
};


void CXmlMap::readXmlStructure(char *filename, bool readAttributes) throw (IMapException){
	try {
		XMLPlatformUtils::Initialize();
	}catch (const XMLException& toCatch) {
		char* message = XMLString::transcode(toCatch.getMessage());
		cout << "Error during initialization! :\n";
		cout << "Exception message is: \n"
		<< message << "\n";
		XMLString::release(&message);
		return;
	};


	SAX2XMLReader* parser = XMLReaderFactory::createXMLReader();
	parser->setFeature(XMLUni::fgSAX2CoreValidation, true);   
	parser->setFeature(XMLUni::fgSAX2CoreNameSpaces, true);   // optional
	
	CMapSAX2Handler* mapHandler = new CMapSAX2Handler();
	parser->setContentHandler(mapHandler);
	parser->setErrorHandler(mapHandler);
	
	mapHandler->init(this);
	
	mapHandler->readLinkAttributes(readAttributes);
	
	// deinitialize the current state of the map
	deinit();
	
	try {
			parser->parse(filename);
	}catch (const XMLException& toCatch) {
		char* message = XMLString::transcode(toCatch.getMessage());
		string msg = string("Exception message is: \n") + 
			string(message) + string("\n");
			XMLString::release(&message);
		throw IMapException(msg,-1);
	}catch (const SAXParseException& toCatch) {
		char* message = XMLString::transcode(toCatch.getMessage());
		string msg = string("Exception message is: \n") + 
			string(message) + string("\n");
		XMLString::release(&message);
		throw IMapException(msg,-1);
	}catch(runtime_error &e){
		string msg = string("check structure and semantics of values in xml-file: ") +
			string(filename)  + string("\n");
		throw IMapException(msg,-1);
	}catch (...) {
		string msg = string("Unexpected Exception while xml-parsing\n");
		throw IMapException(msg,-1);
	};
	
	

    delete mapHandler;
    delete parser;
    XMLPlatformUtils::Terminate();

	
};

void CXmlMap::readXmlFileWithoutAttributes(char *filename) throw (IMapException){
	try{
		// read xml file data without the link attribute values
		readXmlStructure(filename,false);
	}catch(IMapException e){
		string msg = e.getMsg() + string("...while reading xml-file without attributes.\n");
		throw IMapException(msg,-1);
	}catch(...){
		string msg = string("Unexpected Exception while reading xml-file without attributes.\n");
		throw IMapException(msg,-1);		
	};
	return;
};

void CXmlMap::readXmlFile(char *filename) throw (IMapException){
	try{
		// read xml file data including the link attribute values
		readXmlStructure(filename,true);
	}catch(IMapException e){
		string msg = e.getMsg() + string("...while reading xml-file without attributes.\n");
		throw IMapException(msg,-1);
	}catch(...){
		string msg = string("Unexpected Exception while reading xml-file with attributes.\n");
		throw IMapException(msg,-1);		
	};
	return;
};

char *CXmlMap::writeXmlToString() throw(IMapException){
	char *xmlStrDTD = outDTD();
	char *xmlStrMap = outMap();
	int size = strlen(xmlStrDTD) + strlen(xmlStrMap);
	char *xmlStr = new char[size + 1];
	strcpy(xmlStr,xmlStrDTD);
	strcat(xmlStr,xmlStrMap);
	delete [] xmlStrDTD;
	delete [] xmlStrMap;
	return (xmlStr);
};

void CXmlMap::writeXmlFile(char *filename) throw(IMapException){
	// open file
	ofstream *xmlOutStream;
	try{
		xmlOutStream = new ofstream();
	}catch(bad_alloc e){
		return;
	}catch(...){
		return;
	};
	
	xmlOutStream->open(filename);
	if (!xmlOutStream->is_open()){
		string msg = string("Unable to open xml file: ") +
			string(filename);
		throw (IMapException(msg,-1));
	};

	// write DTD
	outDTD(xmlOutStream);
	
	// write data
	outMap(xmlOutStream);
	
	//close file;
	xmlOutStream->close();
	
	delete xmlOutStream;
	xmlOutStream = NULL;
}

char *CXmlMap::outDTD(){
	int size = 30*40;
	char *xmlStr = new char[size];
	xmlStr[0]='\0';
	
	strcat(xmlStr,"<?xml version=\"1.0\" encoding=\"iso-8859-1\"?>\n");
	strcat(xmlStr,"<!DOCTYPE Map [\n");
	strcat(xmlStr,"<!ELEMENT Map (Comment?,Link*)>\n");
	strcat(xmlStr,"<!ATTLIST Map\n");
	strcat(xmlStr,"          InpDim       CDATA #REQUIRED\n");
	strcat(xmlStr,"          OutDim       CDATA #REQUIRED\n");
	strcat(xmlStr,"          LinkAttrDim  CDATA #REQUIRED>\n");
	strcat(xmlStr,"<!ELEMENT Comment (#PCDATA)>\n");
	strcat(xmlStr,"<!ELEMENT Link (InputNode+,OutputNode+,LinkAttr+)>\n");
	
	strcat(xmlStr,"<!ELEMENT InputNode (Component+)>\n");
	strcat(xmlStr,"<!ATTLIST InputNode\n");
	strcat(xmlStr,"    Dim         CDATA #REQUIRED>\n");
	
	strcat(xmlStr,"<!ELEMENT OutputNode (Component+)>\n");    
	strcat(xmlStr,"<!ATTLIST OutputNode\n");
	strcat(xmlStr,"    Dim         CDATA #REQUIRED>\n");
	
	strcat(xmlStr,"<!ELEMENT LinkAttr (LinkAttrValue+)>\n");
	strcat(xmlStr,"<!ATTLIST LinkAttr\n");
	strcat(xmlStr,"    Dim         CDATA #REQUIRED>\n");

	strcat(xmlStr,"<!ELEMENT Component EMPTY>\n");    
	strcat(xmlStr,"<!ATTLIST Component\n");
	strcat(xmlStr,"    Value         CDATA #REQUIRED\n");
	strcat(xmlStr,"    NodeIdx       CDATA #REQUIRED>\n");
	
	strcat(xmlStr,"<!ELEMENT LinkAttrValue EMPTY>\n");    
	strcat(xmlStr,"<!ATTLIST LinkAttrValue\n");
	strcat(xmlStr,"    LinkValue     CDATA #REQUIRED\n");
	strcat(xmlStr,"    Idx           CDATA #REQUIRED>\n");
	
	strcat(xmlStr,"]>\n\0");
	
	int  actSize = strlen(xmlStr);
	char *xmlStrSmall = new char[actSize+1];
	strncpy(xmlStrSmall,xmlStr,actSize);
	xmlStrSmall[actSize]='\0';
	delete [] xmlStr;
	return (xmlStrSmall);
};

void CXmlMap::outDTD(ofstream *fout){
	*fout << outDTD();
};

void CXmlMap::outComment(ofstream *fout){
	*fout << "\t<Comment>\n";
	*fout << comments_ << endl;
	*fout << "\t</Comment>\n";	;	;
}


char *CXmlMap::outComment(){
	int size = 40 + comments_.length();
	char *xmlStr = new char[size];
	char *subStr =new char[comments_.length() + 1];
	
	strcpy(xmlStr,"\t<Comment>\n");
	comments_.copy(subStr,comments_.length());
	subStr[comments_.length()]='\0';
	strcat(xmlStr,subStr);
	strcat(xmlStr,"\n\t</Comment>\n\0");
	
	char *xmlStrSmall = new char[strlen(xmlStr)+1];
	strcpy(xmlStrSmall,xmlStr);
	xmlStrSmall[strlen(xmlStr)]='\0';
	delete [] subStr;
	delete [] xmlStr;
	return(xmlStrSmall);
};


void CXmlMap::outMap(ofstream *fout){
	*fout << "<Map InpDim=\"" << inpDim_ << "\"";
	*fout <<     " OutDim=\"" << outDim_  << "\"";
	*fout <<     " LinkAttrDim=\"" << attrDim_  << "\">\n";
	
	outComment(fout);
	
	for (int i = 0; i < (indexLastLink_+1); i++){
		outLink(fout,i);
	};
	
	*fout << "</Map>\n";
}


char *CXmlMap::outMap(){
	char *xmlStrOpen = new char[255];
	char *xmlStrClose = new char[16];
	sprintf(xmlStrOpen,
			"<Map InpDim=\"%d\" OutDim=\"%d\" LinkAttrDim=\"%d\">\n\0",inpDim_,outDim_,attrDim_);
	sprintf(xmlStrClose,"</Map>\n\0");
	char *commentStr = outComment();
	
	if(nmbOfLinks() < 1){
		int size = strlen(xmlStrOpen) + strlen(commentStr) + strlen(xmlStrClose);
		char *xmlStrSmall = new char[size+1];
		strcpy(xmlStrSmall,xmlStrOpen);
		strcat(xmlStrSmall,commentStr);
		strcat(xmlStrSmall,xmlStrClose);
		xmlStrSmall[size]='\0';
		delete [] xmlStrOpen;
		delete [] xmlStrClose;
		delete [] commentStr;
		return (xmlStrSmall);
	};
	
	char *firstLink = outLink(0);
	int estLinkSize = strlen(firstLink);
	int size = (int)( ((float)  estLinkSize) * ((float) nmbOfLinks()) * 1.2 );
	
	char *xmlStrAllLinks;
	try{
		xmlStrAllLinks = new char[size];
	}catch(...){
		int size = strlen(xmlStrOpen) + strlen(commentStr) + strlen(xmlStrClose);
		char *xmlStrSmall = new char[size+1];
		strcpy(xmlStrSmall,xmlStrOpen);
		strcat(xmlStrSmall,commentStr);
		strcat(xmlStrSmall,xmlStrClose);
		xmlStrSmall[size]='\0';
		delete [] xmlStrOpen;
		delete [] xmlStrClose;
		delete [] commentStr;
		delete [] firstLink;
		return (xmlStrSmall);		
	};
	
	char *xmlSubStr;
	strcpy(xmlStrAllLinks,firstLink);
	delete [] firstLink;
	for (int i = 1; i < (indexLastLink_+1); i++){
		xmlSubStr = outLink(i);
		strcat(xmlStrAllLinks,xmlSubStr);
		delete [] xmlSubStr;
	};
	
	
	size = strlen(xmlStrOpen) + strlen(commentStr) + 
		strlen(xmlStrAllLinks) + strlen(xmlStrClose);
	char *xmlStrSmall = new char[size+1];
	strcpy(xmlStrSmall,xmlStrOpen);
	strcat(xmlStrSmall,commentStr);
	strcat(xmlStrSmall,xmlStrAllLinks);
	strcat(xmlStrSmall,xmlStrClose);
	xmlStrSmall[size]='\0';
	delete [] xmlStrOpen;
	delete [] xmlStrClose;
	delete [] commentStr;
	delete [] xmlStrAllLinks;
	return (xmlStrSmall);
	
}



void CXmlMap::outLink(ofstream *fout, int idx){
	*fout << "\t<Link>\n";
	
	//float linkValues[inpDim_ + outDim_ + attrDim_];
	// 	getLinkByIdx(linkValues, idx);
	float *linkValues = ptrLinkByIndex(idx); 

	outInpNode(fout, linkValues, inpDim_);
	outOutNode(fout, &(linkValues[inpDim_]), outDim_);
	outAttrValues(fout,&(linkValues[inpDim_ + outDim_]),attrDim_);
	
	*fout << "\t</Link>\n";	
}

char *CXmlMap::outLink(int idx){
	float *ptrInpNode;
	float *ptrOutNode;
	float *ptrAttrValues;	
	ptrInpNode    =  ptrLinkByIndex(idx);
	if(ptrInpNode == NULL) return NULL;
	ptrOutNode    = &(ptrInpNode[inpDim_]);
	ptrAttrValues = &(ptrInpNode[inpDim_ + outDim_]);
	
	char *xmlStrInpNode = outInpNode(ptrInpNode, inpDim_);
	char *xmlStrOutNode = outOutNode(ptrOutNode, outDim_);
	char *xmlStrAtrVal  = outAttrValues(ptrAttrValues, attrDim_);

	int size = strlen(xmlStrInpNode) + 
		strlen(xmlStrOutNode) + 
		strlen(xmlStrAtrVal);
	char *xmlStr = new char[size + 60];
	
	strcpy(xmlStr,"\t<Link>\n");
	strncat(xmlStr,xmlStrInpNode,strlen(xmlStrInpNode));
	strncat(xmlStr,xmlStrOutNode,strlen(xmlStrOutNode));
	strncat(xmlStr,xmlStrAtrVal,strlen(xmlStrAtrVal));
	strcat(xmlStr,"\t</Link>\n");	
	
	char *xmlStrSmall = new char[strlen(xmlStr)+1];
	strncpy(xmlStrSmall,xmlStr,strlen(xmlStr));
	xmlStrSmall[strlen(xmlStr)]='\0';
	delete [] xmlStr;
	delete [] xmlStrInpNode;
	delete [] xmlStrOutNode;
	delete [] xmlStrAtrVal;
	return (xmlStrSmall);
};


void CXmlMap::outInpNode(ofstream *fout, float *values, int dim){
	*fout << "\t\t<InputNode Dim=\"" << dim <<    "\">\n";

	for(int i = 0; i < dim; i++){
		*fout << "\t\t\t<Component Value=\"" <<
			values[i] << "\" NodeIdx=\"" << i << "\" />\n";
	};
	
	*fout << "\t\t</InputNode>\n";	;
}

char *CXmlMap::outInpNode(float *values, int dim){
	int subStrSize = 124;
	char *xmlSubStr = new char[subStrSize];
	char *xmlStr    = new char[subStrSize*(dim+2)];
	
	xmlSubStr[0]='\0';
	sprintf(xmlSubStr,"\t\t<InputNode Dim=\"%d\">\n",dim);
	strcpy(xmlStr,xmlSubStr);
	///
	for(int i = 0; i < dim; i++){
		xmlSubStr[0]='\0';
		sprintf(xmlSubStr,"\t\t\t<Component Value=\"%f\" NodeIdx=\"%d\" />\n",values[i],i);
		strcat(xmlStr,xmlSubStr);
	};
	///
	strcat(xmlStr,"\t\t</InputNode>\n\0");
	
	char *xmlStrSmall = new char[strlen(xmlStr)+1];
	strncpy(xmlStrSmall,xmlStr,strlen(xmlStr));
	xmlStrSmall[strlen(xmlStr)]='\0';
	
	delete [] xmlSubStr;
	delete [] xmlStr;
	return (xmlStrSmall);
};

void CXmlMap::outOutNode(ofstream *fout, float *values, int dim){
	*fout << "\t\t<OutputNode Dim=\"" << dim <<    "\">\n";

	for(int i = 0; i < dim; i++){
		*fout << "\t\t\t<Component Value=\"" <<
			values[i] << "\" NodeIdx=\"" << i << "\" />\n";
	};
	
	*fout << "\t\t</OutputNode>\n";	
}

char *CXmlMap::outOutNode(float *values, int dim){
	int subStrSize = 124;
	char *xmlSubStr = new char[subStrSize];
	char *xmlStr    = new char[subStrSize*(dim+2)];
	
	xmlSubStr[0]='\0';
	sprintf(xmlSubStr,"\t\t<OutputNode Dim=\"%d\">\n",dim);
	strcpy(xmlStr,xmlSubStr);
	///
	for(int i = 0; i < dim; i++){
		xmlSubStr[0]='\0';
		sprintf(xmlSubStr,"\t\t\t<Component Value=\"%f\" NodeIdx=\"%d\" />\n",values[i],i);
		strcat(xmlStr,xmlSubStr);
	};
	///
	strcat(xmlStr,"\t\t</OutputNode>\n\0");
	
	char *xmlStrSmall = new char[strlen(xmlStr)+1];
	strncpy(xmlStrSmall,xmlStr,strlen(xmlStr));
	xmlStrSmall[strlen(xmlStr)]='\0';
	
	delete [] xmlSubStr;
	delete [] xmlStr;
	return (xmlStrSmall);
};

void CXmlMap::outAttrValues(ofstream *fout, float *values, int dim){
	*fout << "\t\t<LinkAttr Dim=\"" << dim <<    "\">\n";

	for(int i = 0; i < dim; i++){
		*fout << "\t\t\t<LinkAttrValue LinkValue=\"" <<
			values[i] << "\" Idx=\"" << i << "\" />\n";
	};
	
	*fout << "\t\t</LinkAttr>\n";	
}


char *CXmlMap::outAttrValues(float *values, int dim){
	int subStrSize = 124;
	char *xmlSubStr = new char[subStrSize];
	char *xmlStr    = new char[subStrSize*(dim+2)];
	
	xmlSubStr[0]='\0';
	sprintf(xmlSubStr,"\t\t<LinkAttr Dim=\"%d\">\n",dim);
	strcpy(xmlStr,xmlSubStr);
	///
	for(int i = 0; i < dim; i++){
		xmlSubStr[0]='\0';
		sprintf(xmlSubStr,"\t\t\t<LinkAttrValue LinkValue=\"%f\" Idx=\"%d\" />\n",values[i],i);
		strcat(xmlStr,xmlSubStr);
	};
	///
	strcat(xmlStr,"\t\t</LinkAttr>\n\0");
	
	char *xmlStrSmall = new char[strlen(xmlStr)+1];
	strncpy(xmlStrSmall,xmlStr,strlen(xmlStr));
	xmlStrSmall[strlen(xmlStr)]='\0';
	
	delete [] xmlSubStr;
	delete [] xmlStr;
	return (xmlStrSmall);
};


void CXmlMap::writeComments(string s){
	cout << "Received string: " << s << endl;
	comments_ = string(s);
};

string CXmlMap::getComments(){
	return (comments_);
};



/*
 * 
 * 
 * Handler implementation
 * 
 * 
 * 
 */


CMapSAX2Handler::CMapSAX2Handler(){
	   	xmlMap_ = XMLString::transcode("Map");
	   	xmlMapInpDimAttr_ = XMLString::transcode("InpDim");
	   	xmlMapOutDimAttr_ = XMLString::transcode("OutDim");
	   	xmlMapLinkAttrDimAttr_ = XMLString::transcode("LinkAttrDim");
	   	xmlComment_ = XMLString::transcode("Comment");
	   	xmlLink_ = XMLString::transcode("Link");
	   	xmlInpNode_ = XMLString::transcode("InputNode");
	   	xmlOutNode_ = XMLString::transcode("OutputNode");
	   	xmlComponent_ = XMLString::transcode("Component");
	   	xmlCompAttrValue_ = XMLString::transcode("Value");
	   	xmlCompAttrIdx_ = XMLString::transcode("NodeIdx");
	   	
		xmlLinkAttr_ = XMLString::transcode("LinkAttr");
		xmlLinkAttrComponent_ = XMLString::transcode("LinkAttrValue");
		xmlLinkAttrCompValue_ = XMLString::transcode("LinkValue");
		xmlLinkAttrCompIdx_ = XMLString::transcode("Idx");

	
		readInpNode_ = false;
		readOutNode_ = false;
		readNodeAttr_ = false;
		firstLink_ = true;
		inpNodeValues_  = NULL;
		outNodeValues_  = NULL;
		nodeAttrValues_ = NULL;
}
CMapSAX2Handler::~CMapSAX2Handler(){
		XMLString::release(&xmlMap_);
		XMLString::release(&xmlComment_);
		XMLString::release(&xmlMapInpDimAttr_);
		XMLString::release(&xmlMapOutDimAttr_);
		XMLString::release(&xmlMapLinkAttrDimAttr_);
		XMLString::release(&xmlLink_);
		XMLString::release(&xmlInpNode_);
		XMLString::release(&xmlOutNode_);
		XMLString::release(&xmlComponent_);
		XMLString::release(&xmlCompAttrValue_);
		XMLString::release(&xmlCompAttrIdx_);
   	
		XMLString::release(&xmlLinkAttr_);
		XMLString::release(&xmlLinkAttrComponent_);
		XMLString::release(&xmlLinkAttrCompValue_);
		XMLString::release(&xmlLinkAttrCompIdx_);
		
		
		if(inpNodeValues_ != NULL)  delete [] inpNodeValues_;
		if(outNodeValues_ != NULL)  delete [] outNodeValues_;  
		if(nodeAttrValues_ != NULL) delete [] nodeAttrValues_; 
};

void CMapSAX2Handler::readLinkAttributes(bool b){
	readLinkAttributes_ = b;
}


//void CMapSAX2Handler::characters( const XMLCh* const ch, const unsigned int length ){
void CMapSAX2Handler::characters(const XMLCh* const    chars, const   XMLSize_t    length){
	// the comment element is the only place where characters are processes, 
	// hence we put them into the comment field of our xml-structure
	//string s = map_->getComments();

//	cout << "Processing character data " << chars << endl;
	map_->writeComments(XMLString::transcode(chars));
};

//void CMapSAX2Handler::characters(const XMLCh* const ch, const unsigned int length ){
//	map_->writeComments(XMLString::transcode(ch));
//};



void CMapSAX2Handler::startElement(const   XMLCh* const    uri,const   XMLCh* const    localname, const   XMLCh* const    qName, const   Attributes&     attrs){
		char * ptrQname = XMLString::transcode(qName);
		
		char * ptrCompareName;
		
		
		ptrCompareName = XMLString::transcode(xmlMap_);
	    if(!XMLString::compareString(ptrQname,ptrCompareName)){
	    	char * chInpDim  = XMLString::transcode(attrs.getValue(xmlMapInpDimAttr_));
	    	char * chOutDim  = XMLString::transcode(attrs.getValue(xmlMapOutDimAttr_));
	    	inpDim_ = atoi(chInpDim);
	    	outDim_ = atoi(chOutDim);
	    	delete [] chInpDim;
	    	delete [] chOutDim;
	    	
	    	char * chNodeAttrDim;
	    	if(readLinkAttributes_){
	    		chNodeAttrDim = XMLString::transcode(attrs.getValue(xmlMapLinkAttrDimAttr_));
	    		attrDim_ = atoi(chNodeAttrDim);
	    	}else{
	    		chNodeAttrDim = XMLString::transcode(attrs.getValue(xmlMapLinkAttrDimAttr_));
	    		attrDim_ = 1;
	    	};
	    	delete [] chNodeAttrDim;
	    	
	    	
	    	// chech boundaries
	    	if((inpDim_ < 1) || (outDim_ < 1) || (attrDim_ < 1)){
	    		cout << "Map and attribute dimensions in xml file less than 1.\n"; cout.flush();
	    		throw (runtime_error("Map and attribute dimensions in xml file less than 1.\n"));
	    	}   	    	
	    	// create storage
	    	inpNodeValues_ = new float[inpDim_];
	    	outNodeValues_ = new float[outDim_];
	    	nodeAttrValues_ = new float[attrDim_];
	    	
	    	map_->init(inpDim_,outDim_,attrDim_,1000);	
	    	
	    };
	    delete [] ptrCompareName;

		ptrCompareName = XMLString::transcode(xmlComment_);
	    if(!XMLString::compareString(ptrQname,ptrCompareName)){
	    	// do nothing, if "comment" is the only element where characters are processes
	    	;
//	    	cout << "start ignorning something" << endl;
	    };
	    delete [] ptrCompareName;

		ptrCompareName = XMLString::transcode(xmlInpNode_);
	    if(!XMLString::compareString(ptrQname,ptrCompareName)){
	    	inpNodeReadMode_ = true;
	    	readInpNode_ = true;
	    };
	    delete [] ptrCompareName;
	    
		ptrCompareName = XMLString::transcode(xmlOutNode_);
	    if(!XMLString::compareString(ptrQname,ptrCompareName)){	    	
	    	inpNodeReadMode_ = false;
	    	readOutNode_  = true;
	    };
	    delete [] ptrCompareName;

		ptrCompareName = XMLString::transcode(xmlLinkAttr_);
	    if(!XMLString::compareString(ptrQname,ptrCompareName)){
	    	if(readLinkAttributes_){
	    		inpNodeReadMode_ = false;
	    		outNodeReadMode_  = false;
	    		readNodeAttr_ = true;
	    	};

	    };
	    delete [] ptrCompareName;
	    
		ptrCompareName = XMLString::transcode(xmlComponent_);
	    if(!XMLString::compareString(ptrQname,ptrCompareName)){
	    	int index;
	    	float value;
	    	// get attribute value
	    	char * chValue = XMLString::transcode(attrs.getValue(xmlCompAttrValue_));
	    	char * chIdx = XMLString::transcode(attrs.getValue(xmlCompAttrIdx_));
	    	index = atoi(chIdx);
	    	value = atof(chValue);
	    	delete [] chValue;
	    	delete [] chIdx;
	    	
	    	if(inpNodeReadMode_){
	    		if(index >= inpDim_){
		    		cout << "wrong INPUT NODE index in xml file == " << 
		    			index << " of max. " << inpDim_ << endl; cout.flush();
	    			throw (runtime_error("wrong index in xml file.\n"));
	    		};
	    		inpNodeValues_[index] = value;
	    	}else{
	    		if(index >= outDim_){
		    		cout << "wrong OUTPUT NODE index in xml file.\n"; cout.flush();
	    			throw (runtime_error("wrong index in xml file.\n"));
	    		};
	    		outNodeValues_[index] = value;
	    	};
	    };
	    delete [] ptrCompareName;	    
	    
	    if(readLinkAttributes_){
	    	ptrCompareName = XMLString::transcode(xmlLinkAttrComponent_);
	    	if(!XMLString::compareString(ptrQname,ptrCompareName) ){
	    		int index;
	    		float value;
	    		// get attribute value
	    		char * chValue = XMLString::transcode(attrs.getValue(xmlLinkAttrCompValue_));
	    		char * chIdx = XMLString::transcode(attrs.getValue(xmlLinkAttrCompIdx_));
	    	
	    		if(readLinkAttributes_){
	    			index = atoi(chIdx);
	    			value = atof(chValue);
	    		}else{
	    			index = 0;
	    			value = 0.0;
	    		}
	    		delete [] chValue;
	    		delete [] chIdx;
	    	
	    		if(index >= attrDim_){
	    			cout << "wrong ATTRIBUTE index in xml file.\n"; cout.flush();
	    			throw (runtime_error("wrong index in xml file.\n"));
	    		};
	    		nodeAttrValues_[index] = value;
	    	};
	    	delete [] ptrCompareName;
	    };
	    
		delete [] ptrQname;
};


void CMapSAX2Handler::endElement(const XMLCh* const uri, const XMLCh* const localname, const XMLCh* const qName){
	char * ptrQname = XMLString::transcode(qName);
	char * ptrCompareName;

	ptrCompareName = XMLString::transcode(xmlComment_);
    if(!XMLString::compareString(ptrQname,ptrCompareName)){
    	// do nothing, if comment is the only element where characters are processes
    	;
//    	cout << "end ignoring something" << endl;
    };

	ptrCompareName = XMLString::transcode(xmlLink_);
    if(!XMLString::compareString(ptrQname,ptrCompareName)){
    	
    	if(readLinkAttributes_){
    		if(readOutNode_ && readInpNode_ && readNodeAttr_){
    			// put data into map    		 
    			map_->addLink(inpNodeValues_,outNodeValues_,nodeAttrValues_);
    		
    			// ready for new data
    			readOutNode_ = false;
    			readInpNode_ = false;
    			readNodeAttr_ = false;
    		}else{
    			cout << " New link detected but haven't read the old link completely.\n";
    			cout.flush();
    			throw (runtime_error(" New link detected but haven't read the old link completely.\n"));
    		};
    	}else{
    		if(readOutNode_ && readInpNode_){
    			// put data into map
    			float f = 0.0;
    			map_->addLink(inpNodeValues_,outNodeValues_,&f);
    		
    			// ready for new data
    			readOutNode_ = false;
    			readInpNode_ = false;
    			readNodeAttr_ = false;
    		}else{
    			cout << " New link detected but haven't read the old link completely.\n";
    			cout.flush();
    			throw (runtime_error(" New link detected but haven't read the old link completely.\n"));
    		};
    		
    	};
    };
    
    delete [] ptrQname;
    delete [] ptrCompareName;
};

void CMapSAX2Handler::fatalError(const SAXParseException& exception){
	string msg = string("Fatal Error: ") + 
		string(XMLString::transcode(exception.getMessage())) + 
		string("\n");
	throw (IMapException(msg,-1));
};

void  CMapSAX2Handler::init(CXmlMap *map){
	map_ = map;
	readLinkAttributes_ = true;
};

