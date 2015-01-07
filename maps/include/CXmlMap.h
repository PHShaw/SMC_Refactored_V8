/*
    Declarations of the xml-write/read class for mappings, called CXmlMap.
     
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

#ifndef  AU_DCS_REVERB_SW_MAPS_CXMLMAP_H_
#define  AU_DCS_REVERB_SW_MAPS_CXMLMAP_H_

#include "CBaseMap.h"
#include <iostream>
#include <fstream>
#include <xercesc/sax2/SAX2XMLReader.hpp>
#include <xercesc/sax2/XMLReaderFactory.hpp>
#include <xercesc/sax2/DefaultHandler.hpp>
#include <xercesc/sax2/Attributes.hpp>
#include <xercesc/util/XMLString.hpp>
#include <xercesc/framework/MemBufInputSource.hpp>
XERCES_CPP_NAMESPACE_USE

/** 
 *
 * \file CXmlMap.h 
 *
 * \brief Contains the definition of a class
 * implementing a mapping between spaces of 
 * arbitrary dimension.
 */



class CMapSAX2Handler;



/**
 * 
 *  \class CXmlMap
 *  \brief     A class implementing the essential
 *   functionality for sensorymotor mappings between
 *   spaces of arbitrary dimension.
 *
 * 
 *  \author Martin Huelse, msh@aber.ac.uk
 *  
 *  \date    16.05.2008
 *
 */
class CXmlMap : public CBaseMap {

	public: 
		CXmlMap();
		~CXmlMap();
		
		/**
		 * 
		 * \anchor readXmlFile
		 * Read the complete data of a mapping stored in a
		 * xml-file.
		 * @param filename string containing the file name of the xml-file.
		 * The given xml-file must have input, output and 
		 * link attributes. If not, an exception is thrown.
		 * For reading older xml-versions without link attributes use
		 * the method \ref readXmlFileWithoutAttributes   
		 */
		void readXmlFile(char *filename) throw (IMapException);
		
		/**
		 * 
		 * 
		 * See \ref readXmlFile
		 *  \anchor readXmlFile
		 */
		void readXmlFileWithoutAttributes(char *filename) throw (IMapException);
		
		/**
		 * 
		 * Write the complete data of the mapping into a
		 * xml-file of the given name.
		 * @param filename string containing the file name
		 */
		void writeXmlFile(char *filename) throw(IMapException);
		

		/**
		 * 
		 * Write the complete data of the mapping into a
		 * string.
		 * @return char* string containing the whole mapping in XML
		 */
		char *writeXmlToString() throw(IMapException);

		/**
		 * 
		 * Read the complete data of a mapping stored in a
		 * string.
		 * @param char *xmlString string containing the complete
		 * data in xml-format.
		 * The given xml-string must have input, output and 
		 * link attributes. If not, an exception is thrown.
		 * For reading older xml-versions without link attributes
		 * can only be read from files
		 * the method \ref readXmlFileWithoutAttributes   
		 */
		void readXmlFromString(char *xmlStr) throw (IMapException);

		
		
		/**
		 * 
		 * Write comments into the mapping, which will be 
		 * stored into the xml-file, but won't touch any 
		 * data of the mapping and it won't influence the 
		 * behaviour of the mapping. 
		 */
		void writeComments(string s);
		
		/**
		 * 
		 * \return string containing the current comment of 
		 * the mapping.
		 */
		string getComments();

		
	protected:
		void readXmlStructure(char *filename, bool readAttributes) throw (IMapException);	
		void outInpNode(ofstream *fout, float *values, int dim);
		void outOutNode(ofstream *fout, float *values, int dim);
		void outAttrValues(ofstream *fout, float *values, int dim);
		void outLink(ofstream *fout, int idx);
		void outComment(ofstream *fout);
		void outDTD(ofstream *fout);
		void outMap(ofstream *fout);
		
		char *outDTD();
		char *outMap();
		char *outLink(int idx);
		char *outInpNode(float *values, int dim);
		char *outOutNode(float *values, int dim);
		char *outAttrValues(float *values, int dim);
		char *outComment();
		
		
		string comments_;
};

/**
 * 
 *  \class CMapSAX2Handler
 *  \brief     A class implementing
 * 
 *  \author Martin Huelse, msh@aber.ac.uk
 *  
 *  \date    16.05.2008
 *
 */
class CMapSAX2Handler : public DefaultHandler {
public:
	CMapSAX2Handler();
	~CMapSAX2Handler();
    void startElement(const   XMLCh* const    uri,const   XMLCh* const    localname, const   XMLCh* const    qname, const   Attributes&     attrs);
    void endElement(const XMLCh* const uri, const XMLCh* const localname, const XMLCh* const qname);
    //void characters(const XMLCh* const ch, const unsigned int length );
    void characters(const     XMLCh* const    chars, const   XMLSize_t    length);
    void fatalError(const SAXParseException& exception);
    void init(CXmlMap *map);
    void readLinkAttributes(bool b);
    
protected:
	

	
	XMLCh* xmlMap_;
	XMLCh* xmlMapInpDimAttr_;
	XMLCh* xmlMapOutDimAttr_;
	XMLCh* xmlMapLinkAttrDimAttr_;
	XMLCh* xmlComment_;
	XMLCh* xmlLink_;
	XMLCh* xmlInpNode_;
	XMLCh* xmlOutNode_;
	XMLCh* xmlNodeAttr_;
	
	XMLCh* xmlComponent_;
	XMLCh* xmlCompAttrValue_;
	XMLCh* xmlCompAttrIdx_;
	
	XMLCh* xmlLinkAttr_;
	XMLCh* xmlLinkAttrComponent_;
	XMLCh* xmlLinkAttrCompValue_;
	XMLCh* xmlLinkAttrCompIdx_;
	
	float *inpNodeValues_;
	float *outNodeValues_;
	float *nodeAttrValues_;
	
	int inpDim_;
	int outDim_;
	int attrDim_;
	
	bool readInpNode_;
	bool readOutNode_;
	bool readNodeAttr_;
	
	bool inpNodeReadMode_;
	bool outNodeReadMode_;
	bool nodeAttrReadMode_;
	
	bool firstLink_;
	
	
	bool readLinkAttributes_;
	
	CXmlMap *map_;
};


#endif /* AU_DCS_REVERB_SW_MAPS_CXMLMAP_H_*/
