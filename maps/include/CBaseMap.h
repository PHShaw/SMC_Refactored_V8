/*
    Declarations of the base class for mappings, called CBaseMap.
     
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
    
#ifndef AU_DCS_REVERB_SW_MAPS_CBASEMAP_H_
#define AU_DCS_REVERB_SW_MAPS_CBASEMAP_H_

#include <string>
using namespace std;

/**
 * 
 * \mainpage Reverb SW_BASEMAP Documentation
 *
 * \author Martin Huelse
 * 
 *
 * Aberystwyth University <BR>
 * Dept. Computer Science
 *
 * email: msh@aber.ac.uk
 * 
 * \date 15.05.2008
 *
 * \section intro_sec Introduction
 * These implementation provides the tools 
 * for the generation of sensorymotor maps
 * between spaces of arbitrary dimensions.
 * 
 *  
 */
 
 
/** 
 *
 * \file CBaseMap.h 
 *
 * \brief Contains the definition of a class
 * implementing a mapping between spaces of 
 * arbitrary dimension.
 */


/**
 * 
 * \class IMapException 
 * \brief Expection for our \ref CBaseMap
 * implementation and is derived classes.
 * 
 */
class IMapException {
	public:
	IMapException();

	/**
	 * 
	 * Constructor
	 * @param s const char*, string contains messages that 
	 * might help to give proper error messages, because 
	 * the are given while throwing the exception.
	 * @param c int, might used for specific codes
	 */
	IMapException(string s, int c);
	~IMapException();
	/**
	 * 
	 * \return string data about current error state
	 */
	string getMsg();
	/**
	 * 
	 * \return string data about current error state incl. additional information
	 */
	string getSupport();
	/**
	 * contains error code provided by underlying libraries
	 */
	int getCode();
	
	protected:
	/**
	 * 
	 * might contain specific data with respect to
	 * the error
	 */ 
	string msg_;
	/**
	 * 
	 * might contain general information for 
	 * the users about the reasons of the error
	 */ 
	string support_;
	/**
	 * 
	 * might contain a specific error code
	 */ 
	int code_;
};

/**
 * 
 *  \class CBaseMap
 *  \brief     A class implementing the essential
 *   functionality for sensorymotor mappings between
 *   spaces of arbitrary dimension.
 *
 * 
 *  \author Martin Huelse, msh@aber.ac.uk
 *  
 *  \date    15.05.2008
 *
 */
class CBaseMap {

	public:
		/**
		 * 
		 * Constructor
		 */
		CBaseMap();
		/**
		 * 
		 * Destructor
		 * 
		 */
		~CBaseMap();
		
		/**
		 * 
		 * \fn void init(int inpDim, int outDim, int attrDim, int initSize) throw (IMapException)
		 * \anchor initMap
		 * Initialization of the map. 
		 * @param inpDim int dimension of the input nodes
		 * @param outDim int dimension of the output node
		 * @param attrDim int dimension of the link attributes
		 * @param initSize int storage size of the map after
		 * initialization. If the map is full storage will be 
		 * enlarged in such a way that initSize additional 
		 * links can be stored. 
		 */
		void init(int inpDim, int outDim, int attrDim, int initSize) throw (IMapException);
		
		/**
		 * 
		 * Deinitialization. The map incl. its parameter
		 * about the dimension of input and output space
		 * is completely deleted. Allocated memory is free. 
		 */
		void deinit();
		
		/**
		 * 
		 * Deletes all nodes in the map. The parameters
		 * and the storage size remain the same.
		 */
		void clean();
		
		/**
		 * 
		 * 
		 * Adds a link to the map. If map is full new 
		 * memory will be allocated. If this fails an
		 * exception is thrown. A link is defined by 
		 * an input node and an output node. Each 
		 * link has attribute values. Here there are set to 
		 * zero per default. The nodes
		 * are represented by 1-dim. arrays of floats.
		 * The size of the array must not be smaller then 
		 * the defined input and output dimension. 
		 * @param inpNode *float  1-dim array of floats defining
		 * the input node. The size of the array must be at 
		 * least \ref inpDim_ .
		 * @param outNode *float  1-dim array of floats defining
		 * the output node. The size of the array must be 
		 * at least \ref outDim_ .
		 * 
		 * \anchor addLinkWithoutAttributes
		 */
		void addLink(float *inpNode, float *outNode) throw (IMapException);
		
		/**
		 * As \ref addLinkWithoutAttributes but here the
		 * attribute values are explicitly given.
		 * @param inpNode *float  1-dim array of floats defining
		 * the input node. The size of the array must be at 
		 * least \ref inpDim_ .
		 * @param outNode *float  1-dim array of floats defining
		 * the output node. The size of the array must be 
		 * at least \ref outDim_ .
		 * @param linkAttributes *floar 1-dim array of floats
		 * defining the attributes of the link. The size of the array
		 * must be ar leasr \ref attrDim_ .
		 * 
		 */
		void addLink(float *inpNode, float *outNode, float *linkAttributes) throw (IMapException);
		
		/**
		 * 
		 * returns the number of currently stored 
		 * links in the map.
		 * \return int > 0
		 */
		int  nmbOfLinks();  
		
		
		/**
		 * 
		 * Remove node of the given index. 
		 * After this method call the index 
		 * organization is totally different. 
		 * Any former index value might refer to a total
		 * different link.
		 * @param nodeIdx int  index of the node in this 
		 * mapping.
		 * If given index doesn't exist nothing changes.
		 */
		void removeLink(int nodeIdx);
		

		
		/**
		 * 
		 * return a pointer pointing to the 
		 * i-th link. If i is out of range, the return 
		 * value is the null-pointer NULL.
		 * @param i int  number of the link the list,
		 * starting with 0
		 */
		float *ptrLinkByIndex(int i);
		
		
		/**
		 * 
		 * Returns the dimension of the 
		 * input space of the map.
		 * \return int 0 iff map is not initialized yet
		 * otherwise it is a positive number.
		 */
		int getInpDim();
		
		/**
		 * 
		 * Returns the dimension of the 
		 * input space of the map.
		 * \return int 0 iff map is not initialized yet
		 * otherise it is a positive number. 
		 */
		int getOutDim();

		/**
		 * 
		 * Returns the dimension of the 
		 * attribute values of the link in this mapping.
		 * \return int -1 iff map is not initialized yet
		 * otherise it is a positive number. 
		 */
		int getAttrDim();

		
	protected:
		
		/**
		 * 
		 * Copies the content of source array into the 
		 * destination array. Both array have at least 
		 * nmb entries.
		 */
		void reCopy(float *destination, float *source, int nmb);
		
		/**
		 * 
		 * Initial size of the mapping indicating how many links
		 * the mapping can store before new memory must be allocated. 
		 * If needed the more memory will be provided automatically. 
		 */
		int initSize_;
		
		/**
		 * 
		 * Inidicates how many link the current mapping can store.
		 * 
		 */
		int maxNmbLinks_;
		
		/**
		 * 
		 * Index referring to the last link in the mapping.
		 */
		int indexLastLink_;
		
		/**
		 * 
		 * \var inpDim_
		 * 
		 *	Dimension of the input space.
		 * This value is set during initialization
		 * of the map \ref initMap
		 */
		int inpDim_;
		
		/**
		 * 
		 * \var outDim_
		 * 
		 *	Dimension of the output space.
		 * This value is set during initialization
		 * of the map \ref initMap
		 */
		int outDim_;
		
		/**
		 * 
		 *  \var attrDim_
		 * 
		 *  Dimension of the attribute values. This values
		 * is set during initialisation of the mapping 
		 * \ref initMap
		 */
		int attrDim_;
		
		/**
		 * 
		 * pointer to the 1-dim. array of floats which
		 * contains the mapping
		 */
		float *listOfLinks_;		
};

#endif /*AU_DCS_REVERB_SW_MAPS_CBASEMAP_H_*/
