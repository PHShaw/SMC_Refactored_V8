# Created by Patricia Shaw, Aberystwyth University, Feb 2012
# Depends on following environment variable:
# XERCES_DIR
# Where it is expected it will point to something like:
# XERCES_DIR=/usr/local/xerces-c-3.1.1


CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

PROJECT(SMcKinematics)

set(CMAKE_MODULE_PATH $(CMAKE_MODULE_PATH) ${PROJECT_SOURCE_DIR}/../cmakeModules/)

INCLUDE_DIRECTORIES(
	${PROJECT_SOURCE_DIR}/include
)

#Need to list these separately to remove executables and duplications
FILE(GLOB folder_source 
	src/CalcGazeChange.cpp
	src/CalcHand.cpp
	src/kinematicCalcs.cpp
)
FILE(GLOB folder_header
	include/CalcGazeChange.h
	include/CalcHand.h
	include/kinematicCalcs.h
	include/matricManipulation.h
)

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

# Set up maps library
ADD_LIBRARY(kinLib SHARED ${folder_source} ${folder_header})
TARGET_LINK_LIBRARIES(kinLib)
#INSTALL_TARGETS(${PROJECT_SOURCE_DIR}/lib kinLib)


#ADD_EXECUTABLE(gazeCalcTest ${PROJECT_SOURCE_DIR}/src/gazeCalcTest.cpp ${folder_header})
#ADD_EXECUTABLE(matrixTest ${PROJECT_SOURCE_DIR}/src/matrixTest.cpp ${folder_header})


#TARGET_LINK_LIBRARIES(gazeCalcTest kinLib)
#TARGET_LINK_LIBRARIES(matrixTest kinLib)
