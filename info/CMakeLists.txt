# Created by Patricia Shaw, Aberystwyth University, Feb 2012
# Depends on following environment variable:
# XERCES_DIR
# Where it is expected it will point to something like:
# XERCES_DIR=/usr/local/xerces-c-3.1.1


CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

PROJECT(SMcMaps)

set(CMAKE_MODULE_PATH $(CMAKE_MODULE_PATH) ${PROJECT_SOURCE_DIR}/../cmakeModules/)

FIND_PACKAGE(Xerces REQUIRED)
FIND_PACKAGE(YARP REQUIRED)

INCLUDE_DIRECTORIES(
	${PROJECT_SOURCE_DIR}/../maps/include
	${PROJECT_SOURCE_DIR}/../kinematics/include
	${XERCES_INCLUDE_DIRS}
	${YARP_INCLUDE_DIRS}
)

LINK_DIRECTORIES(
	${PROJECT_SOURCE_DIR}/../maps
	${PROJECT_SOURCE_DIR}/../kinematics
)

ADD_EXECUTABLE(mapOverlay ${PROJECT_SOURCE_DIR}/mapOverlay.cpp )

TARGET_LINK_LIBRARIES(mapOverlay ${YARP_LIBRARIES} ${XERCES_LIBRARIES} kinLib mapsLib)



