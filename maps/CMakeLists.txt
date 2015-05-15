# Created by Patricia Shaw, Aberystwyth University, Feb 2012
# Depends on following environment variable:
# XERCES_DIR
# Where it is expected it will point to something like:
# XERCES_DIR=/usr/local/xerces-c-3.1.1


CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

PROJECT(SMcMaps)

set(CMAKE_MODULE_PATH $(CMAKE_MODULE_PATH) ${PROJECT_SOURCE_DIR}/../cmakeModules/)

FIND_PACKAGE(Xerces REQUIRED)
INCLUDE_DIRECTORIES(
	${PROJECT_SOURCE_DIR}/../kinematics/include
	${PROJECT_SOURCE_DIR}/include
	${XERCES_INCLUDE_DIRS}
)
LINK_DIRECTORIES(
	${PROJECT_SOURCE_DIR}/../kinematics
)

#Need to list these separately to remove executables and duplications
FILE(GLOB folder_source 
	src/CBaseMap.cpp
	src/CXmlMap.cpp
	src/FFM_IO.cpp
	src/Field.cpp
	src/FieldFieldMapping.cpp
	src/GazeField.cpp
	src/Gaze_IO.cpp
	src/GazeMap.cpp
	src/metricField.cpp
	src/PolarField.cpp
	src/ReachField.cpp
)
FILE(GLOB folder_header
	include/CBaseMap.h
	include/CXmlMap.h
	include/FFM_IO.h
	include/Field.h
	include/FieldFieldMapping.h
	include/GazeField.h
	include/Gaze_IO.h
	include/GazeMap.h
	include/metricField.h
	include/PolarField.h
	include/ReachField.h
)

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

# Set up maps library
ADD_LIBRARY(mapsLib SHARED ${folder_source} ${folder_header})
TARGET_LINK_LIBRARIES(mapsLib)
#INSTALL_TARGETS(${PROJECT_SOURCE_DIR}/lib mapsLib)


#ADD_EXECUTABLE(FFMtester ${PROJECT_SOURCE_DIR}/src/FFMtester.cpp ${folder_header})
ADD_EXECUTABLE(fieldExtract ${PROJECT_SOURCE_DIR}/src/fieldExtract.cpp ${folder_header})
#ADD_EXECUTABLE(gazeTester ${PROJECT_SOURCE_DIR}/src/gazeTester.cpp ${folder_header})
ADD_EXECUTABLE(gazeExtract ${PROJECT_SOURCE_DIR}/src/gazeExtract.cpp ${folder_header})
ADD_EXECUTABLE(allFieldExtract ${PROJECT_SOURCE_DIR}/src/allFieldExtract.cpp ${folder_header})

#TARGET_LINK_LIBRARIES(FFMtester ${XERCES_LIBRARIES} kinLib mapsLib)
TARGET_LINK_LIBRARIES(fieldExtract ${XERCES_LIBRARIES} kinLib mapsLib)
TARGET_LINK_LIBRARIES(allFieldExtract ${XERCES_LIBRARIES} kinLib mapsLib)
#TARGET_LINK_LIBRARIES(gazeTester ${XERCES_LIBRARIES} kinLib mapsLib)
TARGET_LINK_LIBRARIES(gazeExtract ${XERCES_LIBRARIES} kinLib mapsLib)



