# Created by Patricia Shaw, Aberystwyth University, Feb 2012

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

PROJECT(SMcMaps)
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_MODULE_PATH $(CMAKE_MODULE_PATH) ${PROJECT_SOURCE_DIR}/../cmakeModules/)

FIND_PACKAGE(Xerces REQUIRED)
FIND_PACKAGE(Boost REQUIRED)
FIND_PACKAGE(YARP REQUIRED)



INCLUDE_DIRECTORIES(
	${PROJECT_SOURCE_DIR}/include
	${PROJECT_SOURCE_DIR}/../maps/include
	${PROJECT_SOURCE_DIR}/../kinematics/include
	${XERCES_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
	${YARP_INCLUDE_DIRS}
)
LINK_DIRECTORIES(
	${PROJECT_SOURCE_DIR}/../kinematics
	${PROJECT_SOURCE_DIR}/../maps
)

FILE(GLOB folder_source 
	src/armCompensator.cpp
	src/armController.cpp
	src/armReaching.cpp
	src/Excitation.cpp
	src/eyeController.cpp
	src/eyeHeadCalc.cpp
	src/EyeHeadController.cpp
	src/eyeSaccading.cpp
	src/graspController.cpp
	src/handEyeCoordination.cpp
	src/headController.cpp
	src/headSaccading.cpp
	src/ReachController.cpp
	src/ShortTermMemoryV2.cpp
	src/target.cpp
	src/testFunctions.cpp
	src/torsoController.cpp
	src/TorsoReach.cpp
	src/torsoSaccading.cpp
	src/utilities.cpp
	src/VamTarget.cpp
)
FILE(GLOB folder_header
	include/*.h
)

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

# Set up smc library
ADD_LIBRARY(smcLib ${folder_source} ${folder_header})
TARGET_LINK_LIBRARIES(smcLib)

ADD_EXECUTABLE(learningMenu ${PROJECT_SOURCE_DIR}/src/learningMenu.cpp ${folder_header})
ADD_EXECUTABLE(smc_cleverb4 ${PROJECT_SOURCE_DIR}/src/smc_cleverb4.cpp ${folder_header})
ADD_EXECUTABLE(rpcController ${PROJECT_SOURCE_DIR}/src/rpcControllerV3.cpp ${folder_header})
ADD_ExECUTABLE(emotionController ${PROJECT_SOURCE_DIR}/src/emotionController.cpp)
ADD_EXECUTABLE(ArmTester ${PROJECT_SOURCE_DIR}/src/ArmTester.cpp ${folder_header})
ADD_EXECUTABLE(TactileGrasp ${PROJECT_SOURCE_DIR}/src/TactileGrasp.cpp ${folder_header})
ADD_EXECUTABLE(demoVid ${PROJECT_SOURCE_DIR}/src/demoVid.cpp ${folder_header})
ADD_EXECUTABLE(BBcontroller ${PROJECT_SOURCE_DIR}/src/BBcontroller.cpp ${folder_header})

TARGET_LINK_LIBRARIES(learningMenu ${YARP_LIBRARIES} ${Boost_thread_LIBRARY} ${XERCES_LIBRARIES} kinLib mapsLib smcLib)
TARGET_LINK_LIBRARIES(smc_cleverb4 ${YARP_LIBRARIES} ${Boost_thread_LIBRARY} ${XERCES_LIBRARIES} kinLib mapsLib smcLib)
TARGET_LINK_LIBRARIES(rpcController ${YARP_LIBRARIES} ${Boost_thread_LIBRARY} ${XERCES_LIBRARIES} kinLib mapsLib smcLib)
TARGET_LINK_LIBRARIES(emotionController ${YARP_LIBRARIES})
TARGET_LINK_LIBRARIES(ArmTester ${YARP_LIBRARIES} ${Boost_thread_LIBRARY} ${XERCES_LIBRARIES} kinLib mapsLib smcLib)
TARGET_LINK_LIBRARIES(BBcontroller ${YARP_LIBRARIES} ${Boost_thread_LIBRARY} ${XERCES_LIBRARIES} kinLib mapsLib smcLib)
TARGET_LINK_LIBRARIES(demoVid ${YARP_LIBRARIES} ${Boost_thread_LIBRARY} ${XERCES_LIBRARIES} kinLib mapsLib smcLib)
TARGET_LINK_LIBRARIES(TactileGrasp ${YARP_LIBRARIES})
