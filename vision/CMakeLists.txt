CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

FIND_PACKAGE(YARP REQUIRED)
FIND_PACKAGE(OpenCV REQUIRED)

INCLUDE_DIRECTORIES(
		${YARP_INCLUDE_DIRS}
		${OpenCV_INCLUDE_DIRS} 
)

FILE(GLOB folder_header
	*.h
)


ADD_EXECUTABLE(vision vision4.cpp)
ADD_EXECUTABLE(visionSim vision4_alt.cpp)
ADD_EXECUTABLE(gazeViewLive gazeViewLive.cpp)
ADD_EXECUTABLE(opencv_example opencv_example.cpp)
ADD_EXECUTABLE(vision6 vision6.cpp ${folder_header})

TARGET_LINK_LIBRARIES(vision ${YARP_LIBRARIES})
TARGET_LINK_LIBRARIES(visionSim ${YARP_LIBRARIES})
TARGET_LINK_LIBRARIES(gazeViewLive ${YARP_LIBRARIES})
TARGET_LINK_LIBRARIES(opencv_example ${YARP_LIBRARIES} ${OpenCV_LIBS})
TARGET_LINK_LIBRARIES(vision6 ${YARP_LIBRARIES} ${OpenCV_LIBS})



INSTALL_TARGETS($ENV{ICUB_DIR}/bin vision6)
