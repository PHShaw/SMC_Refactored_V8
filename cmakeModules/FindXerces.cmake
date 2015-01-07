## 
# Try to find Xerces library
# Once run this will define: 
# 
# XERCES_INCLUDE_DIRS
# XERCES_LIBRARIES
##



##### check XERCES_DIR
IF (EXISTS "$ENV{XERCES_DIR}")

  SET(XERCES_INCLUDE_DIRS
	"$ENV{XERCES_DIR}/include")
  
  SET(XERCES_LIBRARIES
	"$ENV{XERCES_DIR}/lib/libxerces-c.so")
	
ENDIF (EXISTS "$ENV{XERCES_DIR}")
