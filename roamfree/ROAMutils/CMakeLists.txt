FILE(GLOB_RECURSE sources *.cpp)

include_directories(include/ROAMutils)

add_library(ROAMutils SHARED
	${sources}
)

if(catkin_FOUND)
# Install libraries
install(TARGETS ROAMutils
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

else()
# Non-catkin installation TODO
endif()