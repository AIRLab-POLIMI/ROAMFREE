FILE(GLOB_RECURSE sources *.cpp)

include_directories(include/ROAMmath)

add_library(ROAMmath SHARED
	${sources}
)

if(catkin_FOUND)
# Install libraries
install(TARGETS ROAMmath
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

else()
# Non-catkin installation TODO
endif()