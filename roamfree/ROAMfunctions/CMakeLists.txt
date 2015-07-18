FILE(GLOB_RECURSE sources *.cpp)

include_directories(. include/ROAMfunctions)

add_library(ROAMfunctions SHARED
	${sources}
)

target_link_libraries(ROAMfunctions ROAMmath)

if(catkin_FOUND)
# Install libraries
install(TARGETS ROAMfunctions
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

else()
# Non-catkin installation TODO
endif()
