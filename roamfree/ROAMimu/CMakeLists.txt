FILE(GLOB_RECURSE sources *.cpp)

include_directories(. include/ROAMimu)

add_library(ROAMimu SHARED
	${sources}
)

if(catkin_FOUND)
# Install libraries
install(TARGETS ROAMimu 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install headers
install(DIRECTORY include/ROAMimu
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

else()
# Non-catkin installation TODO
endif()