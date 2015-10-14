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
# Non-catkin installation
# Install libraries
install(TARGETS ROAMmath 
  ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
)
# Install headers
install(DIRECTORY include/ROAMmath
  DESTINATION ${CMAKE_INSTALL_PREFIX}/include/roamfree
  PATTERN ".svn" EXCLUDE
)
endif()
