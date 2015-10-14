file(GLOB_RECURSE sources *.cpp)
list(REMOVE_ITEM sources src/RectangularInitialization.cpp)


include_directories(. include/ROAMvision)

add_library(ROAMvision SHARED
	${sources}
)

# add_executable(rectTest src/RectangularInitialization.cpp)

if(catkin_FOUND)
# Install libraries
install(TARGETS ROAMvision 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install headers
install(DIRECTORY include/ROAMvision
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

else()
# Non-catkin installation
# Install libraries
install(TARGETS ROAMvision 
  ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
)
# Install headers
install(DIRECTORY include/ROAMvision
  DESTINATION ${CMAKE_INSTALL_PREFIX}/include/roamfree
  PATTERN ".svn" EXCLUDE
)
endif()
