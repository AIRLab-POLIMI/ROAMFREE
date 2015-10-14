FILE(GLOB_RECURSE sources *.cpp)

include_directories(. include/ROAMestimation)

add_library(ROAMestimation SHARED
	${sources}
)
target_link_libraries(ROAMestimation ROAMfunctions ROAMmath ROAMutils ROAMlog ${G2O_LIBS})

if(catkin_FOUND)
# Install libraries
install(TARGETS ROAMestimation 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install headers
install(DIRECTORY include/ROAMestimation
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

else()
# Non-catkin installation
# Install libraries
install(TARGETS ROAMestimation 
  ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
)
# Install headers
install(DIRECTORY include/ROAMestimation
  DESTINATION ${CMAKE_INSTALL_PREFIX}/include/roamfree
  PATTERN ".svn" EXCLUDE
)
endif()
