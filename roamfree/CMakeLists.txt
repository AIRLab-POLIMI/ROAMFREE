cmake_minimum_required(VERSION 2.8.3)

set(CATKIN_MSG FALSE)
find_package(catkin QUIET)
project(roamfree)

if(catkin_FOUND)
	catkin_package(INCLUDE_DIRS ROAMimu/include ROAMestimation/include ROAMvision/include
	               LIBRARIES ROAMestimation ROAMimu ROAMvision)
	set(CATKIN_MSG TRUE)
else()
	set(CATKIN_MSG FALSE)
endif()

# ----------------------- INCLUDES ----------------------------

include_directories(ROAMestimation/include 
                    ROAMmath/include
                    ROAMlog/include
                    ROAMimu/include
                    ROAMfunctions/
                    ROAMfunctions/include
                    ROAMutils/include
                    ROAMvision/include)

# ----------------------- LIBRARIES ---------------------------

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake/modules)

# --- Boost

find_package(Boost COMPONENTS python REQUIRED)

include_directories("${Boost_INCLUDE_DIRS}")
set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
  
# --- Eigen3

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# --- g2o

include_directories("g2o")
include_directories(${PROJECT_BINARY_DIR}/g2o) # for the config.h file
set(G2O_LIBS core solver_csparse)

# --- SuiteSparse

find_package(SuiteSparse REQUIRED)
include_directories(${CSPARSE_INCLUDE_DIR})

# --- python

find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_PATH})

# ----------------------- GCC FLAGS ----------------------------

set(CMAKE_CXX_FLAGS_DEBUG "-std=c++11 -g3 ${OTHERS_DEFINE}")
set(CMAKE_CXX_FLAGS_RELEASE "-std=c++11 -O3 ${OTHERS_DEFINE}")


if(CMAKE_BUILD_TYPE MATCHES Debug)	
	add_definitions( -DDEBUG_BUILD=1 )	
	
	# if uncommented, outputs to stderr the current active edges 
	# and vertices before each estimation run.
	add_definitions( -DDEBUG_PRINT_GRAPH_STRUCTURE )
	
	# if uncommented, outputs to stderr some Info messages
	add_definitions( -DDEBUG_PRINT_INFO_MESSAGES)
	add_definitions( -DDEBUG_PRINT_FACTORGRAPHFILTER_INFO_MESSAGES)
	add_definitions( -DDEBUG_PRINT_VISION_INFO_MESSAGES)
	                  
	# if enabled, in each edge the numeric jacobians are computed
	# and the estimation aborts if substantial difference
	# wrt the analytic one occurs. 
	add_definitions( -DDEBUG_COMPARE_WITH_NUMERIC_JACOBIANS )
	
	# check if Jacobians are properly initialized in GLC factory
	add_definitions( -DDEBUG_ARE_ALL_JACOBIANS_INITIALIZED )
	
	# the g2o optimizer constructor verbose flag 
	add_definitions( -DDEBUG_G2O_OPTIMIZER_VERBOSE=1 )

	# disable OpenMP in g2o TODO check message
	set( G2O_USE_OPENMP OFF CACHE BOOL "Build g2o with OpenMP support (EXPERIMENTAL)")
	
	set(BUILD_TYPE_MSG "Debug") 

else()
	add_definitions( -DDEBUG_G2O_OPTIMIZER_VERBOSE=0 )
	
	add_definitions( -DDEBUG_PRINT_VISION_INFO_MESSAGES)
	
	#enable OpenMP in g2o
	set( G2O_USE_OPENMP ON CACHE BOOL "Build g2o with OpenMP support (EXPERIMENTAL)")
	
	set(BUILD_TYPE_MSG "Release")
endif()

# ------------------------ MESSAGES ----------------------------

message(STATUS "Using catkin?    : " ${CATKIN_MSG})
message(STATUS "Build type       : " ${BUILD_TYPE_MSG})

message(STATUS "g20 libs         : " ${G2O_LIBS})

message(STATUS "csparse found?   : " ${CSPARSE_FOUND})
message(STATUS "csparse includes : " ${CSPARSE_INCLUDE_DIR})
message(STATUS "csparse libs     : " ${CSPARSE_LIBRARY})
message(STATUS "cholmod libs     : " ${CHOLMOD_LIBRARY})
message(STATUS "python includes  : " ${PYTHON_INCLUDE_PATH})
# -------------------------------------------------------------

add_subdirectory(g2o)

add_subdirectory(ROAMestimation)
add_subdirectory(ROAMfunctions)
add_subdirectory(ROAMutils)
add_subdirectory(ROAMmath)
add_subdirectory(ROAMlog)
add_subdirectory(ROAMtest)
add_subdirectory(ROAMimu)
add_subdirectory(ROAMvision)
