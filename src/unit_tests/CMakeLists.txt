
message(STATUS "==== Unit Test programs ====")

#--------------------------------------------------------------
# Always use full RPATH (differentiating between the build and install trees)

# use, i.e. don't skip the full RPATH for the build tree
SET(CMAKE_SKIP_BUILD_RPATH  FALSE)

# when building, don't use the install RPATH already
# (but later on when installing)
SET(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE) 

SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

# add the automatically determined parts of the RPATH
# which point to directories outside the build tree to the install RPATH
SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# the RPATH to be used when installing, but only if it's not a system directory
LIST(FIND CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES "${CMAKE_INSTALL_PREFIX}/lib" isSystemDir)
IF("${isSystemDir}" STREQUAL "-1")
   SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
ENDIF("${isSystemDir}" STREQUAL "-1")

#--------------------------------------------------------------
# Option to enable metrics tracking
OPTION(TEST_METRICS "Enable tracking of performance metrics from instrumented tests" OFF)

if (TEST_METRICS)
  set(TEST_METRICS_PATH "" CACHE PATH "Output directory for JSON performance metrics files")
  mark_as_advanced(CLEAR TEST_METRICS_PATH)
else()
  mark_as_advanced(FORCE TEST_METRICS_PATH)
endif()

#--------------------------------------------------------------
# Propagate to subdirectories

option(BUILD_TESTS_BASE "Build unit tests for base Chrono module" TRUE)
mark_as_advanced(FORCE BUILD_TESTS_BASE)
if(BUILD_TESTS_BASE)
	ADD_SUBDIRECTORY(core)
	ADD_SUBDIRECTORY(benchmark)
	ADD_SUBDIRECTORY(physics)
endif()

IF (ENABLE_MODULE_MATLAB)
	option(BUILD_TESTS_MATLAB "Build unit tests for Matlab module" TRUE)
	mark_as_advanced(FORCE BUILD_TESTS_MATLAB)
	if(BUILD_TESTS_MATLAB)
  		ADD_SUBDIRECTORY(matlab)
  	endif()
ENDIF()

IF (ENABLE_MODULE_MKL)
	option(BUILD_TESTS_MKL "Build unit tests for MKL module" TRUE)
	mark_as_advanced(FORCE BUILD_TESTS_MKL)
	if(BUILD_TESTS_MKL)
  		ADD_SUBDIRECTORY(mkl)
  	endif()
ENDIF()

IF (ENABLE_MODULE_PARALLEL)
	option(BUILD_TESTS_PARALLEL "Build unit tests for Parallel module" TRUE)
	mark_as_advanced(FORCE BUILD_TESTS_PARALLEL)
	if(BUILD_TESTS_PARALLEL)
  		ADD_SUBDIRECTORY(parallel)
  	endif()
ENDIF()

IF (ENABLE_MODULE_FEA)
	option(BUILD_TESTS_FEA "Build unit tests for FEA module" TRUE)
	mark_as_advanced(FORCE BUILD_TESTS_FEA)
	if(BUILD_TESTS_FEA)
  		ADD_SUBDIRECTORY(fea)
  	endif()
ENDIF()
