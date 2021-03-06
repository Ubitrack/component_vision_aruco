project(ubitrack_component_vision_aruco)

cmake_minimum_required(VERSION 3.4.0)

include(${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
conan_basic_setup(TARGETS)

include(GNUInstallDirs)
include(cmake/UbitrackConfig)

# OpenGL
SET(HAVE_OPENGL 0)
find_package(OpenGL)
IF(OpenGL_FOUND)
  set(UBITRACK_COMPILE_DEFINITIONS ${UBITRACK_COMPILE_DEFINITIONS} "HAVE_OPENGL")
  SET(HAVE_OPENGL 1)
  MESSAGE(STATUS "Found OpenGL: ${OpenGL_INCLUDE_DIR} - ${OpenGL_LIBRARIES}")
ENDIF(OpenGL_FOUND)

# OpenCL
SET(HAVE_OPENCL 0)
find_package(OpenCL)
IF(OpenCL_FOUND)
  set(UBITRACK_COMPILE_DEFINITIONS ${UBITRACK_COMPILE_DEFINITIONS} "HAVE_OPENCL")
  include_directories(${OpenCL_INCLUDE_DIR})
  SET(HAVE_OPENCL 1)
  MESSAGE(STATUS "Found OpenCL: ${OpenCL_INCLUDE_DIR} - ${OpenCL_LIBRARY}")
ENDIF(OpenCL_FOUND)

# since we depend on it we now we have utvision:
set(UBITRACK_COMPILE_DEFINITIONS ${UBITRACK_COMPILE_DEFINITIONS} "HAVE_UTVISION")

add_subdirectory(src/utVisionComponentsAruco)

add_subdirectory(apps)

ubitrack_install_utql("doc/utql/*.xml" "doc/utql/*/*.xml"  "doc/utql/*/*/*.xml")
ubitrack_install_doc("doc/utqlDoc/*" "doc/utqlDoc/*/*" "doc/utqlDoc/*/*/*")
ubitrack_install_doc("doc/marker/*" "doc/marker/*/*")
