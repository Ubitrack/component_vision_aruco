set(UBITRACK_LIBRARY_NAME components_vision_aruco)

# All cpp files for UTCORE
FILE(GLOB UBITRACK_SRCS RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}"
		"${CMAKE_CURRENT_SOURCE_DIR}/*.cpp"
		)

# All header files for UTCORE
FILE(GLOB UBITRACK_HEADERS RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}"
		"${CMAKE_CURRENT_SOURCE_DIR}/*.h"
		)


# library definition
ADD_LIBRARY(${UBITRACK_LIBRARY_NAME} SHARED ${UBITRACK_SRCS} ${UBITRACK_HEADERS})

target_link_libraries(${UBITRACK_LIBRARY_NAME} 
		CONAN_PKG::ubitrack_core
		CONAN_PKG::ubitrack_vision
		CONAN_PKG::ubitrack_dataflow
		OpenGL::GL
		OpenGL::GLU
		OpenCL::OpenCL
		)

# do not create symlinks to versioned names of component libraries
set_target_properties(${UBITRACK_LIBRARY_NAME} PROPERTIES NO_SONAME ON)

# Apply Target Properties
if(MSVC)
  if(CMAKE_CROSSCOMPILING)
    set_target_properties(${UBITRACK_LIBRARY_NAME} PROPERTIES LINK_FLAGS "/NODEFAULTLIB:secchk")
  endif()
  set_target_properties(${UBITRACK_LIBRARY_NAME} PROPERTIES LINK_FLAGS "/NODEFAULTLIB:libc /DEBUG")
endif()

ubitrack_set_target_properties(${UBITRACK_LIBRARY_NAME})
ubitrack_install_component(${UBITRACK_LIBRARY_NAME})
