SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/../../cmake")

# find H3DUtil and HAPI
# these should be handled as conan dependencies soon
FIND_PACKAGE(H3DUtil REQUIRED)
FIND_PACKAGE(HAPI REQUIRED)
FIND_PACKAGE(OpenHaptics REQUIRED)
FIND_PACKAGE(PTHREAD REQUIRED)
FIND_PACKAGE(DCMTK REQUIRED)

IF(H3DUTIL_FOUND AND HAPI_FOUND AND OPENHAPTICS_FOUND)
	include_directories(${HAPI_INCLUDE_DIR})
	include_directories(${H3DUTIL_INCLUDE_DIR})
	include_directories(${PTHREAD_INCLUDE_DIR})
	include_directories(${DCMTK_INCLUDE_DIR})
	include_directories(${OPENHAPTICS_INCLUDE_DIR})

	# All cpp files for UTCORE
	FILE(GLOB UBITRACK_SRCS RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}"
	   "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp"
	)

	set(UBITRACK_LIBRARY_NAME uthapticcalibration_io_components)

	SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/../../cmake")

	set(UBITRACK_EXTRA_LIBS ${UBITRACK_EXTRA_LIBS} ${PTHREAD_LIBRARIES} ${HAPI_LIBRARIES} ${H3DUTIL_LIBRARIES} ${DCMTK_LIBRARIES} ${OPENHAPTICS_LIBRARIES})

	add_library(${UBITRACK_LIBRARY_NAME} SHARED ${UBITRACK_SRCS})

	target_link_libraries(${UBITRACK_LIBRARY_NAME} 
  		CONAN_PKG::ubitrack_core
  		CONAN_PKG::ubitrack_dataflow
  		CONAN_PKG::ubitrack_hapticcalibration
  		${UBITRACK_EXTRA_LIBS}
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

ENDIF(H3DUTIL_FOUND AND HAPI_FOUND AND OPENHAPTICS_FOUND)