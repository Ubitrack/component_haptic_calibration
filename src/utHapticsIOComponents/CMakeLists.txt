set(the_description "The UbiTrack Haptics IO Components")
ut_add_component(uthapticsiocomponents DEPS utcore utdataflow uthaptics)


SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/../../cmake")

# find H3DUtil and HAPI
FIND_PACKAGE(H3DUtil)
FIND_PACKAGE(HAPI)
IF(H3DUTIL_FOUND AND HAPI_FOUND)

ut_component_include_directories(../ ${UBITRACK_CORE_DEPS_INCLUDE_DIR} ${HAPI_INCLUDE_DIR} ${H3DUTIL_INCLUDE_DIR})
ut_glob_component_sources(HEADERS "*.h" SOURCES "*.cpp")
ut_create_single_component()

ENDIF(H3DUTIL_FOUND AND HAPI_FOUND)