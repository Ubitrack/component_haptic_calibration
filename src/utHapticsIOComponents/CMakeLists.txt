set(the_description "The UbiTrack Haptics IO Components")
# currently it's a single component and it should be renamed ..
ut_add_component(uthapticsiocomponents DEPS utcore utdataflow uthaptics)


SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/../../cmake")

# find H3DUtil and HAPI
FIND_PACKAGE(H3DUtil)
FIND_PACKAGE(HAPI)
 FIND_PACKAGE( OpenHaptics )
FIND_PACKAGE(PTHREAD)
FIND_PACKAGE(DCMTK)
IF(H3DUTIL_FOUND AND HAPI_FOUND AND OPENHAPTICS_FOUND)

ut_component_include_directories(../ ${UBITRACK_CORE_DEPS_INCLUDE_DIR} ${HAPI_INCLUDE_DIR} ${H3DUTIL_INCLUDE_DIR} ${PTHREAD_INCLUDE_DIR} ${DCMTK_INCLUDE_DIR} ${OPENHAPTICS_INCLUDE_DIR})
ut_glob_component_sources(HEADERS "*.h" SOURCES "*.cpp")
# some linking is missing .. segfaults at exit with:
# symbol lookup error: /usr/lib/libdcmdata.so.2: undefined symbol: _ZN7OFMutexD1Ev
ut_create_single_component(${PTHREAD_LIBRARIES} ${HAPI_LIBRARIES} ${H3DUTIL_LIBRARIES} ${DCMTK_LIBRARIES} ${OPENHAPTICS_LIBRARIES})

ENDIF(H3DUTIL_FOUND AND HAPI_FOUND AND OPENHAPTICS_FOUND)
