set(the_description "The UbiTrack Haptics Library")
ut_add_module(component_haptic_calibration utcore utdataflow)
ut_module_include_directories(../ ${UBITRACK_CORE_DEPS_INCLUDE_DIR})
ut_glob_module_sources(HEADERS "*.h" "*/*.h" SOURCES "*.cpp")

ut_create_module(${TINYXML_LIBRARIES} ${LOG4CPP_LIBRARIES} ${LAPACK_LIBRARIES} ${Boost_LIBRARIES})
