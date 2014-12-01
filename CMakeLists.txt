add_subdirectory(src/utHaptics)
add_subdirectory(src/utHapticsComponents)
add_subdirectory(src/utHapticsIOComponents)

# pattern should be split into uthaptics and uthapticscomponents .. until then, we force the installation of utHaptics.h
install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/src/utHaptics.h" DESTINATION "${UBITRACK_INCLUDE_INSTALL_PATH}" COMPONENT main)

ut_install_utql_patterns()
