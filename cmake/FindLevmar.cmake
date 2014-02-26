# - Find levmar
# Find the native LEVMAR headers and libraries.
#
#  LEVMAR_INCLUDE_DIR -  where to find levmar.h, etc.
#  LEVMAR_LIBRARIES    - List of libraries when using levmar.
#  LEVMAR_FOUND        - True if levmar found.

GET_FILENAME_COMPONENT(module_file_path ${CMAKE_CURRENT_LIST_FILE} PATH )

# Look for the header file.
FIND_PATH(LEVMAR_INCLUDE_DIR NAMES levmar.h
                              PATHS /usr/local/include/levmar
									/usr/include/levmar
									${LEVMAR_ROOT}/include
									${module_file_path}/../../../External/include
                                    ${module_file_path}/../../../External/include/levmar
                              DOC "Path in which the file levmar.h is located." )

MARK_AS_ADVANCED(LEVMAR_INCLUDE_DIR)

# Look for the library.

IF( CMAKE_CL_64 )
  SET( LIB "lib64" )
ELSE( CMAKE_CL_64 )
  SET( LIB "lib" )
ENDIF( CMAKE_CL_64 )

IF(WIN32)
  FIND_LIBRARY(LEVMAR_LIBRARY NAMES levmar 
                               PATHS ${LEVMAR_ROOT}/${LIB}
									${module_file_path}/../../../External/${LIB}
                               DOC "Path to levmar library." )
ELSE(WIN32)
  FIND_LIBRARY( LEVMAR_LIBRARY NAMES levmar
                DOC "Path to levmar library." )
ENDIF(WIN32)
MARK_AS_ADVANCED(LEVMAR_LIBRARY)

# Copy the results to the output variables.
IF(LEVMAR_INCLUDE_DIR AND LEVMAR_LIBRARY)
  SET(LEVMAR_FOUND 1)
  SET(LEVMAR_LIBRARIES ${LEVMAR_LIBRARY})
  SET(LEVMAR_INCLUDE_DIR ${LEVMAR_INCLUDE_DIR})
ELSE(LEVMAR_INCLUDE_DIR AND LEVMAR_LIBRARY)
  SET(LEVMAR_FOUND 0)
  SET(LEVMAR_LIBRARIES)
  SET(LEVMAR_INCLUDE_DIR)
ENDIF(LEVMAR_INCLUDE_DIR AND LEVMAR_LIBRARY)

# Report the results.
IF(NOT LEVMAR_FOUND)
  SET(LEVMAR_DIR_MESSAGE
    "LEVMAR was not found. Make sure LEVMAR_LIBRARY and LEVMAR_INCLUDE_DIR are set.")
  IF(LEVMAR_FIND_REQUIRED)
    SET( LEVMAR_DIR_MESSAGE
         "${LEVMAR_DIR_MESSAGE} Levmar is required to build.")
    MESSAGE(FATAL_ERROR "${LEVMAR_DIR_MESSAGE}")
  ELSEIF(NOT LEVMAR_FIND_QUIETLY)
    SET( LEVMAR_DIR_MESSAGE
         "${LEVMAR_DIR_MESSAGE} LM Optimization might not work properly without levmar.")
    MESSAGE(STATUS "${LEVMAR_DIR_MESSAGE}")
  ENDIF(LEVMAR_FIND_REQUIRED)
ENDIF(NOT LEVMAR_FOUND)
