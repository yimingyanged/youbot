# This is a helper macro, to make it easier to make project builds optional.
#
# Usage:
# OPTIONAL_BUILD(<variable_root> <project_name> <Description> [<subdirectory names>] [<default val>])
#
# A boolean variable will be created in the GUI with the name
# <variable_root><project_name>
# and the description given by
# <Description>
# Its default value will be "OFF", unless specified by
# <default_val>
# which should have the value "ON" or "OFF"
#
# When the variable is set to "ON" by the user, the macro will perform an:
# add_subdirectory(<project name>)
# Unless optional paths are specified after <Description>, in which case only those will be added.
# You might use this if you wish to specify a shortened project name to be used in the GUI.
#
# If you do not wish to specify a <variable_root> then you should pass a quoted empty string ("")
#
# Notes:
# To make the macro available, make sure that directory in which this script resides
# is selected by setting the CMAKE_MODULE_PATH variable, and then use:
#
# INCLUDE(OptionalBuild)
#
# Example 1:
# OPTIONAL_BUILD(BUILD_ pLogStereo "Build pLogStereo process.")
#
# Makes an option in the CMake GUI called "BUILD_pLogStereo", with default value "OFF"
# When set to 'ON' in the CMake GUI, the project is compiled.
#
# Example 2:
# OPTIONAL_BUILD(BUILD_ pLS "Build pLogStereo process." pLogStereo ON)
#
# Makes an option in the CMake GUI called "BUILD_pLS", with default value "ON"
# but when turned on, will include the directory called 'pLogStereo'
#
# Example 3:
# SET(DStar_PROJECTS pDStarMain pDStarVisualizer)
# OPTIONAL_BUILD(BLD- DStar "Build DStar projects." ${DStar_PROJECTS} ON)
#
# Makes an option in the CMake GUI called "BLD-DStar", with default value "ON"
# but when turned on, will include all the directories in ${DSTar_PROJECTS}
#
# Bugs:
# In the current implementation, it is not compulsory to put the default boolean value
# at the end of the command.  This means that if multiple ON or OFF directives are found then
# only the last will be used.
# It also means that you cannot include a subdirectory with the name "ON" or "OFF"!

MACRO(OPTIONAL_BUILD opt_root proj_name comment)

  SET(default_val "OFF")
  SET(subdir_names "")

  IF ( ${ARGC} GREATER 3 )
    # OK, we've got optional args.  Let's loop through each and decide
    # what to do with it
    FOREACH(opt_var ${ARGN})
      IF   (opt_var STREQUAL "ON" OR opt_var STREQUAL "OFF")
        # It's the default value of the option
        SET(default_val ${opt_var})
      ELSE ()
        # It's a subdirectory name, so append it to the list
        LIST(APPEND subdir_names ${opt_var})
      ENDIF()
    ENDFOREACH(opt_var)
  ENDIF ()

  IF (NOT subdir_names)
    # We were given no optional subdirs, so use the project name as the name
    # of the subdirectory to add
    SET(subdir_names ${proj_name})
  ENDIF()

  # Make a cmake user variable with the correct name and default value
  SET(opt_name ${opt_root}${proj_name})
  OPTION(${opt_name} ${comment} ${default_val})
  
  # If the option is turned on by the user then add all of the subdirectories
  IF (${opt_name})
    FOREACH(subdir_name ${subdir_names})
      add_subdirectory(${subdir_name})
    ENDFOREACH(subdir_name)
  ENDIF (${opt_name})
  
ENDMACRO(OPTIONAL_BUILD)
