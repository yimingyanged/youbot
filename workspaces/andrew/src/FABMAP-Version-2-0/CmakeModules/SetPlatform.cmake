# This module simply sets the following variables
#
# PLATFORM_DARWIN
# PLATFORM_LINUX
# PLATFORM_SUNOS
# 
# as well as pre-processor definitions for each
#

IF( WIN32 )
        IF( MSVC )
                ADD_DEFINITIONS(-DWINDOWS_NT 
                                -D_CRT_SECURE_NO_DEPRECATE -D_SCL_SECURE_NO_DEPRECATE)
                SET(PLATFORM_LIBS wsock32 comctl32)
        ENDIF( MSVC )
ENDIF( WIN32 )

IF( UNIX )
        ADD_DEFINITIONS(-DUNIX)
        SET(PLATFORM_LIBS m pthread)
        IF("${CMAKE_SYSTEM_NAME}" MATCHES "Darwin")
                ADD_DEFINITIONS( -DPLATFORM_DARWIN )
                SET( PLATFORM_DARWIN 1 )
        ENDIF("${CMAKE_SYSTEM_NAME}" MATCHES "Darwin")
        IF("${CMAKE_SYSTEM_NAME}" MATCHES "Linux")
                ADD_DEFINITIONS( -DPLATFORM_LINUX )
                SET( PLATFORM_LINUX 1 )
        ENDIF("${CMAKE_SYSTEM_NAME}" MATCHES "Linux")
        IF("${CMAKE_SYSTEM_NAME}" MATCHES "SunOS")
                ADD_DEFINITIONS( -DPLATFORM_SUNOS )
                SET( PLATFORM_SUNOS 1 )
                SET(PLATFORM_LIBS ${PLATFORM_LIBS} socket nsl rt)
        ENDIF("${CMAKE_SYSTEM_NAME}" MATCHES "SunOS")
ENDIF( UNIX )

