# FindDBus1.cmake
#
# Find the D-Bus library using pkg-config.
# This module is needed because CMake does not ship a FindDBus1 module,
# and many Linux distributions do not provide a DBus1Config.cmake.
#
# Sets:
#   DBus1_FOUND        - True if D-Bus was found
#   DBus1_INCLUDE_DIRS - D-Bus include directories
#   DBus1_LIBRARIES    - D-Bus libraries to link against
#
# Creates imported target:
#   DBus1::DBus1

find_package(PkgConfig QUIET)

if (PkgConfig_FOUND)
    pkg_check_modules(PC_DBUS1 QUIET dbus-1)
endif()

find_path(DBus1_INCLUDE_DIR
    NAMES dbus/dbus.h
    HINTS ${PC_DBUS1_INCLUDE_DIRS}
    PATH_SUFFIXES dbus-1.0
)

# dbus arch-specific config header is in lib/dbus-1.0/include
find_path(DBus1_ARCH_INCLUDE_DIR
    NAMES dbus/dbus-arch-deps.h
    HINTS ${PC_DBUS1_INCLUDE_DIRS}
    PATH_SUFFIXES dbus-1.0/include
    PATHS
        /usr/lib/${CMAKE_LIBRARY_ARCHITECTURE}/dbus-1.0/include
        /usr/lib/dbus-1.0/include
        /usr/lib64/dbus-1.0/include
)

find_library(DBus1_LIBRARY
    NAMES dbus-1
    HINTS ${PC_DBUS1_LIBRARY_DIRS}
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(DBus1
    REQUIRED_VARS DBus1_LIBRARY DBus1_INCLUDE_DIR
)

if (DBus1_FOUND)
    set(DBus1_INCLUDE_DIRS ${DBus1_INCLUDE_DIR})
    if (DBus1_ARCH_INCLUDE_DIR)
        list(APPEND DBus1_INCLUDE_DIRS ${DBus1_ARCH_INCLUDE_DIR})
    endif()
    set(DBus1_LIBRARIES ${DBus1_LIBRARY})

    if (NOT TARGET DBus1::DBus1)
        add_library(DBus1::DBus1 UNKNOWN IMPORTED)
        set_target_properties(DBus1::DBus1 PROPERTIES
            IMPORTED_LOCATION "${DBus1_LIBRARY}"
            INTERFACE_INCLUDE_DIRECTORIES "${DBus1_INCLUDE_DIRS}"
        )
    endif()
endif()

mark_as_advanced(DBus1_INCLUDE_DIR DBus1_ARCH_INCLUDE_DIR DBus1_LIBRARY)
