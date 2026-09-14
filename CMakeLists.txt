cmake_minimum_required(VERSION 3.10)
project(grblHAL-glowforge C)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS Debug Release RelWithDebInfo MinSizeRel)
endif()

# PTHREAD_RECURSIVE_MUTEX_INITIALIZER_NP and friends.
add_compile_definitions(_GNU_SOURCE)

if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    # The stepper producer thread runs core "ISR" code concurrently with the
    # protocol thread; keep type-punning-heavy embedded code out of
    # strict-aliasing territory.
    add_compile_options(-fno-strict-aliasing)
    # An implicitly declared function returning float reads the wrong
    # register on the hard-float target; make the whole class a hard error.
    add_compile_options(-Werror=implicit-function-declaration)
endif()

# Machine configuration: my_machine.h (-> boards/glowforge.h) is force-
# included into every translation unit, core sources included, so the
# core's #ifndef-guarded defaults in grbl/config.h pick up the measured
# Glowforge machine constants without touching the submodule.
add_compile_options(-include ${CMAKE_CURRENT_LIST_DIR}/src/my_machine.h)

# Capture the effective build-time flags for the -v/--version output.
string(TOUPPER "${CMAKE_BUILD_TYPE}" _drv_build_type)
if(_drv_build_type STREQUAL "")
    set(DRV_BUILD_TYPE "None")
else()
    set(DRV_BUILD_TYPE "${CMAKE_BUILD_TYPE}")
endif()
string(STRIP "${CMAKE_C_FLAGS} ${CMAKE_C_FLAGS_${_drv_build_type}}" DRV_C_FLAGS)
# Keep host build directories out of the binary: distro toolchains pass
# path-remapping and sysroot flags that only describe the build machine.
string(REGEX REPLACE "-f[a-z-]*prefix-map=[^ ]*" "" DRV_C_FLAGS "${DRV_C_FLAGS}")
string(REGEX REPLACE "--sysroot=[^ ]*" "" DRV_C_FLAGS "${DRV_C_FLAGS}")
string(REGEX REPLACE "  +" " " DRV_C_FLAGS "${DRV_C_FLAGS}")
string(STRIP "${DRV_C_FLAGS}" DRV_C_FLAGS)
# The driver version $I reports: the describing git tag or commit of the
# source, "unknown" outside a checkout.
execute_process(
    COMMAND git -C ${CMAKE_CURRENT_LIST_DIR} describe --always --dirty --tags
    OUTPUT_VARIABLE DRV_VERSION OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET RESULT_VARIABLE _drv_git_rc)
if(NOT _drv_git_rc EQUAL 0 OR "${DRV_VERSION}" STREQUAL "")
    set(DRV_VERSION "unknown")
endif()
configure_file(
    ${CMAKE_CURRENT_LIST_DIR}/src/build_info.h.in
    ${CMAKE_CURRENT_BINARY_DIR}/build_info.h
    @ONLY
)

include(src/grbl/CMakeLists.txt)

set(DRIVER_SOURCES
    src/main.c
    src/bind_addr.c
    src/driver.c
    src/stepper_stream.c
    src/glowforge_io.c
    src/glowforge_cooling.c
    src/glowforge_homing.c
    src/glowforge_laser.c
    src/glowforge_status.c
    src/glowforge_switches.c
    src/serial.c
    src/eeprom.c
    src/grbl_eeprom_extensions.c
    src/platform_linux.c
    src/fflog.c
)

add_executable(grblHAL_glowforge ${DRIVER_SOURCES})

if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    # The DRIVER's own sources are warning-free and held to that; the
    # core submodule is upstream code and is not held to -Werror here.
    set_source_files_properties(${DRIVER_SOURCES}
        tests/switch_map_test.c tests/laser_arm_test.c tests/serial_test.c
        tests/bind_addr_test.c tests/lens_home_test.c tests/xy_scale_test.c
        tests/latch_test.c tests/cooling_test.c tests/switches_test.c
        PROPERTIES COMPILE_OPTIONS "-Werror")
endif()

target_link_libraries(grblHAL_glowforge PRIVATE
    m
    grbl
    rt
    pthread
)

# Generated build_info.h lands in the build dir.
target_include_directories(grblHAL_glowforge PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

install(TARGETS grblHAL_glowforge RUNTIME DESTINATION bin)

# Host unit test for the EV_SW switch-map decode (the pure mapping in
# src/glowforge_switch_map.h). Built everywhere, installed nowhere; CI
# runs it after the build.
add_executable(switch_map_test tests/switch_map_test.c)
# Headers only - the test exercises the pure mapping and must not link
# the core (whose entry points expect the full driver).
target_include_directories(switch_map_test PRIVATE src ${CMAKE_CURRENT_BINARY_DIR})

# Host unit test for the operator-arm coolant re-check (G-4): the arm
# must re-check gfcool_fire_ok() after the button wait. It includes the
# driver source directly and stubs the core, so - like switch_map_test -
# it must not link the core. Built everywhere, installed nowhere.
add_executable(laser_arm_test tests/laser_arm_test.c)
target_include_directories(laser_arm_test PRIVATE src ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(laser_arm_test m)

# Host unit test for the RX ring under a sender that ignores flow
# control: the overrunning line is dropped whole, earlier lines are
# delivered intact, real-time characters keep passing, and the overrun
# is reported once. Includes the serial source directly, stubs the core.
add_executable(serial_test tests/serial_test.c)
target_include_directories(serial_test PRIVATE src ${CMAKE_CURRENT_BINARY_DIR})

# Host unit test for the -b listen-address parser (src/bind_addr.c): an
# IPv6 literal binds as written, an IPv4 literal binds in its v4-mapped
# form, and anything that is not a literal is rejected. Links the parser
# source alone; must not link the core.
add_executable(bind_addr_test tests/bind_addr_test.c src/bind_addr.c)
target_include_directories(bind_addr_test PRIVATE src ${CMAKE_CURRENT_BINARY_DIR})

# Host unit test for the Z a home sets (the pure helpers in
# src/glowforge_homing.h): the edge's focal height lands on the step
# grid the controller counts, so the reported position agrees with the
# home position, and the park from the edge stays inside the window
# every head reaches without touching a stop. Headers only; must not
# link the core.
add_executable(lens_home_test tests/lens_home_test.c)
target_include_directories(lens_home_test PRIVATE src ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(lens_home_test m)

# Host unit test for the XY microstep scale (the pure helpers in
# src/glowforge_io.h): the mode a config value names, and the $100/$101,
# machine tick, stop ramp and feed ceiling derived from it. Headers only.
add_executable(xy_scale_test tests/xy_scale_test.c)
target_include_directories(xy_scale_test PRIVATE src ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(xy_scale_test m)

# Host unit test for the laser latch writer (src/stepper_stream.c): a
# lock is retried and faults the stream when it still fails, an unlock
# that fails unlocks nothing, an unlock of a latch believed unlocked
# writes nothing, and a run start relights only a lock this process
# wrote under an open fire gate. Includes the stream source, stubs the
# sysfs layer; must not link the core.
add_executable(latch_test tests/latch_test.c)
target_include_directories(latch_test PRIVATE src ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(latch_test m pthread)

# Host unit test for the cooling client's verdict tiers
# (src/glowforge_cooling.c): the fail tier ends the job, the pause tier
# locks the latch under the window and resumes on the clean verdict,
# and every armed change is reported at once. Includes the client
# source, publishes verdict files; must not link the core.
add_executable(cooling_test tests/cooling_test.c)
target_include_directories(cooling_test PRIVATE src ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(cooling_test pthread)

# Host unit test for the switch poll's cancel path
# (src/glowforge_switches.c): a lid cancel in flight sends its reset and
# runs its park on polls whose switch read fails. Includes the switch
# source, drives the file-backed test source; must not link the core.
add_executable(switches_test tests/switches_test.c)
target_include_directories(switches_test PRIVATE src ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(switches_test m)
