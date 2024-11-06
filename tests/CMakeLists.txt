cmake_minimum_required(VERSION 3.11)
project(SlimSerialRTDE_utests CXX)

if(NOT TARGET SlimSerialRTDE)
    # Stand-alone build
    find_package(SlimSerialRTDE REQUIRED)
endif()

include(../cmake/utils.cmake)

find_package(PkgConfig)
 

find_package(Catch2 REQUIRED)
if(Catch2_FOUND)
    message(STATUS "Packaged version of Catch will be used.")
else()
    message(STATUS "Bundled version of Catch will be downloaded and used.")
    include(FetchContent)
    FetchContent_Declare(Catch2
            GIT_REPOSITORY https://github.com/catchorg/Catch2.git
            GIT_TAG 53d0d913a422d356b23dd927547febdf69ee9081 # v3.5.0
    )
    FetchContent_MakeAvailable(Catch2)
endif()

set(SlimSerialRTDE_UTESTS_SOURCES
    test_communication.cpp 
)


enable_testing()

function(SlimSerialRTDE_prepare_test test_target SlimSerialRTDE_lib)
    add_executable(${test_target} ${SlimSerialRTDE_UTESTS_SOURCES})
    SlimSerialRTDE_enable_warnings(${test_target})
    target_link_libraries(${test_target} PRIVATE ${SlimSerialRTDE_lib})
    target_link_libraries(${test_target} PRIVATE Catch2::Catch2WithMain)
    if(SlimSerialRTDE_SANITIZE_ADDRESS)
        SlimSerialRTDE_enable_sanitizer(${test_target})
    endif()
    message("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
    message("!!!!!!!!!!!!!!!!!!!Need to run the following command in another terminal for test: !!!!!!!!!!!!!!!!!!!!!!")
    message("socat -d -d pty,rawer,echo=0,link=/tmp/ttyV0 pty,rawer,echo=0,link=/tmp/ttyV1 &")
    message("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
    # add_custom_command(TARGET SlimSerialRTDE-utests PRE_BUILD
    # COMMAND socat -d -d pty,rawer,echo=0,link=/tmp/ttyV0 pty,rawer,echo=0,link=/tmp/ttyV1 &
    # VERBATIM
    # )
    add_test(NAME ${test_target} COMMAND ${test_target})
    set_tests_properties(${test_target} PROPERTIES RUN_SERIAL ON)
endfunction()
 
# The compiled library tests
if(SlimSerialRTDE_BUILD_TESTS OR SlimSerialRTDE_BUILD_ALL)
    SlimSerialRTDE_prepare_test(SlimSerialRTDE-utests SlimSerialRTDE::SlimSerialRTDE)
endif()
 