# CMake instructions to make the static lib
cmake_minimum_required(VERSION 3.16.0)

set(CMAKE_CXX_STANDARD 20) 
 
project ( SlimSerialRTDE)

add_definitions("-Wall -g")

add_compile_options(-fsanitize=address)
add_link_options(-fsanitize=address)

if(${UNIX})
find_package(Threads REQUIRED)
endif()
############################ check system platform*******************
if(${CMAKE_HOST_UNIX})
    message("Linux x86_64")
    set(UNIX 1)
    message(UNIX = ${UNIX})
elseif(${CMAKE_HOST_WIN32})
    message("Windows")
    set(WINDOWS 1)
    add_definitions(-D_WIN32_WINNT=0x0601)
    message(WINDOWS = ${WINDOWS})
endif()
if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64")
    message("Arm aarch64")
    set(ARM64   1)
    message(ARM64 = ${ARM64})
endif()

if(${UNIX})
    add_compile_options(-fPIC)
endif()


 
if(${UNIX})
    SET(Boost_USE_STATIC_LIBS ON)
    set(Boost_USE_STATIC_RUNTIME ON)
    SET(Boost_USE_MULTITHREAD ON)
    set(Boost_NO_BOOST_CMAKE ON) 
elseif(${WINDOWS})
    set(Boost_NO_WARN_NEW_VERSIONS 1)
    add_definitions(-D_WIN32_WINNT=0x0601)
    set(Boost_USE_STATIC_LIBS ON)
    set(Boost_USE_STATIC_RUNTIME ON)
    set(Boost_INCLUDE_DIR ${_VCPKG_INSTALLED_DIR}/x64-windows-static-md/include)
    FIND_PACKAGE(Boost REQUIRED COMPONENTS system)
endif()
FIND_PACKAGE(Boost REQUIRED COMPONENTS system)
SET(BOOST_LIBS ${Boost_SYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY})

if (Boost_FOUND)
    include_directories(${Boost_INCLUDE_DIRS})
    link_directories(${Boost_LIBRARY_DIRS})
    message("found boost Boost_INCLUDE_DIRS=" ${Boost_INCLUDE_DIRS})
    message("found boost Boost_LIBRARY_DIRS=" ${Boost_LIBRARY_DIRS})
endif(Boost_FOUND)


  
find_package(fmt REQUIRED) 
ADD_LIBRARY( loguru
             src/loguru.cpp 
             )
target_include_directories(loguru
             PRIVATE
             ${CMAKE_CURRENT_SOURCE_DIR}/include 
)
 
############################ Pybind11 (local subfolder)*******************
#add_subdirectory(extern/pybind11)
#include_directories(${pybind11_INCLUDE_DIRS})
#include_directories(extern/pybind11/include)
#message("${pybind11_INCLUDE_DIRS}" aaaaaaaaaais ${pybind11_INCLUDE_DIRS})
 


############################ SlimSerialRTDE static library *******************
ADD_LIBRARY( SlimSerialRTDE
             src/SlimSerialRTDE.cpp
             src/AsyncSerial.cpp  
             )
target_include_directories(SlimSerialRTDE
             PRIVATE
             ${CMAKE_CURRENT_SOURCE_DIR}/include 
             ${Boost_INCLUDE_DIRS}

)
 
if(${UNIX})
target_link_libraries(SlimSerialRTDE
 pthread dl
 -fPIC
)
endif()

target_link_libraries(SlimSerialRTDE
    ${BOOST_LIBS}
    loguru
)


if(${WINDOWS})
include(InstallRequiredSystemLibraries)
endif()
include(GNUInstallDirs)

 
set_property(TARGET SlimSerialRTDE
 PROPERTY PUBLIC_HEADER ${CMAKE_CURRENT_SOURCE_DIR}/include/SlimSerialRTDE.h
 ${CMAKE_CURRENT_SOURCE_DIR}/include/AsyncSerial.h
 ${CMAKE_CURRENT_SOURCE_DIR}/include/loguru.hpp
 ${CMAKE_CURRENT_SOURCE_DIR}/include/slimCircularBuffer.h
 ${CMAKE_CURRENT_SOURCE_DIR}/include/thread_safe_queue.h)
install(TARGETS SlimSerialRTDE
    EXPORT SlimSerialRTDETargets  
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/SlimSerialRTDE
)
 


#find_package(Python COMPONENTS Interpreter Development)
 
############################ install python *******************    
# pybind11_add_module(pySlimSerialRTDE 
# python_binding/SlimSerialRTDEWrap.cpp
 
# src/SlimSerialRTDE.cpp
# example/Maxwell_SoftArm_SerialClient.cpp
# example/slimRegister.h)
# target_link_libraries(pySlimSerialRTDE PRIVATE 
# SlimSerialRTDE
# )
# target_include_directories(pySlimSerialRTDE
#      PUBLIC
#      ${CMAKE_CURRENT_SOURCE_DIR}/include
#      ${CMAKE_CURRENT_SOURCE_DIR}/example
# )
# install(TARGETS pySlimSerialRTDE
#   COMPONENT python
#   RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
#   LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
#   ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}")

############################ install cmake ******************* 
 
 

include(CMakePackageConfigHelpers)
set(INCLUDE_INSTALL_DIR ${CMAKE_INSTALL_INCLUDEDIR}/SlimSerialRTDE
CACHE  PATH "Location of header files" )
set(LIBRARY_INSTALL_DIR ${CMAKE_INSTALL_LIBDIR}
CACHE PATH "Location of library files" )
configure_package_config_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/SlimSerialRTDEConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/SlimSerialRTDEConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/SlimSerialRTDE
    PATH_VARS INCLUDE_INSTALL_DIR LIBRARY_INSTALL_DIR)
 
install(FILES  ${CMAKE_CURRENT_BINARY_DIR}/SlimSerialRTDEConfig.cmake 
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/SlimSerialRTDE 
        )




############################      example   EXE    *******************





 

############################ Begin example WISSON_DUMMU_SERVER  EXE *******************
# add_executable( wisson_dummy_server
#                example/wisson_dummy_server.cpp
#              )
# target_include_directories(wisson_dummy_server
#              PRIVATE
#              ${CMAKE_CURRENT_SOURCE_DIR}/example
#              ${CMAKE_CURRENT_SOURCE_DIR}/include
# )
# TARGET_LINK_LIBRARIES( wisson_dummy_server
#             SlimSerialRTDE
# )
# install(TARGETS wisson_dummy_server
#     EXPORT wisson_dummy_serverTargets)
# if(${WINDOWS})
# install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/example/config.ini
#         DESTINATION ${CMAKE_INSTALL_BINDIR})
# else()
# install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/example/config.ini
#         DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/build)
# endif()
############################ End example WISSON_DUMMU_SERVER  EXE *******************




############################ Begin example speaker  EXE *******************
ADD_LIBRARY( speaker
               example/speaker.cpp
             )
target_include_directories(speaker
             PRIVATE
             ${CMAKE_CURRENT_SOURCE_DIR}/example
             ${CMAKE_CURRENT_SOURCE_DIR}/include
)
TARGET_LINK_LIBRARIES( speaker
            SlimSerialRTDE
)

 
############################ End example speaker  EXE *******************

# find_package(jsoncpp REQUIRED) 



ADD_LIBRARY( Maxwell_SoftArm_SerialClient
             example/monos_arm/Maxwell_SoftArm_SerialClient.cpp
             )
target_include_directories(Maxwell_SoftArm_SerialClient PUBLIC
${CMAKE_CURRENT_SOURCE_DIR}/include
${CMAKE_CURRENT_SOURCE_DIR}/example/monos_arm
)
target_link_libraries(Maxwell_SoftArm_SerialClient
SlimSerialRTDE
loguru
fmt::fmt
)

ADD_LIBRARY( monos_arm
             example/monos_arm/monos_arm.cpp 
 
             )
target_include_directories(monos_arm PUBLIC
${CMAKE_CURRENT_SOURCE_DIR}/include
${CMAKE_CURRENT_SOURCE_DIR}/example/monos_arm
${jsoncpp_INCLUDE_DIRS}
)
target_link_libraries(monos_arm
Maxwell_SoftArm_SerialClient
fmt::fmt 
loguru
)



############################ begin modbus client example##########

add_executable( modbus_client
             example/modbus_client.cpp 
             )
target_include_directories(modbus_client PUBLIC
${CMAKE_CURRENT_SOURCE_DIR}/include 
)
target_link_libraries(modbus_client
SlimSerialRTDE
loguru
fmt::fmt
)
############################ end modbus client example##########

############################ Begin example SlimSerialMain  EXE *******************
add_executable( main
               "example/main.cpp"
             )
target_include_directories(main
             PRIVATE
             ${CMAKE_CURRENT_SOURCE_DIR}/include
             ${CMAKE_CURRENT_SOURCE_DIR}/example/monos_arm/
              
 
)
TARGET_LINK_LIBRARIES( main
            SlimSerialRTDE
            Maxwell_SoftArm_SerialClient
            monos_arm
            loguru
)

############################ End example SlimSerialMain  EXE *******************