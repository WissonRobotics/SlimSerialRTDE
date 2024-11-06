
cmake_minimum_required(VERSION 3.11)
project(SlimArm)

if(NOT TARGET SlimSerialRTDE)
    find_package(SlimSerialRTDE REQUIRED)
endif()
 
find_package(spdlog REQUIRED)

# set(CMAKE_CXX_STANDARD 20)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++2a")
  
# add_library(slim_arm STATIC 
# src/SlimArmRTDE.cpp
# src/SlimArm.cpp
# )
# target_include_directories(slim_arm  PUBLIC
# "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>"
# )
# TARGET_LINK_LIBRARIES( slim_arm
# SlimSerialRTDE
# spdlog
# )

# if(${ARM64})
#         message("-- The  slim_arm project platform is  ${CMAKE_SYSTEM_PROCESSOR}  system !")
#         TARGET_LINK_LIBRARIES( slim_arm
#         SlimSerialRTDE
#         config_center)
# else() 
#         find_package(spdlog REQUIRED)
#         message("-- The  slim_arm project platform is not arm64  ${CMAKE_SYSTEM_PROCESSOR}  system !")
#         TARGET_LINK_LIBRARIES( slim_arm
#         SlimSerialRTDE
#         config_center
#         spdlog
#         )
# endif()
