
cmake_minimum_required(VERSION 3.11)
project(test_client_server)

if(NOT TARGET SlimSerialRTDE)
    find_package(SlimSerialRTDE REQUIRED)
endif()
 


if(${SlimSerialRTDE_USE_BUILTIN_SPDLOG_HEADERONLY})
    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/extern/spdlog/include)
else()
    find_package(spdlog REQUIRED)
endif()
# set(CMAKE_CXX_STANDARD 20)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++2a")
  
add_executable(test_client_server 
test_client_server.cpp
)
target_include_directories(test_client_server  PUBLIC
"$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>"
)

if(${SlimSerialRTDE_USE_BUILTIN_SPDLOG_HEADERONLY})
    TARGET_LINK_LIBRARIES( test_client_server
    SlimSerialRTDE
    )
else()
    target_link_libraries(test_client_server PUBLIC
    SlimSerialRTDE
    spdlog::spdlog
    )
endif()


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
