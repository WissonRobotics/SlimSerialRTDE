# Copyright(c) 2019 SlimSerialRTDE authors Distributed under the MIT License (http://opensource.org/licenses/MIT)

cmake_minimum_required(VERSION 3.11)
project(SlimSerialRTDE_examples CXX)
 
# add_subdirectory(SlimArm)
add_subdirectory(test_client_server)
  
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
# ADD_LIBRARY( speaker
#     speaker.cpp
# )
# TARGET_LINK_LIBRARIES( speaker
#             SlimSerialRTDE
# )

 
############################ End example speaker  EXE *******************

# find_package(jsoncpp REQUIRED) 

# ADD_LIBRARY( Maxwell_SoftArm_SerialClient
#              example/monos_arm/Maxwell_SoftArm_SerialClient.cpp
#              )
# target_include_directories(Maxwell_SoftArm_SerialClient PUBLIC
# ${CMAKE_CURRENT_SOURCE_DIR}/include
# ${CMAKE_CURRENT_SOURCE_DIR}/example/monos_arm
# )
# target_link_libraries(Maxwell_SoftArm_SerialClient
# SlimSerialRTDE
# loguru
# fmt::fmt
# )

# ADD_LIBRARY( monos_arm
#              example/monos_arm/monos_arm.cpp 
 
#              )
# target_include_directories(monos_arm PUBLIC
# ${CMAKE_CURRENT_SOURCE_DIR}/include
# ${CMAKE_CURRENT_SOURCE_DIR}/example/monos_arm
# ${jsoncpp_INCLUDE_DIRS}
# )
# target_link_libraries(monos_arm
# Maxwell_SoftArm_SerialClient
# fmt::fmt 
# loguru
# )



# ############################ begin modbus client example##########

# add_executable( modbus_client
#              example/modbus_client.cpp 
#              )
# target_include_directories(modbus_client PUBLIC
# ${CMAKE_CURRENT_SOURCE_DIR}/include 
# )
# target_link_libraries(modbus_client
# SlimSerialRTDE
# loguru
# fmt::fmt
# )
# ############################ end modbus client example##########

# ############################ Begin example SlimSerialMain  EXE *******************
# add_executable( main
#                "example/main.cpp"
#              )
# target_include_directories(main
#              PRIVATE
#              ${CMAKE_CURRENT_SOURCE_DIR}/include
#              ${CMAKE_CURRENT_SOURCE_DIR}/example/monos_arm/
              
 
# )
# TARGET_LINK_LIBRARIES( main
#             SlimSerialRTDE
#             Maxwell_SoftArm_SerialClient
#             monos_arm
#             loguru
# )
