cmake_minimum_required(VERSION 3.16.0)

######################  utils *******************
include(cmake/utils.cmake)
include(cmake/ide.cmake)

#######################  get version from version.h*******************
SlimSerialRTDE_extract_version()

 
############################ Project Info *******************
project (SlimSerialRTDE VERSION ${SlimSerialRTDE_VERSION} LANGUAGES CXX)
message(STATUS "Build SlimSerialRTDE: ${SlimSerialRTDE_VERSION}")

############################ build type default as Release *******************
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose Release or Debug" FORCE)
endif()

############################ choose compiler *******************
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

############################ check system platform*******************
if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64")
    # message("Platform is Arm aarch64")
    set(ARM64   1)
    set(UNIX 1)
    set(WINDOWS 0)
elseif(${CMAKE_HOST_UNIX})
    # message("Platform is Linux x86_64")
    set(ARM64   0)
    set(UNIX 1)
    set(WINDOWS 0)
elseif(${CMAKE_HOST_WIN32})
    # message("Platform is Windows")
    set(ARM64   0)
    set(UNIX 0)
    set(WINDOWS 1)
    add_definitions(-D_WIN32_WINNT=0x0601)
endif()

 
if(${WINDOWS})
include(InstallRequiredSystemLibraries)
endif()
include(GNUInstallDirs)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

 
################## build options #############################
if(NOT DEFINED SlimSerialRTDE_MASTER_PROJECT)
    if(CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
        set(SlimSerialRTDE_MASTER_PROJECT ON)
    else()
        set(SlimSerialRTDE_MASTER_PROJECT OFF)
    endif()
endif()
# message(SlimSerialRTDE_MASTER_PROJECT = ${SlimSerialRTDE_MASTER_PROJECT})
 
# example options
option(SlimSerialRTDE_PYTHON_BINDING "Build Python Binding" OFF)

option(SlimSerialRTDE_USE_LOGURU "Use Loguru for logging" ON)

option(SlimSerialRTDE_BUILD_EXAMPLE "Build example" ON)

# testing options
option(SlimSerialRTDE_BUILD_TESTS "Build tests" OFF)

# install options
option(SlimSerialRTDE_INSTALL "Generate the install target" ON)

# sanitizer options
option(SlimSerialRTDE_SANITIZE_ADDRESS "Enable address sanitizer in tests" OFF)

##################  build static library #############################
if(POLICY CMP0167)
  cmake_policy(SET CMP0167 NEW)
endif()

find_package(Threads REQUIRED)

if(${UNIX})
    find_package(spdlog REQUIRED)
else()
    set(SlimSerialRTDE_USE_BUILTIN_SPDLOG ON)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/extern/spdlog)
    message("---------windows platform, use built-in spdlog")
    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/extern/spdlog/include)
endif()

# enable loguru if necessary
if(SlimSerialRTDE_USE_LOGURU)
    message(STATUS "Using Loguru for logging")
    find_package(loguru REQUIRED)
    add_compile_definitions(-DUSE_LOGURU)
else()
    message(STATUS "Using spdlog for logging")
endif()

 
set(SlimSerialRTDE_SRCS 
src/AsyncSerial.cpp
src/SlimSerialRTDE.cpp
src/SlimAction.cpp
)
add_library(SlimSerialRTDE STATIC ${SlimSerialRTDE_SRCS} ${SlimSerialRTDE_ALL_HEADERS})

add_library(SlimSerialRTDE::SlimSerialRTDE ALIAS SlimSerialRTDE)

set(SlimSerialRTDE_INCLUDES_LEVEL "")
if(SlimSerialRTDE_SYSTEM_INCLUDES)
    set(SlimSerialRTDE_INCLUDES_LEVEL "SYSTEM")
endif()

target_include_directories(SlimSerialRTDE ${SlimSerialRTDE_INCLUDES_LEVEL} PUBLIC 
"$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>"
"$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>")

target_include_directories(SlimSerialRTDE  PUBLIC 
"$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include/SlimSerialRTDE/asio>")

target_link_libraries(SlimSerialRTDE PUBLIC  
Threads::Threads
spdlog::spdlog
)

if(SlimSerialRTDE_USE_LOGURU)
    target_link_libraries(SlimSerialRTDE PUBLIC loguru::loguru)
endif()


# if(${UNIX})
# add_compile_options(-fPIC)
# target_link_libraries(SlimSerialRTDE PUBLIC
#  pthread dl
#  -fPIC
# )
# endif()

set_target_properties(SlimSerialRTDE PROPERTIES VERSION ${SlimSerialRTDE_VERSION} SOVERSION ${SlimSerialRTDE_VERSION_MAJOR}.${SlimSerialRTDE_VERSION_MINOR})


##################  build example #############################
if(SlimSerialRTDE_BUILD_EXAMPLE)
    message(STATUS "Generating example(s)")
    add_subdirectory(example)
endif()


##################  build tests #############################
if(${UNIX})
    if(SlimSerialRTDE_BUILD_TESTS )
        message(STATUS "Generating tests")
        enable_testing()
        add_subdirectory(tests)
    endif()
endif()

 

##################  install #############################
if(${UNIX})
    if(SlimSerialRTDE_INSTALL)
        message(STATUS "Generating install")
        set(project_config_in "${CMAKE_CURRENT_LIST_DIR}/cmake/SlimSerialRTDEConfig.cmake.in")
        set(project_config_out "${CMAKE_CURRENT_BINARY_DIR}/SlimSerialRTDEConfig.cmake")
        set(config_targets_file "SlimSerialRTDEConfigTargets.cmake")
        set(version_config_file "${CMAKE_CURRENT_BINARY_DIR}/SlimSerialRTDEConfigVersion.cmake")
        set(export_dest_dir "${CMAKE_INSTALL_LIBDIR}/cmake/SlimSerialRTDE")
        set(pkgconfig_install_dir "${CMAKE_INSTALL_LIBDIR}/pkgconfig")
        set(pkg_config "${CMAKE_BINARY_DIR}/${PROJECT_NAME}.pc")

        # ---------------------------------------------------------------------------------------
        # Include files
        # ---------------------------------------------------------------------------------------
        install(DIRECTORY include/ DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}")
        install(
            TARGETS SlimSerialRTDE
            EXPORT SlimSerialRTDE
            LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
            ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
            RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

        # ---------------------------------------------------------------------------------------
        # Install pkg-config file
        # ---------------------------------------------------------------------------------------
        if(IS_ABSOLUTE "${CMAKE_INSTALL_INCLUDEDIR}")
            set(PKG_CONFIG_INCLUDEDIR "${CMAKE_INSTALL_INCLUDEDIR}")
        else()
            set(PKG_CONFIG_INCLUDEDIR "\${prefix}/${CMAKE_INSTALL_INCLUDEDIR}")
        endif()
        if(IS_ABSOLUTE "${CMAKE_INSTALL_LIBDIR}")
            set(PKG_CONFIG_LIBDIR "${CMAKE_INSTALL_LIBDIR}")
        else()
            set(PKG_CONFIG_LIBDIR "\${exec_prefix}/${CMAKE_INSTALL_LIBDIR}")
        endif()
        get_target_property(PKG_CONFIG_DEFINES SlimSerialRTDE INTERFACE_COMPILE_DEFINITIONS)
        string(REPLACE ";" " -D" PKG_CONFIG_DEFINES "${PKG_CONFIG_DEFINES}")
        string(CONCAT PKG_CONFIG_DEFINES "-D" "${PKG_CONFIG_DEFINES}")
        configure_file("cmake/${PROJECT_NAME}.pc.in" "${pkg_config}" @ONLY)
        install(FILES "${pkg_config}" DESTINATION "${pkgconfig_install_dir}")

        # ---------------------------------------------------------------------------------------
        # Install CMake config files
        # ---------------------------------------------------------------------------------------
        export(TARGETS SlimSerialRTDE NAMESPACE SlimSerialRTDE::
               FILE "${CMAKE_CURRENT_BINARY_DIR}/${config_targets_file}")
        install(EXPORT SlimSerialRTDE DESTINATION ${export_dest_dir} NAMESPACE SlimSerialRTDE:: FILE ${config_targets_file})

        include(CMakePackageConfigHelpers)
        configure_package_config_file("${project_config_in}" "${project_config_out}" INSTALL_DESTINATION ${export_dest_dir})

        write_basic_package_version_file("${version_config_file}" COMPATIBILITY SameMajorVersion)
        install(FILES "${project_config_out}" "${version_config_file}" DESTINATION "${export_dest_dir}")

        # ---------------------------------------------------------------------------------------
        # Support creation of installable packages
        # ---------------------------------------------------------------------------------------
        include(cmake/SlimSerialRTDECPack.cmake)
    endif()
endif()


##################  uninstall #############################
# make uninstall
# add_custom_target("uninstall" COMMENT "Uninstall installed files")
# add_custom_command(
#     TARGET "uninstall"
#     POST_BUILD
#     COMMENT "Uninstall files with install_manifest.txt"
#     COMMAND xargs rm -vf < install_manifest.txt || echo Nothing in
#             install_manifest.txt to be uninstalled!
#     COMMAND xargs rm -rf  /usr/local/include/SlimSerialRTDE
#     COMMAND xargs rm -rf  /usr/local/lib/cmake/SlimSerialRTDE
# )




 