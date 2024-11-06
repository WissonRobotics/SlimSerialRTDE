# ---------------------------------------------------------------------------------------
# IDE support for headers
# ---------------------------------------------------------------------------------------
set(SlimSerialRTDE_HEADERS_DIR "${CMAKE_CURRENT_LIST_DIR}/../include")


file(GLOB SlimSerialRTDE_TOP_HEADERS "${SlimSerialRTDE_HEADERS_DIR}/SlimSerialRTDE/*.h")
file(GLOB SlimSerialRTDE_LOGURU_HEADERS "${SlimSerialRTDE_HEADERS_DIR}/SlimSerialRTDE/*.hpp")
 
set(SlimSerialRTDE_ALL_HEADERS ${SlimSerialRTDE_TOP_HEADERS} ${SlimSerialRTDE_LOGURU_HEADERS})

source_group("Header Files\\SlimSerialRTDE" FILES ${SlimSerialRTDE_ALL_HEADERS}) 
