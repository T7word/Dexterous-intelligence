cmake_minimum_required(VERSION 3.16)

project(dexhand_example VERSION 1.0 LANGUAGES CXX)

if("${CMAKE_SYSTEM_PROCESSOR}" STREQUAL "arm" OR "${CMAKE_SYSTEM_PROCESSOR}" STREQUAL "aarch64")
    MESSAGE(STATUS "Platform: ARM architecture")
    set(IS_ARM TRUE)
else()
    MESSAGE(STATUS "Platform: x86 architecture")
    set(IS_ARM FALSE)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(INCLUDE_DIRS ${PROJECT_SOURCE_DIR} ${PROJECT_SOURCE_DIR}/../include)

include_directories(${INCLUDE_DIRS})

file(GLOB DX21_SOURCES ./dexhand021.cpp)
file(GLOB DX21S_SOURCES ./dexhand021s.cpp)

if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O2 -g -ggdb -pthread")
    set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -g -ggdb -pthread")
    add_compile_options(-shared -fPIC -fno-strict-aliasing -Wstringop-overflow=0 -Wpessimizing-move)
endif()

if(WIN32)
    set(DEPEND_LIB_DIR ${PROJECT_SOURCE_DIR}/../lib/windows/)
else()
    if(IS_ARM)
        set(DEPEND_LIB_DIR ${PROJECT_SOURCE_DIR}/../lib/linux/aarch64)
    else()
        set(DEPEND_LIB_DIR ${PROJECT_SOURCE_DIR}/../lib/linux/x86_64)
    endif()
endif()

if(MSVC)
    add_compile_options(/utf-8)
endif()

ADD_EXECUTABLE(dexhand021  ${DX21_SOURCES})
ADD_EXECUTABLE(dexhand021s ${DX21S_SOURCES})

target_include_directories(dexhand021  PUBLIC ${INCLUDE_DIRS})
target_include_directories(dexhand021s PUBLIC ${INCLUDE_DIRS})

if(WIN32)
    target_link_libraries(dexhand021  PUBLIC ${DEPEND_LIB_DIR}/dexhand.lib ${DEPEND_LIB_DIR}/zlgcan.lib)
    target_link_libraries(dexhand021s PUBLIC ${DEPEND_LIB_DIR}/dexhand.lib ${DEPEND_LIB_DIR}/zlgcan.lib)
else()
    target_link_libraries(dexhand021 PUBLIC ${DEPEND_LIB_DIR}/libusb-1.0.so)
    target_link_libraries(dexhand021 PUBLIC ${DEPEND_LIB_DIR}/libusbcanfd.so)
    #target_link_libraries(dexhand021 PUBLIC ${DEPEND_LIB_DIR}/libControlCAN.so)
    target_link_libraries(dexhand021 PUBLIC ${DEPEND_LIB_DIR}/libserial.so)
    target_link_libraries(dexhand021 PUBLIC ${DEPEND_LIB_DIR}/libdexhand.so)

    target_link_libraries(dexhand021s PUBLIC ${DEPEND_LIB_DIR}/libusb-1.0.so)
    target_link_libraries(dexhand021s PUBLIC ${DEPEND_LIB_DIR}/libusbcanfd.so)
    #target_link_libraries(dexhand021s PUBLIC ${DEPEND_LIB_DIR}/libControlCAN.so)
    target_link_libraries(dexhand021s PUBLIC ${DEPEND_LIB_DIR}/libserial.so)
    target_link_libraries(dexhand021s PUBLIC ${DEPEND_LIB_DIR}/libdexhand.so)
endif()
