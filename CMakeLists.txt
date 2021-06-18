cmake_minimum_required(VERSION 3.10)
project(Domino)

set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_COMPILER "g++")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "-O3 -pthread")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -FATAL")
set(CMAKE_CXX_FLAGS "-std=c++17")

find_package(OpenCV 4.5 QUIET)
find_package(Threads)

include_directories(
    ${PROJECT_SOURCE_DIR}
    ${CURL_INCLUDE_DIR}
    /home/domino/20210322/Downloads/Galaxy_1.2.1911.9122/Galaxy_camera/inc
)

add_executable(Domino 
    main.cpp
    WzSerialportPlus.cpp
    TCPServer.cpp
    CRC_Check.cpp
    video.cpp
    mainPC.cpp
)

target_link_libraries(
    ${PROJECT_NAME}

    ${OpenCV_LIBS}
    ${CURL_LIBRARY}
    ${CMAKE_THREAD_LIBS_INIT}

    -lco
    -ldl
    -lgxiapi
)

