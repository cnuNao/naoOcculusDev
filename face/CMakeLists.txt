cmake_minimum_required(VERSION 2.8)
project(face)

find_package(qibuild)

set(CMAKE_PREFIX_PATH "/usr/local/bin;/usr/bin;/opt/naoqi_sdk/lib;/opt/naoqi_sdk/share/cmake"; ${CMAKE_PREFIX_PATH})

set(_srcs
    main.cpp
    onfacedetection.h
    onfacedetection.cpp
)

# Create a executable named hello
# with the source file: main.cpp
qi_create_bin(face ${_srcs})

# Add dependencies to created bin
qi_use_lib(face ALCOMMON ALPROXIES ALTHREAD)

# Add a simple test:
enable_testing()
qi_create_test(test_face "test.cpp")
