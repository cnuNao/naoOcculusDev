cmake_minimum_required(VERSION 2.8)
project(hello)

find_package(qibuild)

set(CMAKE_PREFIX_PATH "/usr/local/bin;/usr/bin;/opt/naoqi_sdk/lib;/opt/naoqi_sdk/share/cmake"; ${CMAKE_PREFIX_PATH})

# Create a executable named hello
# with the source file: main.cpp
qi_create_bin(hello "main.cpp")

# Add dependencies to created bin
qi_use_lib(hello ALCOMMON ALPROXIES)

# Add a simple test:
enable_testing()
qi_create_test(test_hello "test.cpp")
