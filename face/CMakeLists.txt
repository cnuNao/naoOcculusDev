cmake_minimum_required(VERSION 2.8)
project(face)

find_package(qibuild)

# define source files
set(_srcs
	main.cpp
	onfacedetection.h
	onfacedetection.cpp
)

# Create a executable named face
# with the source file: main.cpp
qi_create_bin(face ${_srcs})

qi_use_lib(face ALCOMMON ALPROXIES ALTHREAD)

# Add a simple test:
enable_testing()
qi_create_test(test_face "test.cpp")
