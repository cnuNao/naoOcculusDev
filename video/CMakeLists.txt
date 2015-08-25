cmake_minimum_required(VERSION 2.8)
project(video)

find_package(qibuild)

# Create a executable named video
# with the source file: main.cpp

set(_srcs
	main.cpp
	videocapture.h
	videocapture.cpp
)

qi_create_bin(video ${_srcs})
qi_use_lib(video ALCOMMON ALPROXIES ALTHREAD ALVISION)

# Add a simple test:
enable_testing()
qi_create_test(test_video "test.cpp")
