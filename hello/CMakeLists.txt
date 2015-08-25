cmake_minimum_required(VERSION 2.8)
project(hello)

find_package(qibuild)

set(CMAKE_PREFIX_PATH "/usr/local/bin;/usr/bin"; ${CMAKE_PREFIX_PATH})

find_library(CURLPP curlpp)
find_library(CURL curl)

if(CURLPP)
	message("curlpp found")
	set(CURLPP_LIBRARIES ${CURLPP})
else()
	message("CURLPP not found")
	set(CURLPP_LIBRARIES "/usr/local/lib/libcurlpp.dylib;/usr/local/lib/libcurlpp.a;/usr/local/lib/libcurlpp.la")
endif()

if(CURL)
	set(CURLPP_LIBRARIES ${CURLPP_LIBRARIES};${CURL})
	message("curl found")
else()
	set(CURLPP_LIBRARIES ${CURLPP_LIBRARIES};"/usr/lib/libcurl.dylib")
endif()

# Create a executable named hello
# with the source file: main.cpp
qi_create_bin(hello "main.cpp")

# Add dependencies to created bin
qi_use_lib(hello ALCOMMON ALPROXIES)

target_link_libraries(hello LINK_PUBLIC curl curlpp)

# Add a simple test:
enable_testing()
qi_create_test(test_hello "test.cpp")
