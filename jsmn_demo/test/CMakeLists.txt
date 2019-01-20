# This is the project CMakeLists.txt file for the test subproject 
cmake_minimum_required(VERSION 3.5)

set(TEST_COMPONENTS "." CACHE STRING "List of components to test")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(test_jsmn_demo)
