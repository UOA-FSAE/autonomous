project(test)
cmake_minimum_required(VERSION 3.22)


set(Boost_USE_STATIC_LIBS OFF)

# Specify the Boost version required (you can adjust this depending on your Boost version)
find_package(Boost 1.74 REQUIRED COMPONENTS unit_test_framework)

# Enable testing
enable_testing()

file(GLOB SRC_FILES  
    ${PROJECT_SOURCE_DIR}/../src/*.cpp
)
file(GLOB INCLUDE_FILES  
    ${PROJECT_SOURCE_DIR}/../src/*.hpp
)

if(Boost_FOUND)
    message(STATUS "BOOST PACKAGE FOUND")
    message(STATUS "Boost_INCLUDE_DIRS " ${Boost_INCLUDE_DIRS})
    message(STATUS "Boost_UNIT_TEST_FRAMEWORK_LIBRARY " ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
endif()

foreach(ITEM IN LISTS SRC_FILES)
    message(STATUS "source file: ${ITEM}")
endforeach()
foreach(ITEM IN LISTS INCLUDE_FILES)
    message(STATUS "include file: ${ITEM}")
endforeach()

add_executable(planning_unit_tests 
    track_test.cpp
    ${SRC_FILES}
    
)

target_include_directories(planning_unit_tests PUBLIC 
    ${PROJECT_SOURCE_DIR}/../src/
    ~/boost-ver/boost_1_87_0/boost/**
)

target_link_libraries(planning_unit_tests
    ${Boost_INCLUDE_DIRS}
    # Boost::UnitTestFramework
    ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY}
    track_utils_lib
)

# Add a CTest command
add_test(NAME MyClassTest COMMAND planning_unit_tests)