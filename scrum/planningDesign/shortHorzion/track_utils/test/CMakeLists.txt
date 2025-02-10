project(test)
cmake_minimum_required(VERSION 3.22)



file(GLOB SRC_FILES  
    ${PROJECT_SOURCE_DIR}/../src/*.cpp
)
file(GLOB INCLUDE_FILES  
    ${PROJECT_SOURCE_DIR}/../src/*.hpp
)

foreach(ITEM IN LISTS SRC_FILES)
    message(STATUS "source file: ${ITEM}")
endforeach()
foreach(ITEM IN LISTS INCLUDE_FILES)
    message(STATUS "include file: ${ITEM}")
endforeach()

message(STATUS THIS SHOULD HAVE CHANGED)

add_executable(planning_unit_tests 
    track_test.cpp
    ${SRC_FILES}
)

target_include_directories(planning_unit_tests PUBLIC 
    ${PROJECT_SOURCE_DIR}/../src/
    ~/wboost-ver/boost_1_87_0/boost/**
)

# target_link_libraries(planning_unit_tests
#     PUBLIC ~/wboost-ver/boost_1_87_0/boost/**
# )