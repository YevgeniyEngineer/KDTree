cmake_minimum_required(VERSION 3.18 FATAL_ERROR)

project(kdtree)

# Compillation instructions
# cd build
# cmake -DCMAKE_BUILD_TYPE=Release ..
# make 

set(BUILD_TESTS True)

set(CXX_STANDARD 17)
set(CXX_STANDARD_REQUIRED ON)

find_package(TBB REQUIRED)

add_executable(${PROJECT_NAME} 
    ${CMAKE_CURRENT_SOURCE_DIR}/examples/simple_example.cpp
)

target_include_directories(${PROJECT_NAME}
    INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(${PROJECT_NAME}
    TBB::tbb
)

if (BUILD_TESTS)
    find_package(GTest REQUIRED)

    enable_testing()

    set(TEST_NAME ${PROJECT_NAME}_test)

    include(GoogleTest)

    add_executable(${TEST_NAME}
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/kdtree_test.cpp
    )

    target_include_directories(${TEST_NAME}
        PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/include
    )

    target_link_libraries(${TEST_NAME}
        GTest::gtest_main
        TBB::tbb
    )

    gtest_discover_tests(${TEST_NAME})
endif()