cmake_minimum_required(VERSION 3.10.2)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(main)

find_package(MAVSDK REQUIRED)
add_executable(${PROJECT_NAME}
    main.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE include)
target_link_libraries(${PROJECT_NAME}
    PRIVATE
    MAVSDK::mavsdk
    pthread
)


if(NOT MSVC)
    target_compile_options(main PRIVATE -Wall -Wextra)
else()
    target_compile_options(main PRIVATE -W2)
endif()
