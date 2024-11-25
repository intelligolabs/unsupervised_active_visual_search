cmake_minimum_required(VERSION 3.0)
project(pomp_be_pd)

# Set source files
set(SRC_DIR ${CMAKE_SOURCE_DIR}/src)
set(INCLUDE_DIR ${SRC_DIR}/include)

# Set target
set(TARGET_DIR ${CMAKE_SOURCE_DIR}/target)
set(TARGET_NAME pomp_be_pd)

# Find all source and header files
file(GLOB_RECURSE HEADER_FILES ${INCLUDE_DIR}/*.h)
file(GLOB_RECURSE SOURCE_FILES ${SRC_DIR}/*.cpp)

find_package(Boost REQUIRED COMPONENTS program_options)


# Add executable
add_executable(${TARGET_NAME} ${SRC_DIR}/main.cpp ${SOURCE_FILES} ${HEADER_FILES})

if(Boost_FOUND)
    include_directories(${Boost_INCLUDE_DIRS})
    target_link_libraries(${TARGET_NAME} ${Boost_LIBRARIES})
else()
    message(FATAL_ERROR "Boost not found. Please install Boost or check your Boost installation.")
endif()

# Include directories
target_include_directories(${TARGET_NAME} PRIVATE ${INCLUDE_DIR})

# Set output directory
set_target_properties(${TARGET_NAME} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${TARGET_DIR}
)
