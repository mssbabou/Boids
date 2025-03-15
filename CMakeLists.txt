cmake_minimum_required(VERSION 3.14)
project(Boids LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Find SDL3 and SDL3_image
find_package(SDL3 REQUIRED CONFIG)
find_package(SDL3_image REQUIRED CONFIG)

# Source files
file(GLOB_RECURSE SOURCES "src/*.cpp")

# Define executable
add_executable(Boids ${SOURCES})

# Link libraries
target_link_libraries(Boids PRIVATE SDL3::SDL3 SDL3_image::SDL3_image)

# Enable compiler warnings & optimizations
if (CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(Boids PRIVATE -Wall -Wextra -Wpedantic -O2)
endif()

# --- NEW PART BELOW ---

# Copy the assets folder to build directory
add_custom_command(
    TARGET Boids POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
            ${CMAKE_CURRENT_SOURCE_DIR}/assets
            $<TARGET_FILE_DIR:Boids>/assets
)
