cmake_minimum_required(VERSION 3.14)
project(Boids LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# MinGW-Specific Settings
if (MINGW)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -static -std=c++17")
endif()

# Find SDL3 and SDL3_image
find_package(SDL3 REQUIRED CONFIG)
find_package(SDL3_image REQUIRED CONFIG)

# Collect your project sources from src/
file(GLOB_RECURSE SOURCES "src/*.cpp")

# Collect ImGui sources from the external folder
file(GLOB IMGUI_SOURCES 
    "${CMAKE_CURRENT_SOURCE_DIR}/external/imgui/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/external/imgui/backends/imgui_impl_sdl3.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/external/imgui/backends/imgui_impl_opengl3.cpp"
)

# Add executable including both your sources and ImGui files
add_executable(Boids ${SOURCES} ${IMGUI_SOURCES})

# Add include directories for ImGui and its backends
target_include_directories(Boids PRIVATE 
    "${CMAKE_CURRENT_SOURCE_DIR}/external/imgui"
    "${CMAKE_CURRENT_SOURCE_DIR}/external/imgui/backends"
)

target_link_libraries(Boids PRIVATE opengl32)

target_link_libraries(Boids PRIVATE SDL3::SDL3 SDL3_image::SDL3_image)

if (CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(Boids PRIVATE -Wall -Wextra -Wpedantic)
endif()

if (MINGW)
    target_link_options(Boids PRIVATE -static-libgcc -static-libstdc++)
endif()

# --- Copy assets folder to build directory ---
add_custom_command(
    TARGET Boids POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
            ${CMAKE_CURRENT_SOURCE_DIR}/assets
            $<TARGET_FILE_DIR:Boids>/assets
)

# --- Copy SDL3 DLLs to build directory on Windows ---
if (WIN32)
    # Automatically find SDL3 and SDL3_image DLLs
    get_target_property(SDL3_BIN SDL3::SDL3 IMPORTED_LOCATION)
    get_target_property(SDL3_IMAGE_BIN SDL3_image::SDL3_image IMPORTED_LOCATION)

    # Ensure they exist before copying
    if (SDL3_BIN)
        add_custom_command(
            TARGET Boids POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy
                    ${SDL3_BIN}
                    $<TARGET_FILE_DIR:Boids>
        )
    endif()

    if (SDL3_IMAGE_BIN)
        add_custom_command(
            TARGET Boids POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy
                    ${SDL3_IMAGE_BIN}
                    $<TARGET_FILE_DIR:Boids>
        )
    endif()
endif()
