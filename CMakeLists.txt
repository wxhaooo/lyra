cmake_minimum_required(VERSION 3.0)
project(lyra)

set(CMAKE_CXX_STANDARD 14)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

find_package(Boost QUIET REQUIRED COMPONENTS system filesystem)
find_package(Eigen3 REQUIRED)
find_package(glfw3 QUIET REQUIRED)
find_package(GLEW QUIET REQUIRED)
find_package(PNG QUIET REQUIRED)
find_package(OpenGL QUIET REQUIRED)
find_package(OpenMP QUIET REQUIRED)

# message("GLEW FOUND:${GLEW_FOUND}")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")

add_definitions(-D_CRT_NO_VA_START_VALIDATION)
add_definitions(-DSTB_IMAGE_IMPLEMENTATION)

add_subdirectory(${PROJECT_SOURCE_DIR}/deps/alglib)
add_subdirectory(${PROJECT_SOURCE_DIR}/deps/jsoncpp)

set(ALGLIB_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/deps/alglib/cpp/src)
set(JSON_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/deps/jsoncpp/include)

include_directories(${Boost_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${GLFW3_INCLUDE_DIRS})
include_directories(${GLEW32_INCLUDE_DIRS})
include_directories(${PNG_INCLUDE_DIRS})

include_directories(${JSON_INCLUDE_DIR})
# message("${JSON_INCLUDE_DIR}")
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${ALGLIB_INCLUDE_DIR})


# message("PROJECT_SOURCE_DIR：${PROJECT_SOURCE_DIR}")

file(GLOB lyra_SOURCES "src/*.cpp")

add_executable(lyra ${lyra_SOURCES})
# message("PROJECT_SOURCE_DIR：${LyraThinShellSimulator_SOURCES}")
target_link_libraries(lyra ${OPENGL_LIBRARIES} ${Boost_LIBRARIES} ${GLEW_LIBRARIES} ${PNG_LIBRARIES} glfw json alglib)
# target_link_libraries(LyraThinShellSimulator ${OPENGL_LIBRARIES} ${Boost_LIBRARIES} ${GLEW_LIBRARIES} ${PNG_LIBRARIES} glfw json alglib)

