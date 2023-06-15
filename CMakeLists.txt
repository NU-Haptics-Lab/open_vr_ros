cmake_minimum_required(VERSION 3.5)
project(open_vr_ros)

## set definition, path, compileflags
add_definitions(-DWIN -DWIN64)
file(TO_CMAKE_PATH $ENV{HOME} HOME) # converts \ to / for os cross-compatability
set(OPENVR ${HOME}/libraries/openvr)
add_definitions(-DOPENVR_PATH="${OPENVR}") ## for source code to get sample textures and shader files path
set(THIRDPARTY_DIR ${OPENVR}/samples/thirdparty)
set(SHARED_SRC_DIR ${OPENVR}/samples/shared)

## find openvr
find_library(OPENVR_LIBRARIES
  NAMES
    openvr_api
  PATHS
    ${OPENVR}/bin
    ${OPENVR}/lib
  PATH_SUFFIXES
    win64
  NO_DEFAULT_PATH
  NO_CMAKE_FIND_ROOT_PATH
)
set(OPENVR_INCLUDE_DIR ${OPENVR}/headers ${OPENVR}/samples)

## find SDL
set(SDL_REQUIRED_LIBRARIES SDL2)
foreach(lib ${SDL_REQUIRED_LIBRARIES})
  find_library(${lib}_LIBRARY
    NAMES
      ${lib}
    PATHS
      ${THIRDPARTY_DIR}/sdl2-2.0.3/bin
    PATH_SUFFIXES
      win64
    )
  list(APPEND SDL2_LIBRARIES ${${lib}_LIBRARY})
endforeach()
set(SDL2_INCLUDE_DIR ${THIRDPARTY_DIR}/sdl2-2.0.3/include)

## common src from openvr sample
file(GLOB SHARED_SRC_FILES
  ${SHARED_SRC_DIR}/*.cpp
  ${SHARED_SRC_DIR}/*.h
)

## find opengl (OpenGL specific)
find_package(OpenGL REQUIRED)
find_package(GLEW 1.11 REQUIRED)

## find vulkan (vulkan specific)
find_library(VULKAN_LIBRARY
  NAMES
    vulkan-1
    vulkan
  PATHS
    ${THIRDPARTY_DIR}/vulkan-1.0.49.0/lib
  PATH_SUFFIXES
    win64
  NO_DEFAULT_PATH
)
set(VULKAN_INCLUDE_DIR ${THIRDPARTY_DIR}/vulkan-1.0.49.0/include)


## debugging
message("\n\n\n" 
  ${SDL2_INCLUDE_DIR}
  ${OPENVR_INCLUDE_DIR}
  ${OPENGL_INCLUDE_DIR} ## (OpenGL specific)
  ${GLEW_INCLUDE_DIR} ## (OpenGL specific)
  ${VULKAN_INCLUDE_DIR} ## (Vulkan specific)
"\n\n\n"
)

## add include path
include_directories(
  ${CMAKE_SOURCE_DIR}/include/${PROJECT_NAME}
  ${SDL2_INCLUDE_DIR}
  ${OPENVR_INCLUDE_DIR}
  ${OPENGL_INCLUDE_DIR} ## (OpenGL specific)
  ${GLEW_INCLUDE_DIR} ## (OpenGL specific)
  ${VULKAN_INCLUDE_DIR} ## (Vulkan specific)
)

## Find colcon packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(std_srvs REQUIRED)

ament_export_include_directories(include)

# open_vr node:
add_executable(open_vr_node
  ${SHARED_SRC_FILES}
  "src/open_vr_node.cpp"
  "src/vr_interface.cpp"
)
target_include_directories(open_vr_node PUBLIC
  ${SDL2_INCLUDE_DIR}
  ${OPENVR_INCLUDE_DIR}
  ${OPENGL_INCLUDE_DIR} ## (OpenGL specific)
  ${GLEW_INCLUDE_DIR} ## (OpenGL specific)
  ${VULKAN_INCLUDE_DIR} ## (Vulkan specific)
)
## link all
target_link_libraries(open_vr_node
  ${OPENGL_LIBRARIES} ## (OpenGL specific)
  ${GLEW_LIBRARIES} ## (OpenGL specific)
  ${VULKAN_LIBRARY} ## (Vulkan specific)
  ${SDL2_LIBRARIES}
  ${OPENVR_LIBRARIES}
  ${CMAKE_DL_LIBS}
  ${EXTRA_LIBS}
)


ament_target_dependencies(open_vr_node rclcpp std_msgs tf2 tf2_ros tf2_geometry_msgs geometry_msgs cv_bridge image_transport std_srvs)


install(TARGETS open_vr_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)


ament_package()