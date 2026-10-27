cmake_minimum_required(VERSION 3.16)
cmake_policy(SET CMP0009 NEW)
project(z_deploy)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(z_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Threads REQUIRED)

set(POLICY_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../policy")
set(INFERENCE_RUNTIME_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../thirdparty/interface_runtime")

# The runtime setup script places the optional inference dependencies here.
# Keep the package buildable without them (for unit tests), while linking them
# into the deployed executable whenever they are available.
set(LIBTORCH_ROOT "${INFERENCE_RUNTIME_ROOT}/libtorch")
find_package(Torch QUIET CONFIG PATHS "${LIBTORCH_ROOT}/share/cmake/Torch" NO_DEFAULT_PATH)

set(ONNX_ROOT "${INFERENCE_RUNTIME_ROOT}/onnxruntime")
find_path(Z_DEPLOY_ONNX_INCLUDE_DIR onnxruntime_cxx_api.h
  PATHS "${ONNX_ROOT}/include" NO_DEFAULT_PATH)
find_library(Z_DEPLOY_ONNX_LIBRARY onnxruntime
  PATHS "${ONNX_ROOT}/lib" NO_DEFAULT_PATH)

add_library(z_deploy_joystick STATIC library/joystick/joystick.cc)
target_include_directories(z_deploy_joystick PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/library/joystick>
  $<INSTALL_INTERFACE:include>)

add_executable(z_deploy_main src/main.cpp)
target_compile_definitions(z_deploy_main PRIVATE Z_POLICY_ROOT="${POLICY_ROOT}")
target_include_directories(z_deploy_main PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/library/joystick)
ament_target_dependencies(z_deploy_main rclcpp std_srvs z_msgs)
target_link_libraries(z_deploy_main yaml-cpp z_deploy_joystick Threads::Threads)

set(Z_DEPLOY_RUNTIME_RPATH)
if(Torch_FOUND)
  message(STATUS "z_deploy: LibTorch enabled from ${LIBTORCH_ROOT}")
  target_compile_definitions(z_deploy_main PRIVATE Z_DEPLOY_HAS_TORCH=1)
  target_include_directories(z_deploy_main PRIVATE ${TORCH_INCLUDE_DIRS})
  target_compile_options(z_deploy_main PRIVATE ${TORCH_CXX_FLAGS})
  target_link_libraries(z_deploy_main ${TORCH_LIBRARIES})
  list(APPEND Z_DEPLOY_RUNTIME_RPATH "${LIBTORCH_ROOT}/lib")
else()
  message(STATUS "z_deploy: LibTorch not found; .pt policies are unavailable")
  target_compile_definitions(z_deploy_main PRIVATE Z_DEPLOY_HAS_TORCH=0)
endif()

if(Z_DEPLOY_ONNX_INCLUDE_DIR AND Z_DEPLOY_ONNX_LIBRARY)
  message(STATUS "z_deploy: ONNX Runtime enabled from ${ONNX_ROOT}")
  target_compile_definitions(z_deploy_main PRIVATE Z_DEPLOY_HAS_ONNX=1)
  target_include_directories(z_deploy_main PRIVATE ${Z_DEPLOY_ONNX_INCLUDE_DIR})
  target_link_libraries(z_deploy_main ${Z_DEPLOY_ONNX_LIBRARY})
  list(APPEND Z_DEPLOY_RUNTIME_RPATH "${ONNX_ROOT}/lib")
else()
  message(STATUS "z_deploy: ONNX Runtime not found; .onnx policies are unavailable")
  target_compile_definitions(z_deploy_main PRIVATE Z_DEPLOY_HAS_ONNX=0)
endif()

if(Z_DEPLOY_RUNTIME_RPATH)
  set_target_properties(z_deploy_main PROPERTIES
    BUILD_RPATH "${Z_DEPLOY_RUNTIME_RPATH}"
    INSTALL_RPATH "${Z_DEPLOY_RUNTIME_RPATH}")
endif()

install(TARGETS z_deploy_main z_deploy_joystick
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY core/ DESTINATION include/z_deploy)
install(FILES
  include/config_types.hpp
  include/input_types.hpp
  include/types.hpp
  DESTINATION include/z_deploy)
install(DIRECTORY ../../policy/ DESTINATION share/${PROJECT_NAME}/policy)

if(BUILD_TESTING)
  enable_testing()
  add_executable(test_fsm test/test_fsm.cpp)
  target_include_directories(test_fsm PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_compile_definitions(test_fsm PRIVATE Z_DEPLOY_HAS_TORCH=0)
  target_link_libraries(test_fsm yaml-cpp Threads::Threads)
  add_test(NAME fsm COMMAND test_fsm)

  add_executable(test_policy_manager test/test_policy_manager.cpp)
  target_include_directories(test_policy_manager PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_compile_definitions(test_policy_manager PRIVATE Z_DEPLOY_HAS_TORCH=0)
  target_link_libraries(test_policy_manager yaml-cpp Threads::Threads)
  add_test(NAME policy_manager COMMAND test_policy_manager)
endif()

ament_package()
