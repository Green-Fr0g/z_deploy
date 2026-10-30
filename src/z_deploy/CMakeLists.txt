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

# Internal headers are included as <fsm/...>, <utils/...>, <types.hpp>, etc.
# from these two roots; no symlinked include/z_deploy namespace is needed.
set(Z_DEPLOY_INCLUDE_DIRS
  ${CMAKE_CURRENT_SOURCE_DIR}/core
  ${CMAKE_CURRENT_SOURCE_DIR}/include)

set(POLICY_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../policy")
set(INFERENCE_RUNTIME_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../thirdparty/interface_runtime")

# The runtime setup script places the optional inference dependencies here.
# Keep the hardware monitor buildable without them, while linking them
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

add_executable(z_rl_deploy src/main.cpp)
add_executable(test_hardware test/test_hardware.cpp)
foreach(deploy_target IN ITEMS z_rl_deploy test_hardware)
  target_compile_definitions(${deploy_target} PRIVATE Z_POLICY_ROOT="${POLICY_ROOT}")
  target_include_directories(${deploy_target} PRIVATE
    ${Z_DEPLOY_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/library/joystick)
  ament_target_dependencies(${deploy_target} rclcpp std_srvs z_msgs)
  target_link_libraries(${deploy_target} yaml-cpp z_deploy_joystick Threads::Threads)
endforeach()

set(Z_DEPLOY_RUNTIME_RPATH)
option(Z_DEPLOY_ENABLE_UNITREE "Enable Unitree SDK2 real hardware backends" ON)
set(Z_UNITREE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/library/unitree_sdk2")
set(Z_UNITREE_DDS_DIR "${Z_UNITREE_ROOT}/thirdparty/lib/${CMAKE_SYSTEM_PROCESSOR}")
if(Z_DEPLOY_ENABLE_UNITREE)
  find_library(Z_UNITREE_SDK_LIBRARY unitree_sdk2
    PATHS "${Z_UNITREE_ROOT}/lib/${CMAKE_SYSTEM_PROCESSOR}" NO_DEFAULT_PATH)
  find_library(Z_UNITREE_DDS_LIBRARY ddsc PATHS "${Z_UNITREE_DDS_DIR}" NO_DEFAULT_PATH)
  find_library(Z_UNITREE_DDSXX_LIBRARY ddscxx PATHS "${Z_UNITREE_DDS_DIR}" NO_DEFAULT_PATH)
endif()

if(Z_DEPLOY_ENABLE_UNITREE AND Z_UNITREE_SDK_LIBRARY AND
   Z_UNITREE_DDS_LIBRARY AND Z_UNITREE_DDSXX_LIBRARY)
  add_library(z_deploy_unitree_sdk2 STATIC IMPORTED)
  set_target_properties(z_deploy_unitree_sdk2 PROPERTIES
    IMPORTED_LOCATION "${Z_UNITREE_SDK_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${Z_UNITREE_ROOT}/include;${Z_UNITREE_ROOT}/thirdparty/include;${Z_UNITREE_ROOT}/thirdparty/include/ddscxx"
    INTERFACE_LINK_LIBRARIES "${Z_UNITREE_DDSXX_LIBRARY};${Z_UNITREE_DDS_LIBRARY};Threads::Threads")
  foreach(deploy_target IN ITEMS z_rl_deploy test_hardware)
    target_link_libraries(${deploy_target} z_deploy_unitree_sdk2)
    target_compile_definitions(${deploy_target} PRIVATE Z_DEPLOY_HAS_UNITREE=1)
  endforeach()
  list(APPEND Z_DEPLOY_RUNTIME_RPATH "${Z_UNITREE_DDS_DIR}" "$ORIGIN/..")
  # SDK2 binaries use versioned SONAMEs even though the bundled files omit .0.
  install(FILES "${Z_UNITREE_DDS_LIBRARY}" DESTINATION lib RENAME libddsc.so.0)
  install(FILES "${Z_UNITREE_DDSXX_LIBRARY}" DESTINATION lib RENAME libddscxx.so.0)
  message(STATUS "z_deploy: Unitree SDK2 real hardware enabled")
else()
  foreach(deploy_target IN ITEMS z_rl_deploy test_hardware)
    target_compile_definitions(${deploy_target} PRIVATE Z_DEPLOY_HAS_UNITREE=0)
  endforeach()
  message(STATUS "z_deploy: Unitree SDK2 disabled or unavailable; real hardware is unavailable")
endif()

if(Torch_FOUND)
  message(STATUS "z_deploy: LibTorch enabled from ${LIBTORCH_ROOT}")
  target_compile_definitions(z_rl_deploy PRIVATE Z_DEPLOY_HAS_TORCH=1)
  target_include_directories(z_rl_deploy PRIVATE ${TORCH_INCLUDE_DIRS})
  target_compile_options(z_rl_deploy PRIVATE ${TORCH_CXX_FLAGS})
  target_link_libraries(z_rl_deploy ${TORCH_LIBRARIES})
  list(APPEND Z_DEPLOY_RUNTIME_RPATH "${LIBTORCH_ROOT}/lib")
else()
  message(STATUS "z_deploy: LibTorch not found; .pt policies are unavailable")
  target_compile_definitions(z_rl_deploy PRIVATE Z_DEPLOY_HAS_TORCH=0)
endif()

if(Z_DEPLOY_ONNX_INCLUDE_DIR AND Z_DEPLOY_ONNX_LIBRARY)
  message(STATUS "z_deploy: ONNX Runtime enabled from ${ONNX_ROOT}")
  target_compile_definitions(z_rl_deploy PRIVATE Z_DEPLOY_HAS_ONNX=1)
  target_include_directories(z_rl_deploy PRIVATE ${Z_DEPLOY_ONNX_INCLUDE_DIR})
  target_link_libraries(z_rl_deploy ${Z_DEPLOY_ONNX_LIBRARY})
  list(APPEND Z_DEPLOY_RUNTIME_RPATH "${ONNX_ROOT}/lib")
else()
  message(STATUS "z_deploy: ONNX Runtime not found; .onnx policies are unavailable")
  target_compile_definitions(z_rl_deploy PRIVATE Z_DEPLOY_HAS_ONNX=0)
endif()

if(Z_DEPLOY_RUNTIME_RPATH)
  set_target_properties(z_rl_deploy test_hardware PROPERTIES
    BUILD_RPATH "${Z_DEPLOY_RUNTIME_RPATH}"
    INSTALL_RPATH "${Z_DEPLOY_RUNTIME_RPATH}")
endif()

install(TARGETS z_rl_deploy test_hardware z_deploy_joystick
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

ament_package()
