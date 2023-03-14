cmake_minimum_required(VERSION 3.8)
project(cuda_pointpillars_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(autoware_auto_perception_msgs REQUIRED)
find_package(CUDA REQUIRED)

set(CMAKE_C_COMPILER /usr/bin/gcc)
set(CMAKE_CXX_COMPILER /usr/bin/g++)
set(CUDA_INSTALL_TARGET_DIR targets/x86_64-linux)
set(CUDA_TOOLKIT_ROOT_DIR /usr/local/cuda)
set(CUDA_INCLUDE_DIRS ${CUDA_TOOLKIT_ROOT_DIR}/${CUDA_INSTALL_TARGET_DIR}/include)

set( SMS 30 32 35 37 50 52 53 60 61 62 70 72 75 87)
foreach(sm ${SMS})
	set(GENCODE ${GENCODE} -gencode arch=compute_${sm},code=sm_${sm})
endforeach()
set(HIGHEST_SM 87)
set(GENCODE ${GENCODE} -gencode arch=compute_${HIGHEST_SM},code=compute_${HIGHEST_SM})

set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS}
    -ccbin ${CMAKE_CXX_COMPILER}
    -Xcompiler -DWIN_INTERFACE_CUSTOM
    -Xcompiler -I/usr/aarch64-linux-gnu/include/
    -Xlinker -lsocket
    -Xlinker -rpath=/usr/lib/aarch64-linux-gnu/
    -Xlinker -rpath=/usr/aarch64-linux-gnu/lib/
    -Xlinker -L/usr/lib/aarch64-linux-gnu/
    -Xlinker -L/usr/aarch64-linux-gnu/lib/
)

# set(TENSORRT_INCLUDE_DIRS /home/e404/TensorRT-8.4.3.1/include/)
# set(TENSORRT_LIBRARY_DIRS /home/e404/TensorRT-8.4.3.1/lib)
set(TENSORRT_INCLUDE_DIRS /usr/include/x86_64-linux-gnu/)
set(TENSORRT_LIBRARY_DIRS /usr/lib/x86_64-linux-gnu/)


set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-Wno-deprecated-declarations -O2")
add_compile_options(-W)
add_compile_options(-std=c++14)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# add_executable(pc_info src/pc_info.cpp)
cuda_add_executable(pc_process src/pc_process.cpp src/cuda_pp_ros.cpp
src/cuda_pp_ros.cpp
src/pillarScatter.cpp
src/pointpillar.cpp
src/postprocess.cpp
src/preprocess.cpp

src/postprocess_kernels.cu
src/pillarScatterKernels.cu
src/preprocess_kernels.cu
)

# cuda_add_library(cuda_pp_ros 

# src/cuda_pp_ros.cpp
# src/pillarScatter.cpp
# src/pointpillar.cpp
# src/postprocess.cpp
# src/preprocess.cpp

# src/postprocess_kernels.cu
# src/pillarScatterKernels.cu
# src/preprocess_kernels.cu
# )

# add_library(pillarScatter SHARED src/pillarScatter.cpp)
# add_library(pillarScatterKernels SHARED src/pillarScatterKernels.cu)
# add_library(pointpillar SHARED src/pointpillar.cpp)
# add_library(postprocess_kernels SHARED src/postprocess_kernels.cu)
# add_library(postprocess SHARED src/postprocess.cpp)
# add_library(preprocess SHARED src/preprocess.cpp)
# add_library(preprocess_kernels SHARED src/preprocess_kernels.cu)

# file(GLOB_RECURSE SOURCE_FILES
#     src/*.cu
#     src/*.cpp
# )


# target_include_directories(pc_info PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
target_include_directories(pc_process PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${CUDA_INCLUDE_DIRS}
  ${TENSORRT_INCLUDE_DIRS}
  )
# target_include_directories(cuda_pp_ros PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
#   ${CUDA_INCLUDE_DIRS}
#   ${TENSORRT_INCLUDE_DIRS})

include_directories(
  ${CUDA_INCLUDE_DIRS}
  ${TENSORRT_INCLUDE_DIRS}
  include
)
# target_compile_features(pc_info PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_features(pc_process PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

# ament_target_dependencies(pc_info    rclcpp sensor_msgs autoware_auto_perception_msgs)
ament_target_dependencies(pc_process rclcpp sensor_msgs autoware_auto_perception_msgs CUDA
)
# ament_target_dependencies(cuda_pp_ros 
#   sensor_msgs 
#   CUDA    
# )

target_link_libraries(pc_process
    libnvinfer.so
    libnvonnxparser.so
)
link_directories(
	${TENSORRT_LIBRARY_DIRS}
)

# install(TARGETS pc_info
#   DESTINATION lib/${PROJECT_NAME})
install(TARGETS pc_process
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
