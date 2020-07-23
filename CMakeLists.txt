cmake_minimum_required(VERSION 2.8.7)

project(pam_models)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
  mpi_cmake_modules
)

catkin_package(
 LIBRARIES pam_models
 INCLUDE_DIRS include
 CATKIN_DEPENDS mpi_cmake_modules
)

DEFINE_OS()

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(pam_models
  src/hill/brents.cpp
  src/hill/parallel_elastic_element.cpp
  src/hill/serial_elastic_element.cpp
  src/hill/serial_damping_element.cpp
  src/hill/contractile_element.cpp
  src/hill/muscle.cpp)
target_link_libraries(pam_models ${catkin_LIBRARIES})

#catkin_add_gtest(pam_interface_unit_tests
#  tests/main.cpp
#  tests/unit_tests.cpp)
#target_link_libraries(pam_interface_unit_tests pam_interface ${catkin_libraries})
#target_compile_definitions(pam_interface_unit_tests PRIVATE
#  PAM_DEFAULT_CONFIG_FILE_PATH="${JSON_DEFAULT_CONFIG_FILE}"
#  BITFILE_4DOFS_FILE_PATH="${PAM_BITFILE}")

#build_sphinx_documentation()
