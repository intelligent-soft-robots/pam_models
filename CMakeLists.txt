cmake_minimum_required(VERSION 2.8.7)

project(pam_models)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
  mpi_cmake_modules
  json_helper
)

catkin_package(
 LIBRARIES pam_models
 INCLUDE_DIRS include internal
 CATKIN_DEPENDS mpi_cmake_modules json_helper
)

DEFINE_OS()

include_directories(include internal ${catkin_INCLUDE_DIRS})

# absolute path to default json configuration file
get_filename_component(JSON_CONFIG_DIR config ABSOLUTE)
set(JSON_DEFAULT_CONFIG_FILE "${JSON_CONFIG_DIR}/hill.json")
set(JSON_TEST_CONFIG_FILE "${JSON_CONFIG_DIR}/hill_test.json")
set(JSON_DEPRECATED_CONFIG_FILE "${JSON_CONFIG_DIR}/deprecated/muscle_params_orig.json")

# to be sure that linkage remains after install
set(CMAKE_SKIP_BUILD_RPATH  FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_INSTALL_RPATH ${CMAKE_INSTALL_PREFIX}/lib)


add_library(pam_models
  src/hill/brents.cpp
  src/hill/internal/parallel_elastic_element.cpp
  src/hill/internal/serial_elastic_element.cpp
  src/hill/internal/serial_damping_element.cpp
  src/hill/internal/contractile_element.cpp
  src/hill/muscle.cpp
  src/hill/factory.cpp
  src/hill/internal/deprecated/deprecated_muscle.cpp)
target_link_libraries(pam_models ${catkin_LIBRARIES})
target_compile_definitions(pam_models PRIVATE
  JSON_DEFAULT_CONFIG_FILE="${JSON_DEFAULT_CONFIG_FILE}")

catkin_python_setup()

catkin_add_gtest(pam_models_unit_tests
  tests/main.cpp
  tests/unit_tests.cpp)
target_link_libraries(pam_models_unit_tests pam_models ${catkin_libraries})
target_compile_definitions(pam_models_unit_tests PRIVATE
  JSON_TEST_CONFIG_FILE="${JSON_TEST_CONFIG_FILE}"
  JSON_DEFAULT_CONFIG_FILE="${JSON_DEFAULT_CONFIG_FILE}"
  JSON_DEPRECATED_CONFIG_FILE="${JSON_DEPRECATED_CONFIG_FILE}" )



#build_sphinx_documentation()
