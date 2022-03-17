cmake_minimum_required(VERSION 3.10.2)

project(pam_models)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED on)


################
# Dependencies #
################

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(mpi_cmake_modules REQUIRED)
find_package(json_helper REQUIRED)
find_package(pam_configuration REQUIRED)
ament_export_dependencies(json_helper)

# prepare to export all needed targets
set(all_targets)
set(all_target_exports)


###########
# Library #
###########

add_library( ${PROJECT_NAME} SHARED
  src/hill/internal/parallel_elastic_element.cpp
  src/hill/internal/serial_elastic_element.cpp
  src/hill/internal/serial_damping_element.cpp
  src/hill/internal/contractile_element.cpp
  src/hill/brents.cpp
  src/hill/configuration.cpp
  src/hill/factory.cpp
  src/hill/muscle.cpp
  src/hill/internal/deprecated/deprecated_muscle.cpp)
target_include_directories(
  ${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME} json_helper)
target_link_libraries(${PROJECT_NAME} json_helper::json_helper)
target_link_libraries(${PROJECT_NAME} pam_configuration::pam_configuration)
ament_export_interfaces(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
list(APPEND all_targets ${PROJECT_NAME})
list(APPEND all_target_exports export_${PROJECT_NAME})


######################
# Python Native code #
######################

ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR python/${PROJECT_NAME})


######################
# Install and export #
######################
install(DIRECTORY include/ DESTINATION include)
install(
  TARGETS ${all_targets}
  EXPORT ${all_target_exports}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include)
ament_package()


##############
# Unit tests #
##############

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_${PROJECT_NAME}_cpp tests/main.cpp tests/unit_tests.cpp)
  target_link_libraries(test_${PROJECT_NAME}_cpp ${PROJECT_NAME})
endif()

#################
# Documentation #
#################

add_documentation()
