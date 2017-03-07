cmake_minimum_required(VERSION 3.0)

set(lib_name "samo")
set(lib_test_name ${lib_name}_unit_tests)

project(${lib_name})

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

#############
### Files #####
#############
file(GLOB_RECURSE sources src/*.cpp src/*.h)
file(GLOB_RECURSE sources_test test/*.cpp test/*.h)
file(GLOB_RECURSE resources resources/*)

SET(CMAKE_C_FLAGS_DEBUG "/MTd")
SET(CMAKE_CXX_FLAGS_DEBUG "/MTd")

add_subdirectory(libs)

#############
### Lib #####
#############
add_library(${lib_name} ${sources})
target_include_directories(${lib_name} PUBLIC src libs/Eigen)
file(COPY ${resources} DESTINATION resources)

if (MSVC)
  add_definitions("/MP -D_SCL_SECURE_NO_WARNINGS /W4 /WX /wd4005 /wd4127 /wd4481 /wd4482 /wd4505 /wd4512 /wd4714 /wd4800")
  add_definitions("/wd4189")
endif()

set_target_properties(${lib_name} PROPERTIES ENABLE_EXPORTS on)

#############
### Tests ###
#############
add_executable(${lib_test_name} ${sources_test}  ${resources})
target_link_libraries(${lib_test_name} PRIVATE ${lib_name} gtest)

set_target_properties(${lib_name} ${lib_test_name} PROPERTIES FOLDER ${lib_name})
set_target_properties(gmock gmock_main gtest gtest_main PROPERTIES FOLDER "google")

#############
## Install ##
#############
install(TARGETS ${lib_name}  DESTINATION ${lib_name}_destination)

set(CPACK_PACKAGE_NAME ${lib_name})
set(CPACK_PACKAGE_VERSION "1.0.0")
set(CPACK_MONOLITHIC_INSTALL 1)

include(CPack)
