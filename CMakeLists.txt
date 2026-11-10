cmake_minimum_required(VERSION 3.16)

project(inertial_core VERSION 0.1.0 LANGUAGES CXX)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

option(INERTIAL_CORE_BUILD_TESTS "Build inertial_core tests" ON)
option(INERTIAL_CORE_BUILD_EXAMPLES "Build inertial_core examples" ON)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

add_library(inertial_core
    src/earth_model.cpp
    src/mechanization.cpp
)
add_library(inertial_core::inertial_core ALIAS inertial_core)

target_compile_features(inertial_core PUBLIC cxx_std_17)
target_include_directories(inertial_core
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_link_libraries(inertial_core PUBLIC Eigen3::Eigen)

if(MSVC)
    target_compile_options(inertial_core PRIVATE /W4 /permissive-)
else()
    target_compile_options(inertial_core PRIVATE -Wall -Wextra -Wpedantic)
endif()

if(INERTIAL_CORE_BUILD_TESTS)
    enable_testing()
    add_executable(test_inertial_core tests/test_inertial_core.cpp)
    target_link_libraries(test_inertial_core PRIVATE inertial_core::inertial_core)
    add_test(NAME inertial_core_tests COMMAND test_inertial_core)
endif()

if(INERTIAL_CORE_BUILD_EXAMPLES)
    add_executable(example_pure_inertial examples/pure_inertial.cpp)
    target_link_libraries(example_pure_inertial PRIVATE inertial_core::inertial_core)
endif()

install(TARGETS inertial_core
    EXPORT inertial_coreTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(EXPORT inertial_coreTargets
    FILE inertial_coreTargets.cmake
    NAMESPACE inertial_core::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/inertial_core
)

configure_package_config_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/inertial_coreConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/inertial_coreConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/inertial_core
)
write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/inertial_coreConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/inertial_coreConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/inertial_coreConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/inertial_core
)
