cmake_minimum_required(VERSION 3.16)

project(inertial_core VERSION 0.1.0 LANGUAGES CXX)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

option(INERTIAL_CORE_BUILD_TESTS "Build inertial_core tests" ON)
option(INERTIAL_CORE_BUILD_EXAMPLES "Build inertial_core examples" ON)
option(
    INERTIAL_CORE_BUILD_YAML_CONFIG
    "Build and install the YAML runtime configuration target"
    ON
)

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

set(INERTIAL_CORE_HAS_YAML_CONFIG OFF)
if(INERTIAL_CORE_BUILD_YAML_CONFIG)
    find_package(yaml-cpp CONFIG REQUIRED)
    add_library(inertial_core_yaml INTERFACE)
    add_library(inertial_core::yaml ALIAS inertial_core_yaml)
    if(TARGET yaml-cpp::yaml-cpp)
        set(INERTIAL_CORE_YAML_TARGET yaml-cpp::yaml-cpp)
    elseif(TARGET yaml-cpp)
        set(INERTIAL_CORE_YAML_TARGET yaml-cpp)
    else()
        message(FATAL_ERROR "yaml-cpp CMake target was not found")
    endif()
    target_link_libraries(
        inertial_core_yaml
        INTERFACE
            inertial_core::inertial_core
            ${INERTIAL_CORE_YAML_TARGET}
    )
    set(INERTIAL_CORE_HAS_YAML_CONFIG ON)
endif()

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

    if(INERTIAL_CORE_HAS_YAML_CONFIG)
        add_executable(
            test_runtime_config_yaml
            tests/test_runtime_config_yaml.cpp
        )
        target_link_libraries(
            test_runtime_config_yaml
            PRIVATE
                inertial_core::yaml
        )
        target_compile_definitions(
            test_runtime_config_yaml
            PRIVATE
                RUNTIME_CONFIG_FILE="${CMAKE_CURRENT_SOURCE_DIR}/config/runtime.yaml"
        )
        add_test(
            NAME inertial_core_yaml_config_tests
            COMMAND test_runtime_config_yaml
        )
    endif()
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
if(INERTIAL_CORE_HAS_YAML_CONFIG)
    install(TARGETS inertial_core_yaml
        EXPORT inertial_coreTargets
    )
endif()
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(FILES config/runtime.yaml
    DESTINATION ${CMAKE_INSTALL_DATADIR}/inertial_core/config
)
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
