project(Path_Planning)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

# For coverage
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/CMakeModules/")
include(CodeCoverage)
append_coverage_compiler_flags()


set(sources src/vehicle.cpp src/cost_function.cpp src/path_planner.cpp src/main.cpp)


if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

include_directories(/usr/local/include)
include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/lib)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1.11.0/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 


add_executable(path_planning ${sources})

target_link_libraries(path_planning z ssl uv uWS)



if(NOT without-test)
    enable_testing()

    set(vehicle_spec_sources test/vehicle_spec.cpp src/vehicle.cpp)
    add_executable(VehicleSpec ${vehicle_spec_sources})
    add_test(
        NAME VehicleSpec
        COMMAND $<TARGET_FILE:VehicleSpec>
    )

    set(path_planner_spec_sources test/path_planner_spec.cpp src/vehicle.cpp src/cost_function.cpp src/path_planner.cpp)
    add_executable(PathPlannerSpec ${path_planner_spec_sources})
    add_test(
        NAME PathPlannerSpec
        COMMAND $<TARGET_FILE:PathPlannerSpec>
    )
endif()