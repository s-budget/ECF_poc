cmake_minimum_required(VERSION 3.16)
project(ECF_poc)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Debug)

set(CMAKE_PREFIX_PATH "C:/Users/mpristav/local/cityflowcpp-debug;C:/Users/mpristav/local/ecf-install")

find_package(CityFlowCpp REQUIRED)

include_directories("C:/Users/mpristav/local/ecf-install/include")

add_executable(ECF_poc
        src/main.cpp
        src/include/simulator/SimulatorEngine.cpp
        src/include/agent/Evaluator.hpp
        src/include/agent/Gplightagent.hpp
)

target_include_directories(ECF_poc PRIVATE include)

target_link_libraries(ECF_poc PRIVATE
        CityFlow::cityflow_lib
        "C:/Users/mpristav/local/ecf-install/lib/libECF.a"
)