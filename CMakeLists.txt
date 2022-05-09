cmake_minimum_required(VERSION 3.10)

project(autopilot)

set(CMAKE_CXX_STANDARD 11)
set (EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set( SRC_LIST
./main.cc
./tools/file_tool.cc
./modules/plan/sin_cos_plan.cc
./configs/carconfig/car_lon_config.cc
./models/carmodel/car_lon_model.cc
./configs/controllerconfig/pid_controller_config.cc
./modules/control/pid_controller.cc
./modules/control/mpc_controller.cc)

find_package(Eigen3 REQUIRED)
find_package(PythonLibs 2.7 COMPONENTS Development NumPy)
find_package(OsqpEigen)

include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})

add_executable(autopilot ${SRC_LIST})
target_link_libraries(autopilot OsqpEigen::OsqpEigen)
target_include_directories(autopilot PRIVATE ${PYTHON_INCLUDE_DIRS})
target_link_libraries(autopilot ${PYTHON_LIBRARIES})
