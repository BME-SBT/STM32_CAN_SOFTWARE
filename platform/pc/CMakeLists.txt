cmake_minimum_required(VERSION 3.20)
project(sbt_can_sim_v1 C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug)
endif()
add_compile_options(-Wall -Wextra)

# Shared firmware lives in <repo>/src; this file builds it for the PC.
set(SRC ${CMAKE_CURRENT_SOURCE_DIR}/../../src)

# Firmware (L2 and up, built against the mock HAL) + simulated network, in one library.
add_library(sbt_sim STATIC
  ${SRC}/protocol/can_msgs.c
  ${SRC}/protocol/can_proto.c
  ${SRC}/protocol/can_ringbuf.c
  ${SRC}/middleware/fault_mgr.c
  ${SRC}/middleware/sched.c
  ${SRC}/app/control/control_node.c
  ${SRC}/app/control/pedal.c
  ${SRC}/app/control/safety.c
  ${SRC}/app/control/vehicle_fsm.c
  hal/hal_mock.c
  sim/sim_bus.c
  sim/sim_dbc.c
  sim/sim_nodes.c
  sim/sim_world.c
)
target_include_directories(sbt_sim PUBLIC
  ${SRC}/hal ${SRC}/protocol ${SRC}/middleware ${SRC}/app/control
  hal sim)
target_link_libraries(sbt_sim PUBLIC m)

# Static linking so the .exe files run without the MinGW DLLs on PATH.
if(MINGW)
  add_link_options(-static)
endif()

add_executable(cansim sim/cansim.c)
target_link_libraries(cansim sbt_sim)