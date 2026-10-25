cmake_minimum_required(VERSION 3.20)
project(sbt_can_sim_v1 C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug)
endif()
add_compile_options(-Wall -Wextra)

# Firmware (L2 and up, built against the mock HAL) + simulated network, in one library.
add_library(sbt_sim STATIC
  firmware/common/can_msgs.c
  firmware/common/can_proto.c
  firmware/common/can_ringbuf.c
  firmware/common/fault_mgr.c
  firmware/common/sched.c
  firmware/app_control/control_node.c
  firmware/app_control/pedal.c
  firmware/app_control/safety.c
  firmware/app_control/vehicle_fsm.c
  firmware/hal/mock/hal_mock.c
  sim/sim_bus.c
  sim/sim_dbc.c
  sim/sim_nodes.c
  sim/sim_world.c
)
target_include_directories(sbt_sim PUBLIC
  firmware/hal firmware/hal/mock firmware/common firmware/app_control sim)
target_link_libraries(sbt_sim PUBLIC m)

# Static linking so the .exe files run without the MinGW DLLs on PATH.
if(MINGW)
  add_link_options(-static)
endif()

add_executable(cansim sim/cansim.c)
target_link_libraries(cansim sbt_sim)

