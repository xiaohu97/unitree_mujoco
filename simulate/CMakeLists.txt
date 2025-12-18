cmake_minimum_required(VERSION 3.16)
project(unitree_mujoco)

enable_language(C)
enable_language(CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 推荐：用 find_package(Threads) 而不是手写 pthread
find_package(Threads REQUIRED)

list(APPEND CMAKE_PREFIX_PATH "/opt/unitree_robotics/lib/cmake")
find_package(mujoco REQUIRED)
find_package(unitree_sdk2 REQUIRED)

file(GLOB SIM_SRC
    src/joystick/joystick.cc
    src/mujoco/*.cc
    src/unitree_sdk2_bridge/*.cc
)

# ---- main sim ----
add_executable(unitree_mujoco ${SIM_SRC} src/main.cc)
target_link_libraries(unitree_mujoco PRIVATE
    Threads::Threads
    mujoco::mujoco
    glfw
    yaml-cpp
    unitree_sdk2
)

# ---- test ----
add_executable(test test/test_unitree_sdk2.cpp)
target_link_libraries(test PRIVATE unitree_sdk2)

# ---- g1_identifiaction ----
add_executable(g1_identifiaction test/g1_identifiaction.cc)
target_link_libraries(g1_identifiaction PRIVATE unitree_sdk2)

# ---- joystick test ----
add_executable(jstest src/joystick/jstest.cc src/joystick/joystick.cc)
# 如果 jstest 不依赖 unitree_sdk2/mujoco，可不链；否则按需补

set(CMAKE_BUILD_TYPE Release)
