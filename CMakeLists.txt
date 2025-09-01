cmake_minimum_required(VERSION 3.8)
project(hil_serl_hiro_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Trova le dipendenze
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Configura la generazione di messaggi
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SimulationState.msg"
  "msg/RealState.msg"
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs
)

################# CPP #################

# Include Cpp "include" directory
include_directories(include)

# Crea l'eseguibile C++ (se necessario)
add_executable(cpp_executable src/cpp_node.cpp)
ament_target_dependencies(cpp_executable rclcpp)

# Installa gli eseguibili C++
install(TARGETS
  cpp_executable
  DESTINATION lib/${PROJECT_NAME}
)

################# PYTHON #################

# Installa gli script Python manualmente
install(DIRECTORY
  src scripts
  DESTINATION share/${PROJECT_NAME}
)

# Installa gli script Python come eseguibili
install(PROGRAMS
  scripts/Xbox_controller/get_gamepad_input.py 
  scripts/Xbox_controller/UR_joystick_move.py 
  scripts/Xbox_controller/UR_joystick_wRotation_move.py
  scripts/ForceBridgeNode.py
  scripts/StateBridgeNode.py
  scripts/RealStateBridgeNode.py
  DESTINATION lib/${PROJECT_NAME}
)

# Installa i file di configurazione YAML
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

# Installa i file di launch
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
