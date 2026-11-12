cmake_minimum_required(VERSION 3.22)
project(curobo_bringup LANGUAGES NONE)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(curobo_bringup)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_robot_config test/test_robot_config.py)

  ament_add_pytest_test(test_robot_stack test/test_robot_stack.py TIMEOUT 120)
endif()

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()
