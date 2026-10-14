cmake_minimum_required(VERSION 3.8)
project(pkg_rob_dif)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Instalamos las carpetas del paquete y el nodo de cinematica
install(DIRECTORY urdf worlds config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/cinematica_llantas.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
