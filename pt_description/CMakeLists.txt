cmake_minimum_required(VERSION 3.28)
project(pt_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  urdf
  meshes
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_urdf_xacro test/test_urdf_xacro.py TIMEOUT 60)
endif()

ament_package()
