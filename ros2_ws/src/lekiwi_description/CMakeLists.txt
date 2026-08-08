cmake_minimum_required(VERSION 3.10)
project(lekiwi_description)

find_package(ament_cmake REQUIRED)

# meshes/ is optional: it only exists when the CAD STLs have been copied in for
# xacro's use_mesh:=true path. Install it only if present so the build does not
# fail on a fresh clone.
set(_install_dirs urdf)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/meshes")
  list(APPEND _install_dirs meshes)
endif()

install(DIRECTORY ${_install_dirs} DESTINATION share/${PROJECT_NAME})

ament_package()
