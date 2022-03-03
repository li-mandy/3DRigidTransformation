# CMakeList.txt : CMake project for 3DRigidTransformation, include source and define
# project specific logic here.
#
cmake_minimum_required (VERSION 3.8)
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
project ("3DRigidTransformation")

# Add source to this project's executable.
add_executable (3DRigidTransformation "3DRigidTransformation.cpp" "3DRigidTransformation.h")
target_link_libraries( 3DRigidTransformation ${OpenCV_LIBS} )
# TODO: Add tests and install targets if needed.