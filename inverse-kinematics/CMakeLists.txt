cmake_minimum_required(VERSION 2.8)
project(Get_inverse_joints)

#set(CMAKE_CXX_STANDARD 11)
set( CMAKE_CXX_FLAGS "-std=c++11" )
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -std=c++11") ##？？？？
set(CMAKE_BUILD_TYPE Release)
	
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

find_package(Eigen3 REQUIRED NO_MODULE)

include_directories(
	${EIGEN3_INCLUDE_DIR}
	${PROJECT_SOURCE_DIR}/include

)

add_executable(
	Get_inverse_joints 
	src/CCartesian.cpp
	src/CRoboticConstraints.cpp
	src/Netown_inverse.cpp
)

target_link_libraries(
   Get_inverse_joints
   Eigen3::Eigen
 )
