cmake_minimum_required(VERSION 2.8.3)
project(akima_smoother)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
)

catkin_package(
  INCLUDE_DIRS lib
  LIBRARIES libakima_smoother
  CATKIN_DEPENDS 
	roscpp 
	nav_msgs
)

SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall ${CMAKE_CXX_FLAGS}")

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(libakima_smoother
            lib/libakima_smoother.cpp
)
add_dependencies(libakima_smoother
        ${catkin_EXPORTED_TARGETS}
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

add_executable(demo
        src/demo.cpp
)

target_link_libraries(demo
        libakima_smoother
	${catkin_LIBRARIES}
)
add_dependencies(demo
	${catkin_EXPORTED_TARGETS}
	${${PROJECT_NAME}_EXPORTED_TARGETS}
)
