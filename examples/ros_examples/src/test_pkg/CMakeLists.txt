cmake_minimum_required(VERSION 3.0.2)
project(test_pkg)
add_compile_options(-std=c++14)
# Catkin provides flags to the cpp compilers
# Other catkin packages (such as roscpp) can be added after COMPONENTS
# find_package are directly what you need. 
find_package(catkin REQUIRED COMPONENTS 
    roscpp  # need this for everything
    geometry_msgs
    )

# catkin_package is what your packages in find_package() depends on (e.g., boost)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES test_pkg
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)
# Specify locations for headers!
include_directories(${catkin_INCLUDE_DIRS})
add_executable(${PROJECT_NAME}_pub_sub_node src/test_pubsub.cpp)
# Linking executable to packages in find_package
target_link_libraries(${PROJECT_NAME}_pub_sub_node
  ${catkin_LIBRARIES}
)

