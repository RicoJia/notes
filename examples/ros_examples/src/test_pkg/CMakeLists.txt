cmake_minimum_required(VERSION 3.0.2)
project(test_pkg)
add_compile_options(-std=c++14)
# Catkin provides flags to the cpp compilers Other catkin packages (such as
# roscpp) can be added after COMPONENTS find_package are directly what you need.
find_package(
  catkin REQUIRED
  COMPONENTS roscpp # need this for everything
             geometry_msgs tf2 tf2_ros message_generation std_msgs)
find_package(Eigen3 REQUIRED)

add_message_files(FILES StringList.msg)

add_service_files(FILES Add2Nums.srv)
# still need this for srv
generate_messages(DEPENDENCIES std_msgs)

# catkin_package is what your packages in find_package() depends on (e.g.,
# boost). Everything in catkin_package should go into package.xml
catkin_package(
  # INCLUDE_DIRS include LIBRARIES test_pkg CATKIN_DEPENDS std_msgs DEPENDS
  # system_lib
)
# Specify locations for headers!
include_directories(${catkin_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIR})
add_executable(${PROJECT_NAME}_pub_sub_node src/test_pubsub.cpp)
# Linking executable to packages in find_package
target_link_libraries(${PROJECT_NAME}_pub_sub_node ${catkin_LIBRARIES})
add_executable(${PROJECT_NAME}_tf_node src/test_tf.cpp)
# Linking executable to packages in find_package
target_link_libraries(${PROJECT_NAME}_tf_node ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_test_sub_spin_node src/test_sub_spin.cpp)
target_link_libraries(${PROJECT_NAME}_test_sub_spin_node ${catkin_LIBRARIES})
