add_library(thruster SHARED thruster.cpp)
add_dependencies(thruster ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(thruster ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

add_library(particle_filter_visualizer SHARED particle_filter_visualizer.cpp)
add_dependencies(particle_filter_visualizer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(particle_filter_visualizer ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})


#simulator bridge
add_executable(simulator_bridge simulator_bridge.cpp)
add_dependencies(simulator_bridge ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(simulator_bridge ${catkin_LIBRARIES})
