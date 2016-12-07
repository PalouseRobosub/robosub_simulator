add_library(thruster SHARED thruster.cpp)
target_link_libraries(thruster ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})


#simulator bridge
add_executable(simulator_bridge simulator_bridge.cpp)
add_dependencies(simulator_bridge ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(simulator_bridge ${catkin_LIBRARIES})
