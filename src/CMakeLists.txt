add_library(thruster SHARED thruster_plugin.cpp thruster.cpp maestro_emulator.cpp)
add_dependencies(thruster ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(thruster ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} serial)

add_library(buoyancy_improved SHARED buoyancy_improved.cpp)
add_dependencies(buoyancy_improved  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(buoyancy_improved   ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

add_library(sensor SHARED bno055_emulator.cpp)
add_dependencies(sensor ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(sensor ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} serial)

#simulator bridge
add_executable(simulator_bridge simulator_bridge.cpp)
add_dependencies(simulator_bridge ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(simulator_bridge sensor ${catkin_LIBRARIES})
