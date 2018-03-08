add_library(thruster SHARED thruster_plugin.cpp thruster.cpp maestro_emulator.cpp)
add_dependencies(thruster ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(thruster ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} serial)

add_library(buoyancy_improved SHARED buoyancy_improved.cpp)
add_dependencies(buoyancy_improved  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(buoyancy_improved   ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

# Plugin for the Marker Dropper
add_library(markerdrop SHARED marker_dropper.cpp)
add_dependencies(markerdrop ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(markerdrop ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

# Plugin for continually spinning the roulette wheel
add_library(roulette_spinner SHARED roulette_spinner.cpp)
add_dependencies(roulette_spinner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(roulette_spinner ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

# Plugin for the Torpedo Shooter
add_library(torpedoshoot SHARED torpedo_shooter.cpp)
add_dependencies(torpedoshoot ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(torpedoshoot ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

#simulator bridge
add_executable(simulator_bridge simulator_bridge.cpp)
add_dependencies(simulator_bridge ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(simulator_bridge ${catkin_LIBRARIES})

# Localization engine diag harness/error calculator
add_executable(localization_harness localization_harness.cpp)
add_dependencies(localization_harness ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(localization_harness ${catkin_LIBRARIES})

# Plugin for Bumper
add_library(gazebo_ros_bumper bumper.cpp)
add_dependencies(gazebo_ros_bumper ${catkin_EXPORTED_TARGETS})
target_link_libraries(gazebo_ros_bumper ${Boost_LIBRARIES} ContactPlugin ${catkin_LIBRARIES})
