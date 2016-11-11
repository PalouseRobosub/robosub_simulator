add_library(thruster SHARED thruster.cpp)
target_link_libraries(thruster ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
