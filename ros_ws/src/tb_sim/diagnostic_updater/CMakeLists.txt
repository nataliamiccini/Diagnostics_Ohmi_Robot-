cmake_minimum_required(VERSION 3.0.2)
project(diagnostic_updater)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED diagnostic_msgs roscpp std_msgs)

catkin_python_setup()

catkin_package(CATKIN_DEPENDS diagnostic_msgs roscpp std_msgs
               INCLUDE_DIRS include
	       LIBRARIES ${PROJECT_NAME}
)

include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(${PROJECT_NAME}_diagnostic_updater_imu src/main_imu.cpp )
add_library(${PROJECT_NAME}
  src/timestamp_status.cpp)
set_target_properties(${PROJECT_NAME}_diagnostic_updater_imu PROPERTIES OUTPUT_NAME imu_node PREFIX "")
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_diagnostic_updater_imu ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_diagnostic_updater_imu sensor_msgs_generate_messages_cpp)

add_executable(${PROJECT_NAME}_diagnostic_updater_scan src/main_scan.cpp )
set_target_properties(${PROJECT_NAME}_diagnostic_updater_scan PROPERTIES OUTPUT_NAME scan_node PREFIX "")
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_diagnostic_updater_scan ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_diagnostic_updater_scan sensor_msgs_generate_messages_cpp)

add_executable(${PROJECT_NAME}_diagnostic_updater_joint_states src/main_joint_states.cpp )
set_target_properties(${PROJECT_NAME}_diagnostic_updater_joint_states PROPERTIES OUTPUT_NAME joint_states_node PREFIX "")
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_diagnostic_updater_joint_states ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_diagnostic_updater_joint_states sensor_msgs_generate_messages_cpp)

add_executable(${PROJECT_NAME}_diagnostic_updater_tele_cam src/main_tele_cam.cpp )
set_target_properties(${PROJECT_NAME}_diagnostic_updater_tele_cam PROPERTIES OUTPUT_NAME tele_cam_node PREFIX "")
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_diagnostic_updater_tele_cam ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_diagnostic_updater_tele_cam sensor_msgs_generate_messages_cpp)

add_executable(${PROJECT_NAME}_diagnostic_updater_nav_cam src/main_nav_cam.cpp )
set_target_properties(${PROJECT_NAME}_diagnostic_updater_nav_cam PROPERTIES OUTPUT_NAME nav_cam_node PREFIX "")
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_diagnostic_updater_nav_cam ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_diagnostic_updater_nav_cam sensor_msgs_generate_messages_cpp)


#set(LOCAL_GTEST_DIR "gtest-1.7.0")
# gtest could be included in ${catkin_INCLUDE_DIRS}, prepend local gtest include directory so it's searched first
#include_directories(BEFORE ${LOCAL_GTEST_DIR}/include)

#if(CATKIN_ENABLE_TESTING)
#  find_package(rostest REQUIRED)
#  add_rostest_gtest(diagnostic_updater_test test/diagnostic_updater_test.xml test/diagnostic_updater_test.cpp)
#  add_rostest(test/diagnostic_updater_fast_test.xml)
#  target_link_libraries(diagnostic_updater_test ${catkin_LIBRARIES} ${PROJECT_NAME})
#endif()

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(diagnostic_updater_test test/diagnostic_updater_test.xml test/diagnostic_updater_test.cpp)
  add_rostest(test/diagnostic_updater_fast_test.xml)
  target_link_libraries(diagnostic_updater_test ${catkin_LIBRARIES} ${PROJECT_NAME})
endif()

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
