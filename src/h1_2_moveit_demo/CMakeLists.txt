cmake_minimum_required(VERSION 3.16)
project(h1_2_moveit_demo)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(fcl REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gz_msgs_vendor REQUIRED)
find_package(gz_plugin_vendor REQUIRED)
find_package(gz_sim_vendor REQUIRED)
find_package(gz_transport_vendor REQUIRED)
find_package(gz-msgs REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-sim8 REQUIRED)
find_package(gz-transport REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_kinematics REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils REQUIRED)
find_package(pluginlib REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(
  h1_door_handoff_path_certificate STATIC
  src/door_handoff_binding_hash.cpp
  src/door_handoff_path_certificate.cpp
  src/door_handoff_robot_description_binding.cpp
)
target_compile_features(h1_door_handoff_path_certificate PUBLIC cxx_std_17)
target_include_directories(
  h1_door_handoff_path_certificate PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(h1_door_handoff_path_certificate rcutils)

add_executable(
  door_handoff_path_certificate_tool
  src/door_handoff_path_certificate_tool.cpp
)
target_compile_features(door_handoff_path_certificate_tool PUBLIC cxx_std_17)
target_link_libraries(
  door_handoff_path_certificate_tool
  h1_door_handoff_path_certificate
)

add_executable(dual_arm_plan_demo src/dual_arm_plan_demo.cpp)
target_compile_features(dual_arm_plan_demo PUBLIC cxx_std_17)
ament_target_dependencies(
  dual_arm_plan_demo
  moveit_ros_planning_interface
  rclcpp
)

add_executable(right_arm_pick_place_demo src/right_arm_pick_place_demo.cpp)
target_compile_features(right_arm_pick_place_demo PUBLIC cxx_std_17)
ament_target_dependencies(
  right_arm_pick_place_demo
  geometry_msgs
  moveit_msgs
  moveit_ros_planning_interface
  rclcpp
  shape_msgs
  std_msgs
  std_srvs
  visualization_msgs
)

add_executable(right_arm_return_demo src/right_arm_return_demo.cpp)
target_compile_features(right_arm_return_demo PUBLIC cxx_std_17)
ament_target_dependencies(
  right_arm_return_demo
  geometry_msgs
  moveit_msgs
  moveit_ros_planning_interface
  rclcpp
  shape_msgs
  std_msgs
  visualization_msgs
)

add_executable(right_arm_pick_place_vision_demo src/right_arm_pick_place_vision_demo.cpp)
target_compile_features(right_arm_pick_place_vision_demo PUBLIC cxx_std_17)
ament_target_dependencies(
  right_arm_pick_place_vision_demo
  geometry_msgs
  moveit_msgs
  moveit_ros_planning_interface
  rclcpp
  shape_msgs
  std_msgs
  visualization_msgs
  tf2_ros
  tf2_geometry_msgs
  trajectory_msgs
)
# Generic entry point.  The legacy right-arm executable remains for existing
# launch files; pass --ros-args -p arm_side:=left to this target for the left hand.
add_executable(arm_pick_place_vision_demo src/right_arm_pick_place_vision_demo.cpp)
target_compile_features(arm_pick_place_vision_demo PUBLIC cxx_std_17)
ament_target_dependencies(
  arm_pick_place_vision_demo
  geometry_msgs
  moveit_msgs
  moveit_ros_planning_interface
  rclcpp
  shape_msgs
  std_msgs
  visualization_msgs
  tf2_ros
  tf2_geometry_msgs
)

add_library(
  h1_gazebo_compliant_grasp_system SHARED
  src/gazebo_grasp_follower_system.cpp
)
target_compile_features(h1_gazebo_compliant_grasp_system PUBLIC cxx_std_17)
target_link_libraries(
  h1_gazebo_compliant_grasp_system
  gz-plugin2::gz-plugin2
  gz-sim8::gz-sim8
)

add_library(
  h1_incubator_latch_system SHARED
  src/incubator_latch_system.cpp
)
target_compile_features(h1_incubator_latch_system PUBLIC cxx_std_17)
target_link_libraries(
  h1_incubator_latch_system
  gz-plugin2::gz-plugin2
  gz-sim8::gz-sim8
  gz-msgs::gz-msgs
  gz-transport::gz-transport
)

add_library(
  h1_incubator_handle_release_probe_system SHARED
  src/incubator_handle_release_probe_system.cpp
)
target_compile_features(
  h1_incubator_handle_release_probe_system PUBLIC cxx_std_17
)
target_link_libraries(
  h1_incubator_handle_release_probe_system
  gz-plugin2::gz-plugin2
  gz-sim8::gz-sim8
  gz-msgs::gz-msgs
  gz-transport::gz-transport
)

add_executable(bimanual_air_handover_demo src/bimanual_air_handover_demo.cpp)
target_compile_features(bimanual_air_handover_demo PUBLIC cxx_std_17)
target_compile_definitions(
  bimanual_air_handover_demo PRIVATE
  H1_GAZEBO_GRASP_FOLLOWER_LIBRARY="$<TARGET_FILE:h1_gazebo_compliant_grasp_system>"
)
add_dependencies(bimanual_air_handover_demo h1_gazebo_compliant_grasp_system)
ament_target_dependencies(
  bimanual_air_handover_demo
  geometry_msgs
  moveit_core
  moveit_msgs
  moveit_ros_planning_interface
  rclcpp
  sensor_msgs
  shape_msgs
  std_msgs
  tf2_ros
  tf2_geometry_msgs
  trajectory_msgs
)
target_link_libraries(
  bimanual_air_handover_demo
  fcl
  gz-msgs::gz-msgs
  gz-transport::gz-transport
)

add_executable(incubator_door_open_demo src/incubator_door_open_demo.cpp)
target_compile_features(incubator_door_open_demo PUBLIC cxx_std_17)
ament_target_dependencies(
  incubator_door_open_demo
  geometric_shapes
  geometry_msgs
  moveit_core
  moveit_kinematics
  moveit_msgs
  moveit_ros_planning
  moveit_ros_planning_interface
  pluginlib
  rclcpp
  sensor_msgs
  shape_msgs
  std_msgs
  std_srvs
  trajectory_msgs
)
target_link_libraries(
  incubator_door_open_demo
  fcl
  gz-msgs::gz-msgs
  gz-transport::gz-transport
  h1_door_handoff_path_certificate
)

add_executable(
  incubator_sim_direct_fast40_demo
  src/incubator_sim_direct_fast40_demo.cpp
)
target_compile_features(incubator_sim_direct_fast40_demo PUBLIC cxx_std_17)
ament_target_dependencies(
  incubator_sim_direct_fast40_demo
  rclcpp
)
target_link_libraries(
  incubator_sim_direct_fast40_demo
  gz-msgs::gz-msgs
  gz-transport::gz-transport
)

add_executable(
  candidate3_clearance_auditor
  src/candidate3_clearance_auditor.cpp
)
target_compile_features(candidate3_clearance_auditor PUBLIC cxx_std_17)
ament_target_dependencies(
  candidate3_clearance_auditor
  moveit_core
  moveit_msgs
  rclcpp
  urdf
  yaml_cpp_vendor
)
target_link_libraries(candidate3_clearance_auditor yaml-cpp::yaml-cpp)

add_executable(
  candidate3_middle_exact_auditor
  src/candidate3_middle_exact_auditor.cpp
)
target_compile_features(candidate3_middle_exact_auditor PUBLIC cxx_std_17)
ament_target_dependencies(
  candidate3_middle_exact_auditor
  geometric_shapes
  moveit_core
  moveit_msgs
  rclcpp
  rcutils
  urdf
  yaml_cpp_vendor
)
target_link_libraries(candidate3_middle_exact_auditor fcl yaml-cpp::yaml-cpp)

add_executable(
  right_handle_path_exact_auditor
  src/right_handle_path_exact_auditor.cpp
)
target_compile_features(right_handle_path_exact_auditor PUBLIC cxx_std_17)
ament_target_dependencies(
  right_handle_path_exact_auditor
  geometric_shapes
  moveit_core
  moveit_msgs
  rclcpp
  rcutils
  urdf
  yaml_cpp_vendor
)
target_link_libraries(right_handle_path_exact_auditor fcl yaml-cpp::yaml-cpp)

add_executable(
  right_handle_live_fcl_auditor_829
  src/right_handle_live_fcl_auditor_829.cpp
)
target_compile_features(right_handle_live_fcl_auditor_829 PUBLIC cxx_std_17)
ament_target_dependencies(
  right_handle_live_fcl_auditor_829
  geometric_shapes
  moveit_core
  moveit_msgs
  rclcpp
  rcutils
  urdf
  yaml_cpp_vendor
)
target_link_libraries(right_handle_live_fcl_auditor_829 fcl yaml-cpp::yaml-cpp)

add_executable(
  right_handle_live_fcl_auditor_fast_831
  src/right_handle_live_fcl_auditor_fast_831.cpp
)
target_compile_features(right_handle_live_fcl_auditor_fast_831 PUBLIC cxx_std_17)
ament_target_dependencies(
  right_handle_live_fcl_auditor_fast_831
  geometric_shapes
  moveit_core
  moveit_msgs
  rclcpp
  rcutils
  urdf
  yaml_cpp_vendor
)
target_link_libraries(
  right_handle_live_fcl_auditor_fast_831 fcl yaml-cpp::yaml-cpp)

add_executable(
  right_handle_splice_search
  src/right_handle_splice_search.cpp
)
target_compile_features(right_handle_splice_search PUBLIC cxx_std_17)
ament_target_dependencies(
  right_handle_splice_search
  geometric_shapes
  moveit_core
  moveit_msgs
  rclcpp
  rcutils
  urdf
  yaml_cpp_vendor
)
target_link_libraries(right_handle_splice_search fcl yaml-cpp::yaml-cpp)

add_executable(
  right_handle_self_diagnostic
  src/right_handle_self_diagnostic.cpp
)
target_compile_features(right_handle_self_diagnostic PUBLIC cxx_std_17)
ament_target_dependencies(
  right_handle_self_diagnostic
  geometric_shapes
  moveit_core
  moveit_msgs
  rclcpp
  rcutils
  urdf
  yaml_cpp_vendor
)
target_link_libraries(right_handle_self_diagnostic fcl yaml-cpp::yaml-cpp)

add_executable(
  right_handle_ik_branch_search
  src/right_handle_ik_branch_search.cpp
)
target_compile_features(right_handle_ik_branch_search PUBLIC cxx_std_17)
ament_target_dependencies(
  right_handle_ik_branch_search
  geometric_shapes
  geometry_msgs
  moveit_core
  moveit_kinematics
  moveit_msgs
  pluginlib
  rclcpp
  rcutils
  urdf
  yaml_cpp_vendor
)
target_link_libraries(right_handle_ik_branch_search fcl yaml-cpp::yaml-cpp)

add_executable(
  right_handle_state_batch_auditor
  src/right_handle_state_batch_auditor.cpp
)
target_compile_features(right_handle_state_batch_auditor PUBLIC cxx_std_17)
ament_target_dependencies(
  right_handle_state_batch_auditor
  geometric_shapes
  moveit_core
  moveit_msgs
  rclcpp
  rcutils
  urdf
  yaml_cpp_vendor
)
target_link_libraries(right_handle_state_batch_auditor fcl yaml-cpp::yaml-cpp)

install(
  TARGETS
    h1_gazebo_compliant_grasp_system
    h1_incubator_latch_system
    h1_incubator_handle_release_probe_system
    dual_arm_plan_demo
    right_arm_pick_place_demo
    right_arm_return_demo
    right_arm_pick_place_vision_demo
    arm_pick_place_vision_demo
    bimanual_air_handover_demo
    incubator_door_open_demo
    incubator_sim_direct_fast40_demo
    candidate3_clearance_auditor
    candidate3_middle_exact_auditor
    right_handle_path_exact_auditor
    right_handle_live_fcl_auditor_829
    right_handle_live_fcl_auditor_fast_831
    right_handle_splice_search
    right_handle_self_diagnostic
    right_handle_ik_branch_search
    right_handle_state_batch_auditor
    door_handoff_path_certificate_tool
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS h1_door_handoff_path_certificate
  ARCHIVE DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  PROGRAMS
    scripts/bimanual_air_handover.sh
    scripts/bimanual_tabletop_relay.sh
    scripts/incubator_handle.sh
    scripts/incubator_sim_direct_fast40.sh
    scripts/incubator_sim_direct_fast60_continue.sh
    scripts/h1_joint_trajectory_asset.py
    scripts/h1_full_incubator_8251.sh
    scripts/h1_cube_support_8251.sh
    scripts/h1_door_support_8251.sh
    scripts/h1_door_support_detach_8253.sh
    scripts/h1_close_and_relock_8253.sh
    scripts/h1_ros_snapshot_8251.py
    scripts/h1_ros_snapshot_monitor_8254.py
    scripts/h1_ros_snapshot_cache_8254.sh
    scripts/h1_persistent_trajectory_runner_8254.py
    scripts/h1_closed_loop_90_8254.sh
    scripts/h1_left_rack_release_retreat_recovery.py
    scripts/h1_right_handle_executor_828.py
    scripts/h1_candidate3_middle_executor_828.py
    scripts/h1_support_orchestrator_828.py
    scripts/h1_candidate3_flow_828.py
    scripts/h1_right_handle_torso_margin_829.py
    scripts/h1_candidate3_live_certificate_829.py
    scripts/h1_candidate3_live_supervisor_829.py
    scripts/h1_robot_joint_state_relay_829.py
    scripts/h1_fast_support_daemon_831.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_gtest(
    test_door_handoff_path_certificate
    test/test_door_handoff_path_certificate.cpp
    TIMEOUT 30
  )
  target_link_libraries(
    test_door_handoff_path_certificate
    h1_door_handoff_path_certificate
  )
  ament_add_gtest(
    test_door_handoff_binding_hash
    test/test_door_handoff_binding_hash.cpp
    TIMEOUT 30
  )
  target_link_libraries(
    test_door_handoff_binding_hash
    h1_door_handoff_path_certificate
  )
  ament_add_gtest(
    test_door_handoff_robot_description_binding
    test/test_door_handoff_robot_description_binding.cpp
    TIMEOUT 30
  )
  target_link_libraries(
    test_door_handoff_robot_description_binding
    h1_door_handoff_path_certificate
  )
  ament_add_gtest(
    test_gazebo_grasp_follower_source_contract
    test/test_gazebo_grasp_follower_source_contract.cpp
    TIMEOUT 30
  )
  target_compile_definitions(
    test_gazebo_grasp_follower_source_contract PRIVATE
    H1_GAZEBO_GRASP_FOLLOWER_SOURCE="${CMAKE_CURRENT_SOURCE_DIR}/src/gazebo_grasp_follower_system.cpp"
  )
  ament_add_pytest_test(
    test_h1_ros_snapshot_cache_8254
    test/test_h1_ros_snapshot_cache_8254.py
    PYTHON_EXECUTABLE /usr/bin/python3
    TIMEOUT 30
  )
  ament_add_pytest_test(
    test_h1_persistent_trajectory_runner_8254
    test/test_h1_persistent_trajectory_runner_8254.py
    PYTHON_EXECUTABLE /usr/bin/python3
    TIMEOUT 30
  )
  ament_add_pytest_test(
    test_h1_fast_support_daemon_831
    test/test_h1_fast_support_daemon_831.py
    PYTHON_EXECUTABLE /usr/bin/python3
    TIMEOUT 30
  )
  ament_add_pytest_test(
    test_h1_door_support_detach_8253_source_contract
    test/test_h1_door_support_detach_8253_source_contract.py
    PYTHON_EXECUTABLE /usr/bin/python3
    TIMEOUT 30
  )
  ament_add_pytest_test(
    test_h1_closed_loop_90_8254_source_contract
    test/test_h1_closed_loop_90_8254_source_contract.py
    PYTHON_EXECUTABLE /usr/bin/python3
    TIMEOUT 30
  )
  ament_add_pytest_test(
    test_candidate3_middle_exact_auditor_source_contract
    test/test_candidate3_middle_exact_auditor_source_contract.py
    PYTHON_EXECUTABLE /usr/bin/python3
    TIMEOUT 30
  )
  ament_add_pytest_test(
    test_right_handle_hardened_regression
    test/test_right_handle_hardened_regression.py
    PYTHON_EXECUTABLE /usr/bin/python3
    TIMEOUT 60
  )
  ament_add_pytest_test(
    test_h1_right_handle_executor_828
    test/test_h1_right_handle_executor_828.py
    PYTHON_EXECUTABLE /usr/bin/python3
    TIMEOUT 60
  )
  ament_add_pytest_test(
    test_h1_candidate3_flow_828
    test/test_h1_candidate3_flow_828.py
    PYTHON_EXECUTABLE /usr/bin/python3
    TIMEOUT 30
  )
  ament_add_pytest_test(
    test_h1_candidate3_middle_executor_828
    test/test_h1_candidate3_middle_executor_828.py
    PYTHON_EXECUTABLE /usr/bin/python3
    TIMEOUT 30
  )
  ament_add_pytest_test(
    test_candidate3_return_plan_builder_831
    test/test_candidate3_return_plan_builder_831.py
    PYTHON_EXECUTABLE /usr/bin/python3
    TIMEOUT 30
  )
  ament_add_pytest_test(
    test_h1_candidate3_roundtrip_middle_831
    test/test_h1_candidate3_roundtrip_middle_831.py
    PYTHON_EXECUTABLE /usr/bin/python3
    TIMEOUT 30
  )
  ament_add_pytest_test(
    test_h1_candidate3_roundtrip_flow_831
    test/test_h1_candidate3_roundtrip_flow_831.py
    PYTHON_EXECUTABLE /usr/bin/python3
    TIMEOUT 30
  )
  ament_add_pytest_test(
    test_h1_support_orchestrator_828
    test/test_h1_support_orchestrator_828.py
    PYTHON_EXECUTABLE /usr/bin/python3
    TIMEOUT 30
  )
  ament_add_pytest_test(
    test_h1_right_handle_torso_margin_829
    test/test_h1_right_handle_torso_margin_829.py
    PYTHON_EXECUTABLE /usr/bin/python3
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_h1_candidate3_live_certificate_829
    test/test_h1_candidate3_live_certificate_829.py
    PYTHON_EXECUTABLE /usr/bin/python3
    TIMEOUT 60
  )
  ament_add_pytest_test(
    test_h1_candidate3_live_supervisor_829
    test/test_h1_candidate3_live_supervisor_829.py
    PYTHON_EXECUTABLE /usr/bin/python3
    TIMEOUT 60
  )
  ament_add_pytest_test(
    test_h1_robot_joint_state_relay_829
    test/test_h1_robot_joint_state_relay_829.py
    PYTHON_EXECUTABLE /usr/bin/python3
    TIMEOUT 30
  )
  ament_add_pytest_test(
    test_right_handle_live_fcl_auditor_829
    test/test_right_handle_live_fcl_auditor_829.py
    PYTHON_EXECUTABLE /usr/bin/python3
    TIMEOUT 60
  )
endif()

ament_package()
