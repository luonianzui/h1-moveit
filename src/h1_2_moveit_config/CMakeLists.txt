cmake_minimum_required(VERSION 3.16)
project(h1_2_moveit_config)
cmake_policy(SET CMP0100 NEW)

find_package(ament_cmake REQUIRED)
find_package(gz_gui_vendor REQUIRED)
find_package(gz_rendering_vendor REQUIRED)
find_package(gz_transport_vendor REQUIRED)
find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-gui8 REQUIRED)
find_package(gz-rendering8 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Quick)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
add_library(CentrifugeButton SHARED
  gui/CentrifugeButton.cc gui/CentrifugeButton.hh gui/CentrifugeButton.qrc)
target_compile_features(CentrifugeButton PRIVATE cxx_std_17)
target_link_libraries(CentrifugeButton PRIVATE
  gz-gui8::gz-gui8 gz-rendering8::gz-rendering8
  gz-transport13::gz-transport13 Qt5::Core Qt5::Quick)
install(TARGETS CentrifugeButton LIBRARY DESTINATION lib)
add_library(CentrifugeInterlock SHARED systems/CentrifugeInterlock.cc)
target_compile_features(CentrifugeInterlock PRIVATE cxx_std_17)
target_link_libraries(CentrifugeInterlock PRIVATE gz-sim8::gz-sim8 gz-transport13::gz-transport13)
install(TARGETS CentrifugeInterlock LIBRARY DESTINATION lib)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

install(PROGRAMS scripts/centrifuge_lid.py scripts/centrifuge_button.py
  scripts/centrifuge_contact_button.py scripts/centrifuge_press_plan.py
  scripts/centrifuge_robot_cycle.py
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(centrifuge_button test/test_centrifuge_button.py)
  ament_add_pytest_test(centrifuge_model test/test_centrifuge_model.py)
  ament_add_pytest_test(centrifuge_contact_button test/test_centrifuge_contact_button.py)
  ament_add_pytest_test(centrifuge_robot_cycle test/test_centrifuge_robot_cycle.py)
  ament_add_pytest_test(lab_48box test/test_lab_48box.py)
  add_executable(test_centrifuge_safety test/test_centrifuge_safety.cc)
  target_compile_features(test_centrifuge_safety PRIVATE cxx_std_17)
  add_test(NAME centrifuge_safety COMMAND test_centrifuge_safety)
endif()

ament_package()
