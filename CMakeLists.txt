cmake_minimum_required(VERSION 3.8)
project(speech_recognitnion_vosk)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
 set(CAMAKE_CXX_STANDARD 14)
endif()
 
# Find dependencies
find_package(std_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sobits_msgs REQUIRED)
find_package(ament_index_pythono REQUIRED)

# Include directories
include_directories(
  ${rclpy_INCLUDE_DIRS}
  ${sobits_msgs_INCLUDE_DIRS}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install scripts
install(PROGRAMS
  scripts/trigger.py
  scripts/vosk_model_downloader.py
  scripts/vosk_node.py
  scripts/vosk_server.py
  scripts/tts_engine.py
  DESTINATION lib/${PROJECT_NAME}
 )
 
 # Package
 ament_package()
  
 
  
  
