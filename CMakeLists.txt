cmake_minimum_required(VERSION 2.8.3)
project(mechanical_scanning_imagery_sonar)


find_package(catkin REQUIRED COMPONENTS roslint
  image_transport 
  roscpp 
  sensor_msgs)
find_package(OpenCV REQUIRED)
find_package(GAZEBO REQUIRED)
find_package(Boost REQUIRED)

set(MECHANICAL_SCANNING_IMAGERY_SONAR "")

catkin_package(
  INCLUDE_DIRS include
    ${GAZEBO_INCLUDE_DIRS}
    ${GAZEBO_MSG_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/test/include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${GAZEBO_ODE_INCLUDE_DIRS}
  ${GAZEBO_MSG_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
set(ROSLINT_CPP_OPTS "--extensions=hpp,cpp,c,hh,cc,h")

set(MECHANICAL_SCANNING_IMAGERY_SONAR_SRCS
  src/MSISSonar.cc
  src/MSISSonarRos.cc)

set(MECHANICAL_SCANNING_IMAGERY_SONAR_HEADERS
 include/${PROJECT_NAME}/MSISSonar.hh
 include/${PROJECT_NAME}/MSISSonarRos.hh
 include/${PROJECT_NAME}/SDFTool.hh)

roslint_cpp()

roslint_cpp(${MECHANICAL_SCANNING_IMAGERY_SONAR_SRCS}
  ${MECHANICAL_SCANNING_IMAGERY_SONAR_HEADERS})

add_library(MSISSonar src/MSISSonar.cc)
target_link_libraries(MSISSonar ${GAZEBO_LIBRARIES} ${OpenCV_LIBRARIES} ${MECHANICAL_SCANNING_IMAGERY_SONAR_LIST})
list(APPEND MECHANICAL_SCANNING_IMAGERY_SONAR_LIST MSISSonar)

add_library(MechanicalScaningImagingSonarGazebo src/MSISSonarRos.cc)
target_link_libraries(MechanicalScaningImagingSonarGazebo ${catkin_LIBRARIES} ${MECHANICAL_SCANNING_IMAGERY_SONAR_LIST})
add_dependencies(MechanicalScaningImagingSonarGazebo ${catkin_EXPORTED_TARGETS})
list(APPEND MECHANICAL_SCANNING_IMAGERY_SONAR_LIST MechanicalScaningImagingSonar)

install(TARGETS ${MECHANICAL_SCANNING_IMAGERY_SONAR_LIST} ${MECHANICAL_SCANNING_IMAGERY_SONAR_LIST}
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hh"
  PATTERN "*~" EXCLUDE
)

if (CATKIN_ENABLE_TESTING)
  find_package(cmake_modules)
  find_package(rostest REQUIRED)
  find_package(TinyXML REQUIRED)
  find_package(GAZEBO REQUIRED)
  find_package(OpenCV REQUIRED)

  include_directories(include ${catkin_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
    ${TinyXML_INCLUDE_DIRS})

  link_directories(
    ${GAZEBO_LIBRARY_DIRS}
  )

  add_library(gazebo_test_fixture EXCLUDE_FROM_ALL test/ServerFixture.cc)
  target_link_libraries(gazebo_test_fixture ${GAZEBO_LIBRARIES})

  add_rostest_gtest(MSISSonar_TEST test/pure_gazebo_test.test test/MSISSonar_TEST.cc)
  target_link_libraries(MSISSonar_TEST gazebo_test_fixture ${GAZEBO_LIBRARIES} gtest ${OpenCV_LIBRARIES} ${MECHANICAL_SCANNING_IMAGERY_SONAR_LIST})
  #Add Media Path
  set_target_properties(MSISSonar_TEST PROPERTIES COMPILE_DEFINITIONS "OGRE_MEDIA_PATH=\"${PROJECT_SOURCE_DIR}/media\"")

  roslint_add_test()
endif()