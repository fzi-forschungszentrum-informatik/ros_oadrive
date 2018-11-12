# -- BEGIN LICENSE BLOCK ----------------------------------------------
# This program is free software licensed under the CDDL
# (COMMON DEVELOPMENT AND DISTRIBUTION LICENSE Version 1.0).
# You can find a copy of this license in LICENSE in the top
# directory of the source code.
#
# © Copyright 2018 FZI Forschungszentrum Informatik, Karlsruhe, Germany
# -- END LICENSE BLOCK ------------------------------------------------

add_executable(${PROJECT_NAME}-test
  UnitTests.cpp
  MultiTrajectoryConverterTest.cpp
  Pose2dConverterTest.cpp
  ${PROJECT_SOURCE_DIR}/src/converter/Pose2dConverter.cpp
)

add_dependencies(${PROJECT_NAME}-test ${PROJECT_NAME}_gencpp)

target_link_libraries(${PROJECT_NAME}-test
  ${catkin_LIBRARIES}
  ${icl_core_LIBRARIES}
  ${oadrive_LIBRARIES}
  ${GTEST_BOTH_LIBRARIES}
)
